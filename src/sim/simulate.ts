import type { DeflectionQuery, Problem } from './problem';
import { buildCompliances } from './compliance';
import type { Compliances, Mode } from './compliance';
import { makeConvexEnvelope, type ConvexEnvelope, type Vec3, type Mat3 } from './math';

export type SimOutcome = SimResult | SimError;

export interface SimResult {
  kind: 'ok';
  /** 1:1 with Problem.queries, same order */
  queryResults: DeflectionQueryResult[];
}

export interface SimError {
  kind: 'error';
  code: 'root-not-at-origin' | 'beams-disconnected' | 'support-singular';
  message: string;
}

/**
 * Per-query result. All vectors here are world-frame.
 */
export interface DeflectionQueryResult {
  /** Index into Problem.queries (and into SimResult.queryResults). */
  queryIx: number;
  query: DeflectionQuery;
  /** Undeformed position. */
  pos_mm: Vec3;
  /** Set of possible translation-deflection vectors at this query (mm). */
  deflection_mm: ConvexEnvelope;
  /** Set of possible linearized rotation vectors at this query (rad). */
  rotation_rad: ConvexEnvelope;
  /** Per-beam (and per-mode) δ in isolation; do not sum to deflection_mm. */
  beamDeflections: BeamDeflection[];
}

export interface BeamDeflection {
  beamIx: number;
  byMode: {
    mode: Mode;
    /** δ from this beam/mode in isolation (mm). */
    deflection_mm: ConvexEnvelope;
    /**
     * Single-load, single-(beam, mode) envelope. Lets a caller attribute the
     * (b, m)-summed pessimistic bound back to loads: at each (b, m)'s
     * furthest direction d*, each load's `support(d*)` is its share, and
     * Σ_{b, m, p} of those equals the bound. Values are mm.
     */
    perLoad: { loadIx: number; deflection_mm: ConvexEnvelope }[];
  }[];
}

/**
 * Translation-deflection envelope for query q. Each per-load compliance Cₚ
 * is scaled by F_max,p to form an ellipsoid shape matrix; the envelope is
 * their Minkowski sum.
 */
function deflectionEnvelopeFor(c: Compliances, queryIx: number): ConvexEnvelope {
  const ts: Mat3[] = [];
  for (const t of c.totals) {
    if (t.queryIx !== queryIx) continue;
    ts.push(scaleMat(t.C, c.loadFmax_N[t.loadIx] ?? 0));
  }
  return makeConvexEnvelope(ts);
}

/** Linearized rotation envelope for query q (rad). */
function rotationEnvelopeFor(c: Compliances, queryIx: number): ConvexEnvelope {
  const ts: Mat3[] = [];
  for (const t of c.rotationTotals) {
    if (t.queryIx !== queryIx) continue;
    ts.push(scaleMat(t.C, c.loadFmax_N[t.loadIx] ?? 0));
  }
  return makeConvexEnvelope(ts);
}

/**
 * Per-(beam, mode) envelopes at query q, worst-cased independently. These
 * do NOT sum to the whole-structure δ_q (Σ ≥ δ, triangle inequality); each is
 * honest only on its own. Per-mode `perLoad` keeps single-load envelopes so
 * callers can attribute the (b, m) furthest-point back to loads.
 */
function beamEnvelopes(c: Compliances, queryIx: number): BeamDeflection[] {
  const byBeam = new Map<number, Map<Mode, Map<number, Mat3>>>();
  for (const e of c.entries) {
    if (e.queryIx !== queryIx) continue;
    let perMode = byBeam.get(e.beamIx);
    if (!perMode) { perMode = new Map(); byBeam.set(e.beamIx, perMode); }
    let pm = perMode.get(e.mode);
    if (!pm) { pm = new Map(); perMode.set(e.mode, pm); }
    accumMat(pm, e.loadIx, e.C);
  }

  return [...byBeam.entries()]
    .sort(([a], [z]) => a - z)
    .map(([beamIx, perMode]) => ({
      beamIx,
      byMode: [...perMode.entries()].map(([mode, m]) => {
        const sorted = [...m.entries()].sort(([a], [z]) => a - z);
        const ts = sorted.map(([loadIx, C]) =>
          scaleMat(C, c.loadFmax_N[loadIx] ?? 0),
        );
        return {
          mode,
          deflection_mm: makeConvexEnvelope(ts),
          perLoad: sorted.map(([loadIx], i) => ({
            loadIx,
            deflection_mm: makeConvexEnvelope([ts[i]!]),
          })),
        };
      }),
    }));
}

// Accumulate C into the per-load matrix bucket (creating a zero one if absent).
function accumMat(m: Map<number, Mat3>, loadIx: number, C: Mat3): void {
  let cur = m.get(loadIx);
  if (!cur) { cur = [0, 0, 0, 0, 0, 0, 0, 0, 0]; m.set(loadIx, cur); }
  for (let i = 0; i < 9; i++) (cur[i] as number) += C[i] as number;
}

function scaleMat(M: Mat3, s: number): Mat3 {
  return [
    M[0] * s, M[1] * s, M[2] * s,
    M[3] * s, M[4] * s, M[5] * s,
    M[6] * s, M[7] * s, M[8] * s,
  ];
}

// Connectivity tolerance: walker-built chains are exact to float precision, so
// anything beyond this is a genuinely disconnected input.
const CONNECT_EPS_MM = 1e-6;

/**
 * Solve the deflection problem. Either succeeds with one result per query, or
 * fails — there is no silent fallback. See SimError for the failure modes.
 */
export function simulate(problem: Problem): SimOutcome {
  const invalid = validate(problem);
  if (invalid) return invalid;

  const beams = problem.beams;
  if (beams.length === 0) {
    return { kind: 'ok', queryResults: [] };
  }

  const compliances = buildCompliances(problem);
  if ('kind' in compliances) return compliances; // SimError

  const queryResults: DeflectionQueryResult[] = [];
  for (let queryIx = 0; queryIx < problem.queries.length; queryIx++) {
    const query = problem.queries[queryIx]!;
    const beam = beams[query.beamIx]!;
    const worldPos: Vec3 = [
      beam.frame.origin_mm[0] + beam.frame.axial[0] * query.offset_mm,
      beam.frame.origin_mm[1] + beam.frame.axial[1] * query.offset_mm,
      beam.frame.origin_mm[2] + beam.frame.axial[2] * query.offset_mm,
    ];
    queryResults.push({
      queryIx,
      query,
      pos_mm: worldPos,
      deflection_mm: deflectionEnvelopeFor(compliances, queryIx),
      rotation_rad: rotationEnvelopeFor(compliances, queryIx),
      beamDeflections: beamEnvelopes(compliances, queryIx),
    });
  }

  return { kind: 'ok', queryResults };
}

// Validate sim/'s structural preconditions: the chain is serial, clamped at
// the world origin, with each beam attached on its parent's axis segment.
function validate(problem: Problem): SimError | null {
  const beams = problem.beams;
  if (beams.length === 0) return null;

  const o = beams[0]!.frame.origin_mm;
  if (Math.hypot(o[0], o[1], o[2]) > CONNECT_EPS_MM) {
    return {
      kind: 'error',
      code: 'root-not-at-origin',
      message: `root beam must be clamped at the world origin; got [${o.join(', ')}]`,
    };
  }

  for (let i = 1; i < beams.length; i++) {
    const parent = beams[i - 1]!;
    const child = beams[i]!.frame.origin_mm;
    const p = parent.frame.origin_mm;
    const a = parent.frame.axial;
    const rel: Vec3 = [child[0] - p[0], child[1] - p[1], child[2] - p[2]];
    const s = rel[0] * a[0] + rel[1] * a[1] + rel[2] * a[2]; // projection onto the unit axis
    const perp = Math.hypot(rel[0] - a[0] * s, rel[1] - a[1] * s, rel[2] - a[2] * s);
    if (perp > CONNECT_EPS_MM || s < -CONNECT_EPS_MM || s > parent.length_mm + CONNECT_EPS_MM) {
      return {
        kind: 'error',
        code: 'beams-disconnected',
        message: `beam ${i} does not attach to beam ${i - 1}'s axis`,
      };
    }
  }
  return null;
}
