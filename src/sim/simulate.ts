import type { Beam, DeflectionQuery, Load, Problem, Vec3 } from './problem';
import { buildCompliances } from './compliance';
import type { Compliances, Mat3, Mode } from './compliance';
import { directionalFor, beamDirectionals, type Directional } from './directional';
import { decompose } from './decompose';

export type SimOutcome = SimResult | SimError;

export interface SimResult {
  kind: 'ok';
  /** 1:1 with Problem.queries, same order */
  queryResults: DeflectionQueryResult[];
  /**
   * Apply concrete per-load forces → fully determined, trivially decomposable
   * deflections. Reuses cached compliances; no structural re-solve.
   */
  under(forces: Force[]): DeterminedResult;
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
  queryIx: number;
  query: DeflectionQuery;
  /** Undeformed position. */
  pos_mm: Vec3;
  /** Full δ(dir) distribution. */
  deflection: Directional;
  /** Per-beam (and per-mode) δ in isolation; do not sum to deflection. */
  beamDeflections: BeamDeflection[];
  /** `deflection.max()` broken down per contributing load. */
  loads: LoadContribution[];
  /** `deflection.max()` broken down per contributing beam. */
  beams: BeamContribution[];
  /** The per-load worst-case forces that realize δ toward `dir`. */
  forcesAt(dir: Vec3): Force[];
}

export interface LoadContribution {
  loadIx: number;
  load: Load;
  /** signed contribution along deflection.max() */
  delta_mm: number;
}

export interface BeamContribution {
  beamIx: number;
  beam: Beam;
  /** signed total along deflection.max() (= sum of the 3 below) */
  delta_mm: number;
  delta_mm_bendIx: number;
  delta_mm_bendIy: number;
  delta_mm_torsionJ: number;
}

export interface BeamDeflection {
  beamIx: number;
  /** δ from this beam's compliance alone (worst-cased independently). */
  deflection: Directional;
  byMode: { mode: Mode; deflection: Directional }[];
}

export interface Force {
  loadIx: number;
  /** World-frame force vector applied at the load node. */
  F_N: Vec3;
}

/** Deflection at every query under one concrete set of per-load forces. */
export interface DeterminedResult {
  /** The applied forces, echoed back. */
  forces: Force[];
  /** 1:1 with Problem.queries, same order. */
  queryResults: DeterminedDeflection[];
}

/**
 * A fully determined deflection: forces are concrete, so the response is a
 * linear system and every contribution list below sums exactly to `vector_mm`.
 */
export interface DeterminedDeflection {
  queryIx: number;
  pos_mm: Vec3;
  vector_mm: Vec3;
  perLoad: { loadIx: number; vector_mm: Vec3 }[];
  /** Ordered root → tip. */
  perBeam: {
    beamIx: number;
    vector_mm: Vec3;
    byMode: { mode: Mode; vector_mm: Vec3 }[];
  }[];
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
    return { kind: 'ok', queryResults: [], under: (forces) => ({ forces, queryResults: [] }) };
  }

  const compliances = buildCompliances(problem);
  if ('kind' in compliances) return compliances; // SimError

  // Per-query Directional.max() gives (d*, δ_max). Decompose along d* eagerly
  // so the UI can switch the selected query without recomputing.
  const queryResults: DeflectionQueryResult[] = [];
  for (let queryIx = 0; queryIx < problem.queries.length; queryIx++) {
    const query = problem.queries[queryIx]!;
    const beam = beams[query.beamIx]!;
    const worldPos: Vec3 = [
      beam.frame.origin_mm[0] + beam.frame.axial[0] * query.offset_mm,
      beam.frame.origin_mm[1] + beam.frame.axial[1] * query.offset_mm,
      beam.frame.origin_mm[2] + beam.frame.axial[2] * query.offset_mm,
    ];
    const deflection = directionalFor(compliances, queryIx);
    const { dir, value } = deflection.max();

    let loads: LoadContribution[] = [];
    let beamContribs: BeamContribution[] = [];
    if (value > 0 && compliances.loadNodes.length > 0) {
      const decomp = decompose(compliances, queryIx, dir);
      loads = decomp.perLoad.map((pl) => ({
        loadIx: pl.loadIx,
        load: problem.loads[pl.loadIx]!,
        delta_mm: pl.delta_mm,
      }));
      beamContribs = decomp.perBeam.map((pb) => ({
        beamIx: pb.beamIx,
        beam: beams[pb.beamIx]!,
        delta_mm: pb.total_mm,
        delta_mm_bendIx: pb.bendIx_mm,
        delta_mm_bendIy: pb.bendIy_mm,
        delta_mm_torsionJ: pb.torsionJ_mm,
      }));
    }

    queryResults.push({
      queryIx,
      query,
      pos_mm: worldPos,
      deflection: deflection,
      beamDeflections: beamDirectionals(compliances, queryIx),
      loads,
      beams: beamContribs,
      forcesAt: (dir) => computeForces(compliances, queryIx, dir),
    });
  }

  return {
    kind: 'ok',
    queryResults,
    under: (forces) => applyForces(compliances, queryResults, forces),
  };
}

// Apply concrete per-load forces over the cached compliances. Pure linear
// superposition: each entry contributes C·F, accumulated per load / beam /
// (beam, mode). No re-solve.
function applyForces(
  c: Compliances,
  queryResults: DeflectionQueryResult[],
  forces: Force[],
): DeterminedResult {
  const F: Vec3[] = c.loadNodes.map(() => [0, 0, 0]);
  for (const f of forces) {
    if (f.loadIx >= 0 && f.loadIx < F.length) F[f.loadIx] = f.F_N;
  }

  const out: DeterminedDeflection[] = queryResults.map((qr) => {
    const q = qr.queryIx;
    const total: Vec3 = [0, 0, 0];
    const perLoadMap = new Map<number, Vec3>();
    const perBeamMap = new Map<number, { vector_mm: Vec3; byMode: Map<Mode, Vec3> }>();

    for (const e of c.entries) {
      if (e.queryIx !== q) continue;
      const cf = matVec(e.C, F[e.loadIx]!);
      addInto(total, cf);

      let pl = perLoadMap.get(e.loadIx);
      if (!pl) { pl = [0, 0, 0]; perLoadMap.set(e.loadIx, pl); }
      addInto(pl, cf);

      let pb = perBeamMap.get(e.beamIx);
      if (!pb) { pb = { vector_mm: [0, 0, 0], byMode: new Map() }; perBeamMap.set(e.beamIx, pb); }
      addInto(pb.vector_mm, cf);
      let bm = pb.byMode.get(e.mode);
      if (!bm) { bm = [0, 0, 0]; pb.byMode.set(e.mode, bm); }
      addInto(bm, cf);
    }

    return {
      queryIx: q,
      pos_mm: qr.pos_mm,
      vector_mm: total,
      perLoad: [...perLoadMap.entries()]
        .sort(([a], [b]) => a - b)
        .map(([loadIx, vector_mm]) => ({ loadIx, vector_mm })),
      perBeam: [...perBeamMap.entries()]
        .sort(([a], [b]) => a - b)
        .map(([beamIx, v]) => ({
          beamIx,
          vector_mm: v.vector_mm,
          byMode: [...v.byMode.entries()].map(([mode, vector_mm]) => ({ mode, vector_mm })),
        })),
    };
  });

  return { forces, queryResults: out };
}

// The per-load worst-case forces F_p* that realize δ_q toward `dir`: each load
// independently maximizes its own contribution along `dir`.
function computeForces(c: Compliances, queryIx: number, dir: Vec3): Force[] {
  const dn = normalizeOrZero(dir);
  const forces: Force[] = [];
  for (const t of c.totals) {
    if (t.queryIx !== queryIx) continue;
    const Fmax = c.loadFmax_N[t.loadIx] ?? 0;
    let F_N: Vec3 = [0, 0, 0];
    if (dn) {
      const v = matVecT(t.C, dn);
      const mag = Math.hypot(v[0], v[1], v[2]);
      if (mag > 1e-30) {
        F_N = [Fmax * v[0] / mag, Fmax * v[1] / mag, Fmax * v[2] / mag];
      }
    }
    forces.push({ loadIx: t.loadIx, F_N });
  }
  return forces;
}

function matVec(M: Mat3, v: Vec3): Vec3 {
  return [
    M[0] * v[0] + M[1] * v[1] + M[2] * v[2],
    M[3] * v[0] + M[4] * v[1] + M[5] * v[2],
    M[6] * v[0] + M[7] * v[1] + M[8] * v[2],
  ];
}

function matVecT(M: Mat3, v: Vec3): Vec3 {
  return [
    M[0] * v[0] + M[3] * v[1] + M[6] * v[2],
    M[1] * v[0] + M[4] * v[1] + M[7] * v[2],
    M[2] * v[0] + M[5] * v[1] + M[8] * v[2],
  ];
}

function normalizeOrZero(v: Vec3): Vec3 | null {
  const m = Math.hypot(v[0], v[1], v[2]);
  if (m < 1e-30) return null;
  return [v[0] / m, v[1] / m, v[2] / m];
}

function addInto(a: Vec3, b: Vec3): void {
  a[0] += b[0];
  a[1] += b[1];
  a[2] += b[2];
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
