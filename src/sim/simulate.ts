import type { DeflectionQuery, Problem, Vec3 } from './problem';
import { buildCompliances } from './compliance';
import type { Mode } from './compliance';
import { directionalFor, rotationFor, beamDirectionals, type Directional } from './directional';

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
  /** Translation worst-case distribution; .at/.max/.sample values are mm. */
  deflection_mm: Directional;
  /** Rotation worst-case distribution; .at/.max/.sample values are rad. */
  rotation_rad: Directional;
  /** Per-beam (and per-mode) δ in isolation; do not sum to deflection_mm. */
  beamDeflections: BeamDeflection[];
}

export interface BeamDeflection {
  beamIx: number;
  byMode: {
    mode: Mode;
    /** δ from this beam/mode in isolation; .at/.max values are mm. */
    deflection_mm: Directional;
    /**
     * Single-load, single-(beam, mode) δ_p(d) = F_p · |C_{b,m,p}^T d|. Lets a
     * caller attribute the (b, m)-summed pessimistic bound back to loads: at
     * each (b, m)'s argmax d*, F_p · |C_{b,m,p}^T d*| is load p's share, and
     * Σ_{b, m, p} of those equals the bound. Values are mm.
     */
    perLoad: { loadIx: number; deflection_mm: Directional }[];
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
      deflection_mm: directionalFor(compliances, queryIx),
      rotation_rad: rotationFor(compliances, queryIx),
      beamDeflections: beamDirectionals(compliances, queryIx),
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
