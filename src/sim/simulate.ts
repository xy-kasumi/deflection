import type { Beam, DeflectionQuery, Load, Problem, Vec3 } from './problem';
import { buildCompliances } from './compliance';
import { directionalFor, type Directional } from './directional';
import { decompose } from './decompose';

export type SimOutcome = SimResult | SimError;

export interface SimResult {
  kind: 'ok';
  queryResults: DeflectionQueryResult[]; // 1:1 with Problem.queries, same order
}

export interface SimError {
  kind: 'error';
  code: 'root-not-at-origin' | 'beams-disconnected' | 'support-singular';
  message: string;
}

// Per-query result. `deflection_mm` is the full δ(dir) distribution; the
// headline (d*, δ_max) comes from `deflection_mm.max()`. `loads` / `beams`
// break that max down so a query can be selected in the UI without re-running
// the math. All vectors here are world-frame; `pos_mm` is the undeformed
// reference position.
export interface DeflectionQueryResult {
  queryIx: number;
  query: DeflectionQuery;
  pos_mm: Vec3;
  deflection_mm: Directional;
  loads: LoadContribution[];
  beams: BeamContribution[];
}

export interface LoadContribution {
  loadIx: number;
  load: Load;
  delta_mm: number; // signed contribution along deflection_mm.max()
}

export interface BeamContribution {
  beamIx: number;
  beam: Beam;
  delta_mm: number; // signed total along deflection_mm.max() (= sum of the 3 below)
  delta_mm_bendIx: number;
  delta_mm_bendIy: number;
  delta_mm_torsionJ: number;
}

// Connectivity tolerance: walker-built chains are exact to float precision, so
// anything beyond this is a genuinely disconnected input.
const CONNECT_EPS_MM = 1e-6;

// Solve the deflection problem. Either succeeds with one result per query, or
// fails — there is no silent fallback. See SimError for the failure modes.
export function simulate(problem: Problem): SimOutcome {
  const invalid = validate(problem);
  if (invalid) return invalid;

  const beams = problem.beams;
  if (beams.length === 0) {
    return { kind: 'ok', queryResults: [] };
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
      deflection_mm: deflection,
      loads,
      beams: beamContribs,
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
