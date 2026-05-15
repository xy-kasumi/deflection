// Pre-physics structural layer: transform a serial-chain `Problem` into a
// flat node-and-segment graph.
//
// Each "user beam" gets split at every attachment offset on it — the offset
// where the next beam branches off, every load's offset, every query's
// offset, plus the two endpoints. Adjacent split offsets become a `Segment`,
// every offset becomes a `Node`, and loads/queries get rewritten as node
// references.
//
// The payoff for the compliance solver downstream: every load and query
// lives at a node, every segment is a textbook cantilever loaded at its tip
// from downstream. There is no "load is past the chain-exit on this beam"
// edge case — that case is dissolved into "two segments with a load node
// between them." The chain-exit bookkeeping that this file replaces is the
// `chainExit_mm` array in the previous `compliance.ts`.
//
// The graph today is a path (DSL is strictly serial), but the data shape is
// the natural tree generalization: switching to multi-child branching only
// changes `buildSegmentation`, not anything downstream.
//
// Caller invariants (mirroring `simulate.validate`):
//   - beams[0].frame.origin_mm is at world origin.
//   - beams[i>0].frame.origin_mm lies on beams[i-1]'s axis within [0, L].
// Both are checked here; failures return a `SimError` (re-exported through
// simulate).

import type { Frame, Material, Problem, Section } from './problem';
import type { Vec3 } from './math';
import type { SimError } from './simulate';

export interface Node {
  pos_mm: Vec3;
}

export interface Segment {
  /** Proximal end (closer to root). */
  baseNodeIx: number;
  /** Distal end. */
  tipNodeIx: number;
  /** Source user-beam — used by the compliance output to aggregate per beam. */
  beamIx: number;
  /** Local frame: axes inherited from the user beam; origin sits at baseNode. */
  frame: Frame;
  length_mm: number;
  section: Section;
  material: Material;
}

export interface SegmentedLoad {
  /** Node carrying the load. */
  nodeIx: number;
  /** Back-pointer to Problem.loads — preserves caller ordering on output. */
  loadIx: number;
  Fmax_N: number;
}

export interface SegmentedQuery {
  nodeIx: number;
  queryIx: number;
}

export interface Segmentation {
  nodes: Node[];
  segments: Segment[];
  loads: SegmentedLoad[];
  queries: SegmentedQuery[];
  /** Node at offset 0 of beam i, ie. its attachment to its parent. beamStartNode[0] is always the root (world origin). */
  beamStartNode: number[];
  /** Node at offset length_mm of beam i (its physical tip). */
  beamEndNode: number[];
}

// Two nodes are deemed identical if their offsets agree within this; matches
// the `CONNECT_EPS_MM` tolerance used by `simulate.validate`.
const SPLIT_EPS_MM = 1e-6;

export function buildSegmentation(problem: Problem): Segmentation | SimError {
  const beams = problem.beams;
  const nodes: Node[] = [];
  const segments: Segment[] = [];
  const segLoads: SegmentedLoad[] = [];
  const segQueries: SegmentedQuery[] = [];
  const beamStartNode: number[] = [];
  const beamEndNode: number[] = [];

  if (beams.length === 0) {
    return { nodes, segments, loads: segLoads, queries: segQueries, beamStartNode, beamEndNode };
  }

  const o = beams[0]!.frame.origin_mm;
  if (Math.hypot(o[0], o[1], o[2]) > SPLIT_EPS_MM) {
    return {
      kind: 'error',
      code: 'root-not-at-origin',
      message: `root beam must be clamped at the world origin; got [${o.join(', ')}]`,
    };
  }

  // sAttach[i] = offset on beam i-1 where beam i attaches; undefined for i=0.
  const sAttach = new Array<number | undefined>(beams.length);
  for (let i = 1; i < beams.length; i++) {
    const parent = beams[i - 1]!;
    const child = beams[i]!.frame.origin_mm;
    const a = parent.frame.axial;
    const rel: Vec3 = [
      child[0] - parent.frame.origin_mm[0],
      child[1] - parent.frame.origin_mm[1],
      child[2] - parent.frame.origin_mm[2],
    ];
    const s = rel[0] * a[0] + rel[1] * a[1] + rel[2] * a[2];
    const perp = Math.hypot(rel[0] - a[0] * s, rel[1] - a[1] * s, rel[2] - a[2] * s);
    if (perp > SPLIT_EPS_MM || s < -SPLIT_EPS_MM || s > parent.length_mm + SPLIT_EPS_MM) {
      return {
        kind: 'error',
        code: 'beams-disconnected',
        message: `beam ${i} does not attach to beam ${i - 1}'s axis`,
      };
    }
    sAttach[i] = clampToRange(s, 0, parent.length_mm);
  }

  // Root node — every chain shares it.
  beamStartNode[0] = pushNode(nodes, beams[0]!.frame.origin_mm);

  for (let i = 0; i < beams.length; i++) {
    const beam = beams[i]!;
    const L = beam.length_mm;

    // Split offsets along this beam: endpoints, the next beam's attachment
    // (if any), and any load/query that lives on this beam.
    const offsets = new Set<number>([0, L]);
    if (i + 1 < beams.length) offsets.add(sAttach[i + 1]!);
    for (const ld of problem.loads) if (ld.beamIx === i) offsets.add(clampToRange(ld.offset_mm, 0, L));
    for (const q of problem.queries) if (q.beamIx === i) offsets.add(clampToRange(q.offset_mm, 0, L));

    // Dedupe within epsilon; keep ascending order.
    const sorted = dedupeSorted([...offsets].sort((a, b) => a - b));

    // Map each beam-local offset to a global node index. Offset 0 reuses
    // `beamStartNode[i]` (which is either the root or a node already created
    // by the previous beam at its sAttach point).
    const offsetToNode = new Map<number, number>();
    offsetToNode.set(sorted[0]!, beamStartNode[i]!);
    for (let k = 1; k < sorted.length; k++) {
      const s = sorted[k]!;
      offsetToNode.set(s, pushNode(nodes, [
        beam.frame.origin_mm[0] + beam.frame.axial[0] * s,
        beam.frame.origin_mm[1] + beam.frame.axial[1] * s,
        beam.frame.origin_mm[2] + beam.frame.axial[2] * s,
      ]));
    }

    beamEndNode[i] = offsetToNode.get(sorted[sorted.length - 1]!)!;
    // The next beam's start node was created here at offset sAttach[i+1].
    if (i + 1 < beams.length) {
      beamStartNode[i + 1] = lookupOffsetNode(offsetToNode, sAttach[i + 1]!);
    }

    // Segments: one per consecutive-offset pair.
    for (let k = 0; k + 1 < sorted.length; k++) {
      const s0 = sorted[k]!, s1 = sorted[k + 1]!;
      segments.push({
        baseNodeIx: offsetToNode.get(s0)!,
        tipNodeIx: offsetToNode.get(s1)!,
        beamIx: i,
        frame: {
          origin_mm: nodes[offsetToNode.get(s0)!]!.pos_mm,
          ex: beam.frame.ex,
          ey: beam.frame.ey,
          axial: beam.frame.axial,
        },
        length_mm: s1 - s0,
        section: beam.section,
        material: beam.material,
      });
    }

    // Resolve this beam's loads/queries to node indices.
    for (let ix = 0; ix < problem.loads.length; ix++) {
      const ld = problem.loads[ix]!;
      if (ld.beamIx !== i) continue;
      segLoads.push({
        nodeIx: lookupOffsetNode(offsetToNode, clampToRange(ld.offset_mm, 0, L)),
        loadIx: ix,
        Fmax_N: ld.Fmax_N,
      });
    }
    for (let ix = 0; ix < problem.queries.length; ix++) {
      const q = problem.queries[ix]!;
      if (q.beamIx !== i) continue;
      segQueries.push({
        nodeIx: lookupOffsetNode(offsetToNode, clampToRange(q.offset_mm, 0, L)),
        queryIx: ix,
      });
    }
  }

  return { nodes, segments, loads: segLoads, queries: segQueries, beamStartNode, beamEndNode };
}

function pushNode(nodes: Node[], pos: Vec3): number {
  nodes.push({ pos_mm: [pos[0], pos[1], pos[2]] });
  return nodes.length - 1;
}

function dedupeSorted(xs: number[]): number[] {
  const out: number[] = [];
  for (const x of xs) {
    if (out.length === 0 || x - out[out.length - 1]! > SPLIT_EPS_MM) out.push(x);
  }
  return out;
}

function clampToRange(x: number, lo: number, hi: number): number {
  return Math.max(lo, Math.min(hi, x));
}

// Map lookup with epsilon-fallback: floating-point keys can disagree at the
// last bit when we e.g. push `sAttach[i+1]` and a load happens to be at the
// same offset.
function lookupOffsetNode(map: Map<number, number>, s: number): number {
  const hit = map.get(s);
  if (hit !== undefined) return hit;
  for (const [key, ix] of map) {
    if (Math.abs(key - s) <= SPLIT_EPS_MM) return ix;
  }
  throw new Error(`segmentation: no node for offset ${s}`);
}
