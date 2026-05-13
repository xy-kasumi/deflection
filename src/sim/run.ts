import type { Diagnostic } from '../dsl/diagnostics';
import type { Structure } from '../dsl/parse';
import type { BeamNode, Vec3 } from '../walker';
import { buildCompliances, getSupportKind } from './compliance';
import { directionalFor, type Directional } from './directional';
import { attribute, type PerBeamAttrib, type PerLoadAttrib } from './attribute';

// Per-query result. `worldPos_undeformed` anchors the directional; `directional`
// is the full δ(d) function the scene can sample for surface rendering.
export interface NodeDeflectionResult {
  queryIx: number;
  worldPos_undeformed: Vec3;
  d_star: Vec3;
  delta_max_mm: number;
  directional: Directional;
}

export interface SimForce {
  loadIx: number;
  beamIx: number;
  offset_mm: number;
  Fmax_N: number;
  delta_mm: number; // signed contribution at the headline query along its d*
  fraction: number;
}

export interface SimBeam {
  beamIx: number;
  total_fraction: number;
  axial_fraction: number;
  bendIx_fraction: number;
  bendIy_fraction: number;
  torsion_fraction: number;
}

export type DisplayScale = 1 | 10 | 100 | 1000;

export interface SimResult {
  nodes: NodeDeflectionResult[];
  // Index into `nodes` of the node with the largest δ_max — the one whose
  // direction d* drives the attribution breakdown and the tip arrow.
  maxNodeIx: number;
  display_scale: DisplayScale;
  forces: SimForce[];
  beams: SimBeam[];
  diagnostics: Diagnostic[];
}

// Orchestrates compliance build → per-query directional max → attribution at
// the worst-case query node. Constrained-to-zero nodes (the support(both) tip)
// are hidden from `nodes` even though compliance keeps them for the solve.
export function runSim(beams: BeamNode[], structure: Structure): SimResult {
  const diagnostics: Diagnostic[] = [];
  if (beams.length === 0) {
    return emptyResult();
  }

  const { compliances, diagnostics: cd } = buildCompliances(beams, structure);
  diagnostics.push(...cd);

  const supportKind = getSupportKind(structure);
  const lastBeamIx = beams.length - 1;
  const lastBeamLen = beams[lastBeamIx]!.length_mm;

  // Per-query Directional.max() gives (d*, δ_max) for each query node.
  const nodes: NodeDeflectionResult[] = [];
  compliances.queryNodes.forEach((qn, queryIx) => {
    // For support(both), hide the pinned tip — it has been constrained to
    // (near-)zero in the perpendicular plane, so visualizing it just adds
    // a degenerate surface at the chain's end.
    if (supportKind === 'both' && qn.beamIx === lastBeamIx && qn.offset_mm === lastBeamLen) return;
    const beam = beams[qn.beamIx]!;
    const worldPos: Vec3 = [
      beam.startFrame.origin[0] + beam.startFrame.fwd[0] * qn.offset_mm,
      beam.startFrame.origin[1] + beam.startFrame.fwd[1] * qn.offset_mm,
      beam.startFrame.origin[2] + beam.startFrame.fwd[2] * qn.offset_mm,
    ];
    const dir = directionalFor(compliances, queryIx);
    const { d, value } = dir.max();
    nodes.push({
      queryIx,
      worldPos_undeformed: worldPos,
      d_star: d,
      delta_max_mm: value,
      directional: dir,
    });
  });

  let maxNodeIx = -1;
  let maxVal = -Infinity;
  for (let i = 0; i < nodes.length; i++) {
    if (nodes[i]!.delta_max_mm > maxVal) {
      maxVal = nodes[i]!.delta_max_mm;
      maxNodeIx = i;
    }
  }

  let forces: SimForce[] = [];
  let beamAttribs: SimBeam[] = [];

  const headline = maxNodeIx >= 0 ? nodes[maxNodeIx]! : undefined;
  if (headline && headline.delta_max_mm > 0 && compliances.loadNodes.length > 0) {
    const attr = attribute(compliances, headline.queryIx, headline.d_star);
    forces = attr.perLoad.map((pl: PerLoadAttrib) => {
      const ln = compliances.loadNodes[pl.loadIx]!;
      return {
        loadIx: pl.loadIx,
        beamIx: ln.beamIx,
        offset_mm: ln.offset_mm,
        Fmax_N: compliances.loadFmax_N[pl.loadIx] ?? 0,
        delta_mm: pl.signed_mm,
        fraction: pl.fraction,
      };
    });
    beamAttribs = attr.perBeam.map((pb: PerBeamAttrib) => ({
      beamIx: pb.beamIx,
      total_fraction: pb.total_fraction,
      axial_fraction: pb.axial_fraction,
      bendIx_fraction: pb.bendIx_fraction,
      bendIy_fraction: pb.bendIy_fraction,
      torsion_fraction: pb.torsion_fraction,
    }));
  }

  return {
    nodes,
    maxNodeIx,
    display_scale: 1,
    forces,
    beams: beamAttribs,
    diagnostics,
  };
}

function emptyResult(): SimResult {
  return {
    nodes: [],
    maxNodeIx: -1,
    display_scale: 1,
    forces: [],
    beams: [],
    diagnostics: [],
  };
}
