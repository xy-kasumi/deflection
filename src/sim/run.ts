import type { Diagnostic } from '../dsl/diagnostics';
import type { Structure } from '../dsl/parse';
import type { BeamNode, Vec3 } from '../walker';
import { buildCompliances, getSupportKind, getTipLoc } from './compliance';
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
  // Index into `nodes` of the chain tip (see vocab). Its d* drives the
  // attribution breakdown and the headline arrow. `-1` when no chain.
  tipNodeIx: number;
  display_scale: DisplayScale;
  forces: SimForce[];
  beams: SimBeam[];
  diagnostics: Diagnostic[];
}

// Orchestrates compliance build → per-query directional max → attribution at
// the worst-case query node. The root beam's end (clamped under support(both))
// is hidden from `nodes` even though compliance keeps it for the solve.
export function runSim(beams: BeamNode[], structure: Structure): SimResult {
  const diagnostics: Diagnostic[] = [];
  if (beams.length === 0) {
    return emptyResult();
  }

  const { compliances, diagnostics: cd } = buildCompliances(beams, structure);
  diagnostics.push(...cd);

  const supportKind = getSupportKind(structure);
  const rootBeamLen = beams[0]!.length_mm;

  // Per-query Directional.max() gives (d*, δ_max) for each query node.
  const nodes: NodeDeflectionResult[] = [];
  compliances.queryNodes.forEach((qn, queryIx) => {
    // Hide the root beam's end under support(both): perpendicular displacement
    // and rotation are zero by construction at the second clamp point, leaving
    // only axial elongation — not useful as a deflection surface. The root
    // beam's start is already excluded as the chain origin.
    if (supportKind === 'both' && qn.beamIx === 0 && qn.offset_mm === rootBeamLen) return;
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

  const tipLoc = getTipLoc(beams, supportKind);
  let tipNodeIx = -1;
  if (tipLoc) {
    tipNodeIx = nodes.findIndex((n) => {
      const qn = compliances.queryNodes[n.queryIx]!;
      return qn.beamIx === tipLoc.beamIx && qn.offset_mm === tipLoc.offset_mm;
    });
  }

  let forces: SimForce[] = [];
  let beamAttribs: SimBeam[] = [];

  const headline = tipNodeIx >= 0 ? nodes[tipNodeIx]! : undefined;
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
    tipNodeIx,
    display_scale: 1,
    forces,
    beams: beamAttribs,
    diagnostics,
  };
}

function emptyResult(): SimResult {
  return {
    nodes: [],
    tipNodeIx: -1,
    display_scale: 1,
    forces: [],
    beams: [],
    diagnostics: [],
  };
}
