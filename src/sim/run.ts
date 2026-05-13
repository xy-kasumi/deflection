import type { Diagnostic } from '../dsl/diagnostics';
import type { Structure } from '../dsl/parse';
import type { BeamNode, Vec3 } from '../walker';
import { buildCompliances, getSupportKind, getTipLoc } from './compliance';
import { directionalFor, type Directional } from './directional';
import { attribute, type PerBeamAttrib, type PerLoadAttrib } from './attribute';

// Per-query result. Carries both the directional max (d*, δ_max) and the
// per-load / per-beam attribution along that d*, so any node can be selected
// in the UI without re-running the math.
export interface NodeDeflectionResult {
  queryIx: number;
  beamIx: number;
  offset_mm: number;
  worldPos_undeformed: Vec3;
  d_star: Vec3;
  delta_max_mm: number;
  directional: Directional;
  loads: SimLoad[];
  beams: SimBeam[];
}

export interface SimLoad {
  loadIx: number;
  beamIx: number;
  offset_mm: number;
  Fmax_N: number;
  delta_mm: number; // signed contribution at this query along its d*
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
  // Index into `nodes` of the chain tip (see vocab). Default selection target
  // for the UI; the headline arrow anchors here when no other node is picked.
  // `-1` when no chain.
  tipNodeIx: number;
  display_scale: DisplayScale;
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

  // Per-query Directional.max() gives (d*, δ_max) for each query node. We
  // also attribute along d* eagerly so the UI can switch selection without
  // recomputing — the cost is small (a few O(L·B·modes) matvecs per node).
  const nodes: NodeDeflectionResult[] = [];
  compliances.queryNodes.forEach((qn, queryIx) => {
    // Hide the root beam's end under support(both): perpendicular deflection
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

    let loads: SimLoad[] = [];
    let beamAttribs: SimBeam[] = [];
    if (value > 0 && compliances.loadNodes.length > 0) {
      const attr = attribute(compliances, queryIx, d);
      loads = attr.perLoad.map((pl: PerLoadAttrib) => {
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

    nodes.push({
      queryIx,
      beamIx: qn.beamIx,
      offset_mm: qn.offset_mm,
      worldPos_undeformed: worldPos,
      d_star: d,
      delta_max_mm: value,
      directional: dir,
      loads,
      beams: beamAttribs,
    });
  });

  const tipLoc = getTipLoc(beams, supportKind);
  let tipNodeIx = -1;
  if (tipLoc) {
    tipNodeIx = nodes.findIndex(
      (n) => n.beamIx === tipLoc.beamIx && n.offset_mm === tipLoc.offset_mm,
    );
  }

  return {
    nodes,
    tipNodeIx,
    display_scale: 1,
    diagnostics,
  };
}

function emptyResult(): SimResult {
  return {
    nodes: [],
    tipNodeIx: -1,
    display_scale: 1,
    diagnostics: [],
  };
}
