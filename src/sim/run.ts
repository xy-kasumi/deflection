import type { Diagnostic } from '../dsl/diagnostics';
import type { Structure } from '../dsl/parse';
import type { BeamNode, Vec3 } from '../walker';
import { buildCompliances } from './compliance';
import { directionalFor } from './directional';
import { attribute, type PerBeamAttrib, type PerLoadAttrib } from './attribute';

// Per-query result. world position is the *undeformed* node location; the
// deformed-chain renderer applies d_star · delta_max_mm · display_scale.
export interface NodeDeflectionResult {
  queryIx: number;
  worldPos_undeformed: Vec3;
  d_star: Vec3;
  delta_max_mm: number;
}

export interface SimForce {
  loadIx: number;
  beamIx: number;
  offset_mm: number;
  Fmax_N: number;
  delta_mm: number; // signed contribution at the tip query along d_star(tip)
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

export type DisplayScale = 1 | 10 | 100 | 1000 | 10000;

export interface SimResult {
  nodes: NodeDeflectionResult[];
  tipQueryIx: number;
  display_scale: DisplayScale;
  forces: SimForce[];
  beams: SimBeam[];
  diagnostics: Diagnostic[];
}

// Orchestrates compliance build → per-query directional max → tip attribution.
// Returns a UI-facing SimResult. If the chain has no beams or no loads, returns
// a degenerate result with all-zero deflections.
export function runSim(beams: BeamNode[], structure: Structure): SimResult {
  const diagnostics: Diagnostic[] = [];
  if (beams.length === 0) {
    return emptyResult();
  }

  const { compliances, diagnostics: cd } = buildCompliances(beams, structure);
  diagnostics.push(...cd);

  // Per-query Directional.max() gives (d*, δ_max) for each query node.
  const nodes: NodeDeflectionResult[] = compliances.queryNodes.map((qn, queryIx) => {
    const beam = beams[qn.beamIx]!;
    const worldPos: Vec3 = [
      beam.startFrame.origin[0] + beam.startFrame.fwd[0] * qn.offset_mm,
      beam.startFrame.origin[1] + beam.startFrame.fwd[1] * qn.offset_mm,
      beam.startFrame.origin[2] + beam.startFrame.fwd[2] * qn.offset_mm,
    ];
    const dir = directionalFor(compliances, queryIx);
    const { d, value } = dir.max();
    return { queryIx, worldPos_undeformed: worldPos, d_star: d, delta_max_mm: value };
  });

  const tipQueryIx = nodes.length - 1;
  const tip = nodes[tipQueryIx];

  let forces: SimForce[] = [];
  let beamAttribs: SimBeam[] = [];

  if (tip && tip.delta_max_mm > 0 && compliances.loadNodes.length > 0) {
    const attr = attribute(compliances, tipQueryIx, tip.d_star);
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
    tipQueryIx,
    display_scale: 1,
    forces,
    beams: beamAttribs,
    diagnostics,
  };
}

function emptyResult(): SimResult {
  return {
    nodes: [],
    tipQueryIx: -1,
    display_scale: 1,
    forces: [],
    beams: [],
    diagnostics: [],
  };
}
