// sim/'s input contract — a self-contained "deflection problem". Standard
// units: mm, N, MPa. Only depends on the leaf math types.
//
// Preconditions (validated by `simulate`, see simulate.ts):
//   - the chain is serial: beams[i] hangs off beams[i-1].
//   - beams[0].frame.origin_mm is the clamp, at world [0, 0, 0].
//   - beams[i>0].frame.origin_mm lies on beam i-1's axis within [0, length].

import type { Vec3 } from './math';

/**
 * Beam-local frame expressed in world coords. (ex, ey) span the cross-sectional
 * plane; axial is the beam axis. Orthonormal right-handed: ex × ey = axial.
 */
export interface Frame {
  origin_mm: Vec3;
  ex: Vec3;
  ey: Vec3;
  axial: Vec3;
}

/** Resolved cross-section, beam-local. */
export interface Section {
  /** Second moment of area resisting bending in the ey direction. */
  Ix_mm4: number;
  /** Second moment of area resisting bending in the ex direction. */
  Iy_mm4: number;
  /** Torsion constant. */
  J_mm4: number;
}

export interface Material {
  E_MPa: number;
  G_MPa: number;
}

/** Spans `frame.origin_mm` → `frame.origin_mm + length_mm · frame.axial`. */
export interface Beam {
  frame: Frame;
  length_mm: number;
  section: Section;
  material: Material;
}

/**
 * A load fully resolved to a world-force magnitude. Mass·acceleration body
 * loads are pre-expanded into ordinary `Load`s by the caller.
 */
export interface Load {
  beamIx: number;
  offset_mm: number;
  Fmax_N: number;
}

/** A point at which to evaluate the deflection distribution δ(d). */
export interface DeflectionQuery {
  beamIx: number;
  offset_mm: number;
}

export interface Problem {
  beams: Beam[];
  loads: Load[];
  queries: DeflectionQuery[];
  support: 'single' | 'both';
}
