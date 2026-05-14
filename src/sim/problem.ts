// sim/'s input contract — a self-contained "deflection problem". Pure data,
// zero imports. Standard units: mm, N, MPa.
//
// Preconditions (validated by `simulate`, see simulate.ts):
//   - the chain is serial: beams[i] hangs off beams[i-1].
//   - beams[0].frame.origin_mm is the clamp, at world [0, 0, 0].
//   - beams[i>0].frame.origin_mm lies on beam i-1's axis within [0, length].

export type Vec3 = [number, number, number];

// Beam-local frame expressed in world coords. (ex, ey) span the cross-sectional
// plane; axial is the beam axis. Orthonormal right-handed: ex × ey = axial.
export interface Frame {
  origin_mm: Vec3;
  ex: Vec3;
  ey: Vec3;
  axial: Vec3;
}

// Resolved cross-section, beam-local. Ix = ∫ey² dA resists bending in ey;
// Iy = ∫ex² dA resists bending in ex.
export interface Section {
  Ix_mm4: number;
  Iy_mm4: number;
  J_mm4: number;
}

export interface Material {
  E_MPa: number;
  G_MPa: number;
}

export interface Beam {
  frame: Frame;
  length_mm: number;
  section: Section;
  material: Material;
}

// A load fully resolved to a world-force magnitude. Mass·acceleration body
// loads are pre-expanded into ordinary `Load`s by the caller.
export interface Load {
  beamIx: number;
  offset_mm: number;
  Fmax_N: number;
}

// A point at which to evaluate the deflection distribution δ(d).
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
