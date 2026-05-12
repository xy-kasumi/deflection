// Axis convention (immutable for the whole app):
//   Beam centerline along world +X; world up is +Z.
//   Cross-section local frame: x_section = world Y, y_section = world Z.
//     Ix (about x_section, = ∫z² dA) resists bending that deflects in z_world.
//     Iy (about y_section, = ∫y² dA) resists bending that deflects in y_world.
//   Force is transverse with magnitude |F|; direction sweeps the YZ plane, so
//   the deflection vector traces an ellipse with semi-axes (k/Iy along Y,
//   k/Ix along Z) where k = |F|·L³/(c·E), c = 3 (cantilever) or 48 (SS).

export type MaterialId = 'PLA' | 'AL' | 'STEEL';
export type BeamType = 'cantilever' | 'simply-supported';

export interface Material {
  label: string;
  E_MPa: number; // Young's modulus  [N/mm^2]
  G_MPa: number; // Shear modulus    [N/mm^2]
}

// Single representative value per material (no specific alloy implied — these
// are back-of-the-envelope numbers, not certified properties).
export const MATERIALS: Record<MaterialId, Material> = {
  PLA:   { label: 'Printed PLA', E_MPa:   3_500, G_MPa:  1_300 },
  AL:    { label: 'Aluminum',    E_MPa:  70_000, G_MPa: 26_000 },
  STEEL: { label: 'Steel',       E_MPa: 200_000, G_MPa: 79_000 },
};

export const KGF_TO_N = 9.80665;

export interface BeamState {
  Ix_mm4: number;
  Iy_mm4: number;
  J_mm4: number;
  L_mm: number;
  material: MaterialId;
  force_kgf: number;
  beamType: BeamType;
}

export function defaultState(): BeamState {
  return {
    Ix_mm4: 50,
    Iy_mm4: 300,
    J_mm4: 150,
    L_mm: 100,
    material: 'PLA',
    force_kgf: 0.5,
    beamType: 'cantilever',
  };
}
