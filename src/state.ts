// Axis convention (immutable for the whole app):
//   beam centerline along +X, world up is +Z, force / deflection along -Z.

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
  I_mm4: number;
  J_mm4: number;
  L_mm: number;
  material: MaterialId;
  force_kgf: number;
  beamType: BeamType;
}

export function defaultState(): BeamState {
  return {
    I_mm4: 1000,
    J_mm4: 2000,
    L_mm: 300,
    material: 'STEEL',
    force_kgf: 1,
    beamType: 'cantilever',
  };
}
