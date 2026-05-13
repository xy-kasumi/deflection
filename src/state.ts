// World is right-handed with +Y up (matches three.js default).
// Beam-local frame: X+ = walker.right, Y+ = walker.up, Z+ = walker.fwd.
// Ix = ∫y² dA resists deflection in walker-up; Iy = ∫x² dA resists deflection
// in walker-right.

export type MaterialId = 'plastic' | 'aluminum' | 'steel';

export interface Material {
  E_MPa: number; // Young's modulus  [N/mm^2]
  G_MPa: number; // Shear modulus    [N/mm^2]
}

// Categorical single-value approximations — these are typical numbers, not
// specific alloys. Suitable for back-of-envelope (±20%) stiffness sizing.
export const MATERIALS: Record<MaterialId, Material> = {
  plastic:  { E_MPa:   3_500, G_MPa:  1_300 },
  aluminum: { E_MPa:  70_000, G_MPa: 26_000 },
  steel:    { E_MPa: 200_000, G_MPa: 79_000 },
};

export const KGF_TO_N = 9.80665;
