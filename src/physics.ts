import { BeamState, MATERIALS, KGF_TO_N } from './state';

export interface Deflection {
  ax_mm: number;  // ellipse semi-axis along world Y (driven by 1/Iy)
  ay_mm: number;  // ellipse semi-axis along world Z (driven by 1/Ix)
  peak_mm: number;
}

export function computeDeflection(s: BeamState): Deflection {
  const E = MATERIALS[s.material].E_MPa;
  const P = Math.abs(s.force_kgf) * KGF_TO_N;
  const L = s.L_mm;
  if (!(E > 0) || !(L > 0)) return { ax_mm: 0, ay_mm: 0, peak_mm: 0 };
  const denom = s.beamType === 'cantilever' ? 3 : 48;
  const k = (P * L * L * L) / (denom * E);
  const ax = s.Iy_mm4 > 0 ? k / s.Iy_mm4 : 0;
  const ay = s.Ix_mm4 > 0 ? k / s.Ix_mm4 : 0;
  return { ax_mm: ax, ay_mm: ay, peak_mm: Math.max(ax, ay) };
}

export type DisplayScale = 1 | 10 | 100 | 1000;
export const SCALES: DisplayScale[] = [1, 10, 100, 1000];

// Visibility floor: keep the current scale unless the rendered ellipse would be
// too small to read, in which case bump up to the smallest scale that clears
// the floor. One-way ratchet: never downshifts automatically.
const VISIBLE_FRAC_OF_L = 0.05;

export function ensureVisibleScale(
  current: DisplayScale,
  peak_mm: number,
  L_mm: number,
): DisplayScale {
  if (!(peak_mm > 0) || !(L_mm > 0)) return current;
  const floor = VISIBLE_FRAC_OF_L * L_mm;
  if (peak_mm * current >= floor) return current;
  for (const s of SCALES) {
    if (peak_mm * s >= floor) return s;
  }
  return SCALES[SCALES.length - 1] as DisplayScale;
}

// "Fake" cross-section for visual sanity-check on (Ix, Iy, J). Two topologies
// cover open- and closed-section regimes; we pick whichever fits better. The
// shape is a sketch, not a claim: composite or anisotropic real-world parts can
// have (Ix, Iy, J) combinations that no isotropic shape reproduces.
export type CrossSection =
  | { type: 'cruciform'; w_mm: number; h_mm: number; t_mm: number }
  | { type: 'hollowBox'; W_mm: number; H_mm: number; t_mm: number }
  | { type: 'unreachable' };

const CRUCIFORM_T_RATIO_MAX = 0.4;

export function computeCrossSection(Ix: number, Iy: number, J: number): CrossSection {
  if (!(Ix > 0) || !(Iy > 0) || !(J > 0)) return { type: 'unreachable' };

  // Thin-walled cruciform: closed-form inverse, exact for the model.
  //   Ix ≈ t·h³/12,  Iy ≈ t·w³/12,  J ≈ (t³/3)(w+h)
  const sumCbrt = Math.cbrt(Ix) + Math.cbrt(Iy);
  const t_c = Math.pow((3 * J) / (Math.cbrt(12) * sumCbrt), 3 / 8);
  const h_c = Math.cbrt((12 * Ix) / t_c);
  const w_c = Math.cbrt((12 * Iy) / t_c);
  if (t_c < CRUCIFORM_T_RATIO_MAX * Math.min(w_c, h_c)) {
    return { type: 'cruciform', w_mm: w_c, h_mm: h_c, t_mm: t_c };
  }

  // Hollow box: aspect r = H/W from the thin-walled Ix/Iy cubic (closed form),
  // then bisect τ = t/min(W,H) so the exact J/Ix matches. The ratio is
  // monotone-decreasing in τ, so when the target lies outside the box's
  // reachable range bisection clamps at the boundary — "best fit" given that
  // Ix and Iy take priority.
  const r = solveCubicForR(Ix / Iy);
  if (!(r > 0) || !Number.isFinite(r)) return { type: 'unreachable' };
  const minSide = Math.min(1, r);
  const target = J / Ix;
  let lo = 1e-4;
  let hi = 0.4995;
  for (let i = 0; i < 60; i++) {
    const mid = 0.5 * (lo + hi);
    const o = boxOutputs(1, r, mid * minSide);
    if (o.J / o.Ix > target) lo = mid;
    else hi = mid;
    if (hi - lo < 1e-10) break;
  }
  const tau = 0.5 * (lo + hi);
  const tn = tau * minSide;
  const oN = boxOutputs(1, r, tn);
  if (!(oN.Ix > 0)) return { type: 'unreachable' };
  // Degree-4 homogeneity: one length-scale s pins Ix exactly.
  const s = Math.pow(Ix / oN.Ix, 0.25);
  return { type: 'hollowBox', W_mm: s, H_mm: r * s, t_mm: tn * s };
}

// Exact moments of area and Roark torsion constant for a hollow rectangle.
function boxOutputs(W: number, H: number, t: number) {
  const Wi = W - 2 * t;
  const Hi = H - 2 * t;
  const Wm = W - t;
  const Hm = H - t;
  return {
    Ix: (W * H * H * H - Wi * Hi * Hi * Hi) / 12,
    Iy: (W * W * W * H - Wi * Wi * Wi * Hi) / 12,
    J: (2 * t * Wm * Wm * Hm * Hm) / (W + H - 2 * t),
  };
}

// Thin-walled hollow box has Ix/Iy = r²·(3+r)/(3r+1) with r = H/W. The cubic
// r³ + 3r² − 3ρr − ρ = 0 has a unique positive real root for ρ > 0; Cardano's
// trigonometric form gives it in closed form (depress r = y−1).
function solveCubicForR(ratio: number): number {
  const k = 1 + ratio;
  if (!(k > 0)) return 1;
  return 2 * Math.sqrt(k) * Math.cos(Math.acos(-1 / Math.sqrt(k)) / 3) - 1;
}
