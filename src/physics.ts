import { BeamState, MATERIALS, KGF_TO_N } from './state';

// Peak deflection in mm. Closed-form Euler-Bernoulli, idealised loads/supports.
//   Cantilever, tip load P at x=L:           delta = P L^3 / (3 E I)
//   Simply-supported, center load at x=L/2:  delta = P L^3 / (48 E I)
// Force is treated as magnitude (always positive); sign convention is captured
// by axis convention in state.ts (force points -Z).
export function computePeakDeflection(s: BeamState): number {
  const E = MATERIALS[s.material].E_MPa;
  const P = Math.abs(s.force_kgf) * KGF_TO_N;
  const L = s.L_mm;
  const I = s.I_mm4;
  if (!(E > 0) || !(I > 0) || !(L > 0)) return 0;
  const denom = s.beamType === 'cantilever' ? 3 : 48;
  return (P * L * L * L) / (denom * E * I);
}

export type DisplayScale = 1 | 10 | 100;

export const SCALES: DisplayScale[] = [1, 10, 100];

// Pick the discrete scale that brings the rendered deflection closest to
// ~10% of L. Snaps to one of {1, 10, 100}.
export function autoDisplayScale(peakMm: number, L_mm: number): DisplayScale {
  if (!(peakMm > 0) || !(L_mm > 0)) return 1;
  const ideal = (0.1 * L_mm) / peakMm;
  const power = Math.round(Math.log10(ideal));
  if (power <= 0) return 1;
  if (power === 1) return 10;
  return 100;
}
