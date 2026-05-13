import type { Compliances, Mat3, Mode } from './compliance';
import type { Vec3 } from '../walker';

// Per-(load, beam, mode) attribution at a fixed query q and direction d.
//
// For each load p:
//   F_p*  =  F_max_p · (C_tot[q][p]ᵀ · d) / |C_tot[q][p]ᵀ · d|
//
// Per-(load, beam, mode) signed projection on d:
//   contrib(d; q, p, b, m)  =  ⟨ d , C[q][p][b][m] · F_p* ⟩
//
// Telescoping sums (exact, no triangle-inequality slop):
//   Σ_{b,m} contrib(d; q, p, ·, ·)  =  F_max_p · | C_tot[q][p]ᵀ · d |
//   Σ_p     ( ... )                  =  δ_q(d)

export interface PerLoadAttrib {
  loadIx: number;
  signed_mm: number;
  fraction: number;
}

export interface PerBeamAttrib {
  beamIx: number;
  total_fraction: number;
  axial_fraction: number;
  bendIx_fraction: number;
  bendIy_fraction: number;
  torsion_fraction: number;
}

export interface AttributionResult {
  delta_mm: number; // δ_q(d)
  perLoad: PerLoadAttrib[];
  perBeam: PerBeamAttrib[];
}

export function attribute(c: Compliances, queryIx: number, d: Vec3): AttributionResult {
  const dn = normalizeOrZero(d);
  if (dn === null) {
    return { delta_mm: 0, perLoad: [], perBeam: [] };
  }

  // Per-load optimal force F_p* in world coords, plus the per-load magnitude
  // contribution to δ.
  const fStar: (Vec3 | null)[] = new Array(c.loadNodes.length).fill(null);
  const perLoadMag: number[] = new Array(c.loadNodes.length).fill(0);
  for (const t of c.totals) {
    if (t.queryIx !== queryIx) continue;
    const Cqp = t.C;
    const v = matVecT(Cqp, dn); // C^T · d  (load-space direction of contribution)
    const mag = Math.hypot(v[0], v[1], v[2]);
    const Fmax = c.loadFmax_N[t.loadIx] ?? 0;
    perLoadMag[t.loadIx] = Fmax * mag;
    if (mag > 1e-30) {
      fStar[t.loadIx] = [Fmax * v[0] / mag, Fmax * v[1] / mag, Fmax * v[2] / mag];
    }
  }

  const delta = perLoadMag.reduce((s, m) => s + m, 0);
  const dInv = delta > 1e-30 ? 1 / delta : 0;

  const perLoad: PerLoadAttrib[] = perLoadMag.map((signed, loadIx) => ({
    loadIx,
    signed_mm: signed,
    fraction: signed * dInv,
  }));

  // Walk entries, accumulating per-beam, per-mode signed contributions.
  type BeamSums = { total: number; axial: number; bendIx: number; bendIy: number; torsion: number; };
  const perBeamMap = new Map<number, BeamSums>();
  for (const e of c.entries) {
    if (e.queryIx !== queryIx) continue;
    const F = fStar[e.loadIx];
    if (!F) continue;
    const CF = matVec(e.C, F);
    const signed = dn[0] * CF[0] + dn[1] * CF[1] + dn[2] * CF[2];

    let bs = perBeamMap.get(e.beamIx);
    if (!bs) {
      bs = { total: 0, axial: 0, bendIx: 0, bendIy: 0, torsion: 0 };
      perBeamMap.set(e.beamIx, bs);
    }
    bs.total += signed;
    bs[modeKey(e.mode)] += signed;
  }

  const perBeam: PerBeamAttrib[] = Array.from(perBeamMap.entries())
    .sort(([a], [b]) => a - b)
    .map(([beamIx, bs]) => ({
      beamIx,
      total_fraction: bs.total * dInv,
      axial_fraction: bs.axial * dInv,
      bendIx_fraction: bs.bendIx * dInv,
      bendIy_fraction: bs.bendIy * dInv,
      torsion_fraction: bs.torsion * dInv,
    }));

  return { delta_mm: delta, perLoad, perBeam };
}

function modeKey(m: Mode): 'axial' | 'bendIx' | 'bendIy' | 'torsion' {
  return m;
}

function matVec(M: Mat3, v: Vec3): Vec3 {
  return [
    M[0] * v[0] + M[1] * v[1] + M[2] * v[2],
    M[3] * v[0] + M[4] * v[1] + M[5] * v[2],
    M[6] * v[0] + M[7] * v[1] + M[8] * v[2],
  ];
}

function matVecT(M: Mat3, v: Vec3): Vec3 {
  return [
    M[0] * v[0] + M[3] * v[1] + M[6] * v[2],
    M[1] * v[0] + M[4] * v[1] + M[7] * v[2],
    M[2] * v[0] + M[5] * v[1] + M[8] * v[2],
  ];
}

function normalizeOrZero(v: Vec3): Vec3 | null {
  const m = Math.hypot(v[0], v[1], v[2]);
  if (m < 1e-30) return null;
  return [v[0] / m, v[1] / m, v[2] / m];
}
