import type { Compliances, Mat3, Mode } from './compliance';
import type { Vec3 } from './problem';

// Decomposes δ_q(d) into per-(load, beam, mode) contributions at a fixed
// query q and direction d.
//
// For each load p:
//   F_p*  =  F_max_p · (C_tot[q][p]ᵀ · d) / |C_tot[q][p]ᵀ · d|
//
// Per-(load, beam, mode) signed contribution (projection on d):
//   contrib(d; q, p, b, m)  =  ⟨ d , C[q][p][b][m] · F_p* ⟩
//
// Telescoping sums (exact, no triangle-inequality slop):
//   Σ_{b,m} contrib(d; q, p, ·, ·)  =  F_max_p · | C_tot[q][p]ᵀ · d |
//   Σ_p     ( ... )                  =  δ_q(d)

export interface PerLoadContribution {
  loadIx: number;
  delta_mm: number; // signed contribution to δ along d
}

export interface PerBeamContribution {
  beamIx: number;
  total_mm: number; // = bendIx_mm + bendIy_mm + torsionJ_mm
  bendIx_mm: number;
  bendIy_mm: number;
  torsionJ_mm: number;
}

export interface Decomposition {
  perLoad: PerLoadContribution[];
  perBeam: PerBeamContribution[];
}

export function decompose(c: Compliances, queryIx: number, dir: Vec3): Decomposition {
  const dn = normalizeOrZero(dir);
  if (dn === null) {
    return { perLoad: [], perBeam: [] };
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

  const perLoad: PerLoadContribution[] = perLoadMag.map((signed, loadIx) => ({
    loadIx,
    delta_mm: signed,
  }));

  // Walk entries, accumulating per-beam, per-mode signed contributions.
  type BeamSums = { total: number; bendIx: number; bendIy: number; torsionJ: number; };
  const perBeamMap = new Map<number, BeamSums>();
  for (const e of c.entries) {
    if (e.queryIx !== queryIx) continue;
    const F = fStar[e.loadIx];
    if (!F) continue;
    const CF = matVec(e.C, F);
    const signed = dn[0] * CF[0] + dn[1] * CF[1] + dn[2] * CF[2];

    let bs = perBeamMap.get(e.beamIx);
    if (!bs) {
      bs = { total: 0, bendIx: 0, bendIy: 0, torsionJ: 0 };
      perBeamMap.set(e.beamIx, bs);
    }
    bs.total += signed;
    bs[modeKey(e.mode)] += signed;
  }

  const perBeam: PerBeamContribution[] = Array.from(perBeamMap.entries())
    .sort(([a], [b]) => a - b)
    .map(([beamIx, bs]) => ({
      beamIx,
      total_mm: bs.total,
      bendIx_mm: bs.bendIx,
      bendIy_mm: bs.bendIy,
      torsionJ_mm: bs.torsionJ,
    }));

  return { perLoad, perBeam };
}

function modeKey(m: Mode): 'bendIx' | 'bendIy' | 'torsionJ' {
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
