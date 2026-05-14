import type { Compliances, Mat3 } from './compliance';
import type { Vec3 } from './problem';

// Directional distribution at a query node q:
//
//   δ_q(d)  :=  Σ_p F_max_p · | C_tot[q][p]ᵀ · d |       (units: mm)
//
// d is a world-frame unit vector. δ_q is convex, positively-homogeneous-1 in d,
// and is the support function of the Minkowski sum of ellipsoids
// { F_max_p · C_tot[q][p]ᵀ · u : |u| ≤ 1 }.
//
// `at` evaluates δ(d) at a given d. `max` finds (d*, δ(d*)) by gradient-power
// iteration on the support-function gradient — exact to machine precision in a
// handful of iterations for any reasonable problem. `sample` returns a
// quasi-uniform grid on S² for visualization.

export interface DirectionalSample { d: Vec3; value: number; }

// One {N, F} pair in the sum δ(d) = Σ F·|N·d|. The Minkowski sum interpretation
// from the file header: each term is a load's contribution as an ellipsoid
// support function.
export interface DeltaTerm { N: Mat3; F: number; }

export interface Directional {
  at(d: Vec3): number;
  max(): { d: Vec3; value: number };
  sample(nDirs: number): DirectionalSample[];
  // Raw terms for callers that need the matrices directly (e.g. shader
  // uniforms or sparse spot-check against a GPU-side δ).
  terms(): readonly DeltaTerm[];
}

// Canonical evaluation of δ on a *unit* direction. The lobe vertex shader is
// a line-for-line GLSL twin of this function; the sparse cpuDelta spot-check
// in the lobe geometry halo-paints any per-vertex disagreement, so the two
// implementations must stay in sync.
export function delta(d_unit: Vec3, terms: readonly DeltaTerm[]): number {
  let s = 0;
  for (const { N, F } of terms) {
    const v = matVec(N, d_unit);
    s += F * Math.hypot(v[0], v[1], v[2]);
  }
  return s;
}

// Build a Directional for query q. Internally caches C_tot[q][p]^T per load.
export function directionalFor(c: Compliances, queryIx: number): Directional {
  const ts: DeltaTerm[] = [];
  for (const t of c.totals) {
    if (t.queryIx !== queryIx) continue;
    ts.push({ N: transpose(t.C), F: c.loadFmax_N[t.loadIx] ?? 0 });
  }

  function at(d: Vec3): number {
    const dn = normalizeOrZero(d);
    if (dn === null) return 0;
    return delta(dn, ts);
  }

  function findMax(): { d: Vec3; value: number } {
    if (ts.length === 0) return { d: [1, 0, 0], value: 0 };

    let best: { d: Vec3; value: number } | null = null;
    // Multiple seeds to escape any flat region; convex max means any single
    // seed almost always converges, but seeding with axes is cheap insurance.
    const seeds: Vec3[] = [
      [1, 0, 0], [0, 1, 0], [0, 0, 1],
      normalize([1, 1, 1]),
    ];
    for (const seed of seeds) {
      let d = seed;
      let val = at(d);
      for (let iter = 0; iter < 60; iter++) {
        // Gradient ∇δ(d) = Σ_p F_p · N_p^T · (N_p d) / |N_p d|
        const g: Vec3 = [0, 0, 0];
        for (const { N, F } of ts) {
          const v = matVec(N, d);
          const mag = Math.hypot(v[0], v[1], v[2]);
          if (mag < 1e-30) continue;
          const u: Vec3 = [v[0] / mag, v[1] / mag, v[2] / mag];
          const gradTerm = matVecT(N, u); // N^T · u
          g[0] += F * gradTerm[0];
          g[1] += F * gradTerm[1];
          g[2] += F * gradTerm[2];
        }
        const gn = normalizeOrZero(g);
        if (gn === null) break;
        const newVal = at(gn);
        if (newVal <= val + val * 1e-15 + 1e-30) {
          // Converged (no improvement). Keep the new d (might still have
          // moved infinitesimally) but stop iterating.
          d = gn; val = newVal;
          break;
        }
        d = gn; val = newVal;
      }
      if (!best || val > best.value) best = { d, value: val };
    }
    return best!;
  }

  function sample(nDirs: number): DirectionalSample[] {
    const out: DirectionalSample[] = [];
    const n = Math.max(1, Math.floor(nDirs));
    const golden = Math.PI * (3 - Math.sqrt(5));
    for (let i = 0; i < n; i++) {
      const z = 1 - (2 * (i + 0.5)) / n;
      const r = Math.sqrt(Math.max(0, 1 - z * z));
      const phi = golden * i;
      const d: Vec3 = [r * Math.cos(phi), r * Math.sin(phi), z];
      out.push({ d, value: at(d) });
    }
    return out;
  }

  return { at, max: findMax, sample, terms: () => ts };
}

// ---------- local math helpers ----------

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

function transpose(M: Mat3): Mat3 {
  return [M[0], M[3], M[6], M[1], M[4], M[7], M[2], M[5], M[8]];
}

function normalize(v: Vec3): Vec3 {
  const m = Math.hypot(v[0], v[1], v[2]);
  return [v[0] / m, v[1] / m, v[2] / m];
}

function normalizeOrZero(v: Vec3): Vec3 | null {
  const m = Math.hypot(v[0], v[1], v[2]);
  if (m < 1e-30) return null;
  return [v[0] / m, v[1] / m, v[2] / m];
}
