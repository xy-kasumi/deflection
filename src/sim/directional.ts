import type { Compliances, Mat3, Mode } from './compliance';
import type { Vec3 } from './problem';
import type { BeamDeflection } from './simulate';

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

export interface DirectionalSample { dir: Vec3; value: number; }

/**
 * One {N, F} pair in the sum δ(d) = Σ F·|N·d|. The Minkowski sum interpretation
 * from the file header: each term is a load's contribution as an ellipsoid
 * support function.
 */
export interface DeltaTerm { N: Mat3; F: number; }

export interface Directional {
  at(dir: Vec3): number;
  max(): { dir: Vec3; value: number };
  sample(nDirs: number): DirectionalSample[];
  /**
   * Terms for GPU evaluation, at most `maxTerms`. When the true term count
   * exceeds the cap the smallest terms are folded into a single residue term
   * — an N=identity term, since |I·d| = 1 — that upper-bounds them. So the
   * GPU sees a slight over-estimate; `at`/`max`/`sample` stay exact.
   */
  termsForGpuCompute(maxTerms: number): readonly DeltaTerm[];
}

/**
 * Canonical evaluation of δ on a *unit* direction. The lobe vertex shader is
 * a line-for-line GLSL twin of this function; the sparse cpuDelta spot-check
 * in the lobe geometry halo-paints any per-vertex disagreement, so the two
 * implementations must stay in sync.
 */
export function delta(d_unit: Vec3, terms: readonly DeltaTerm[]): number {
  let s = 0;
  for (const { N, F } of terms) {
    const v = matVec(N, d_unit);
    s += F * Math.hypot(v[0], v[1], v[2]);
  }
  return s;
}

/** Build a Directional for query q. Internally caches C_tot[q][p]^T per load. */
export function directionalFor(c: Compliances, queryIx: number): Directional {
  const ts: DeltaTerm[] = [];
  for (const t of c.totals) {
    if (t.queryIx !== queryIx) continue;
    ts.push({ N: transpose(t.C), F: c.loadFmax_N[t.loadIx] ?? 0 });
  }
  return makeDirectional(ts);
}

/**
 * Rotation Directional at query q. δθ_q(d) = Σ_p F_max_p · |C_rot[q][p]ᵀ · d|
 * — same support-function machinery as `directionalFor`, just over rotation
 * compliance instead of translation. d is a unit *rotation axis*; the returned
 * value is the linearized worst-case rotation magnitude about that axis (rad).
 */
export function rotationFor(c: Compliances, queryIx: number): Directional {
  const ts: DeltaTerm[] = [];
  for (const t of c.rotationTotals) {
    if (t.queryIx !== queryIx) continue;
    ts.push({ N: transpose(t.C), F: c.loadFmax_N[t.loadIx] ?? 0 });
  }
  return makeDirectional(ts);
}

/**
 * δ restricted to each beam's compliance in isolation — each beam worst-cased
 * independently. These do NOT sum to the whole-structure Directional
 * (Σ ≥ δ, triangle inequality); each is honest only on its own.
 */
export function beamDirectionals(c: Compliances, queryIx: number): BeamDeflection[] {
  const byBeam = new Map<number, {
    total: Map<number, Mat3>;
    perMode: Map<Mode, Map<number, Mat3>>;
  }>();
  for (const e of c.entries) {
    if (e.queryIx !== queryIx) continue;
    let b = byBeam.get(e.beamIx);
    if (!b) { b = { total: new Map(), perMode: new Map() }; byBeam.set(e.beamIx, b); }
    accumMat(b.total, e.loadIx, e.C);
    let pm = b.perMode.get(e.mode);
    if (!pm) { pm = new Map(); b.perMode.set(e.mode, pm); }
    accumMat(pm, e.loadIx, e.C);
  }

  return [...byBeam.entries()]
    .sort(([a], [z]) => a - z)
    .map(([beamIx, b]) => ({
      beamIx,
      deflection: makeDirectional(termsOf(b.total, c)),
      byMode: [...b.perMode.entries()].map(([mode, m]) => ({
        mode,
        deflection: makeDirectional(termsOf(m, c)),
        perLoad: [...m.entries()]
          .sort(([a], [z]) => a - z)
          .map(([loadIx, C]) => ({
            loadIx,
            deflection: makeDirectional([{ N: transpose(C), F: c.loadFmax_N[loadIx] ?? 0 }]),
          })),
      })),
    }));
}

// Build the δ(d) = Σ F·|N·d| evaluator over a fixed term list.
function makeDirectional(ts: DeltaTerm[]): Directional {
  function at(dir: Vec3): number {
    const dn = normalizeOrZero(dir);
    if (dn === null) return 0;
    return delta(dn, ts);
  }

  function findMax(): { dir: Vec3; value: number } {
    if (ts.length === 0) return { dir: [1, 0, 0], value: 0 };

    let best: { dir: Vec3; value: number } | null = null;
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
      if (!best || val > best.value) best = { dir: d, value: val };
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
      out.push({ dir: d, value: at(d) });
    }
    return out;
  }

  const dir: Directional = {
    at,
    max: findMax,
    sample,
    termsForGpuCompute: (maxTerms) => capTerms(ts, maxTerms),
  };
  return dir;
}

// At most `maxTerms` terms for a fixed-size GPU uniform array. Keep the largest
// by F·‖N‖_F; fold the rest into one residue term. Since |N·d| ≤ ‖N‖_F for unit
// d, c = Σ_residue F·‖N‖_F upper-bounds the dropped terms, and an N=identity
// term reproduces that constant (|I·d| = 1).
function capTerms(ts: readonly DeltaTerm[], maxTerms: number): readonly DeltaTerm[] {
  if (ts.length <= maxTerms) return ts;
  const ranked = ts
    .map((t) => ({ t, w: t.F * frobenius(t.N) }))
    .sort((a, b) => b.w - a.w);
  const kept: DeltaTerm[] = ranked.slice(0, maxTerms - 1).map((r) => r.t);
  let residue = 0;
  for (let i = maxTerms - 1; i < ranked.length; i++) residue += ranked[i]!.w;
  kept.push({ N: [1, 0, 0, 0, 1, 0, 0, 0, 1], F: residue });
  return kept;
}

function frobenius(M: Mat3): number {
  let s = 0;
  for (let i = 0; i < 9; i++) s += (M[i] as number) * (M[i] as number);
  return Math.sqrt(s);
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

// Accumulate C into the per-load matrix bucket (creating a zero one if absent).
function accumMat(m: Map<number, Mat3>, loadIx: number, C: Mat3): void {
  let cur = m.get(loadIx);
  if (!cur) { cur = [0, 0, 0, 0, 0, 0, 0, 0, 0]; m.set(loadIx, cur); }
  for (let i = 0; i < 9; i++) (cur[i] as number) += C[i] as number;
}

function termsOf(byLoad: Map<number, Mat3>, c: Compliances): DeltaTerm[] {
  const ts: DeltaTerm[] = [];
  for (const [loadIx, C] of byLoad) {
    ts.push({ N: transpose(C), F: c.loadFmax_N[loadIx] ?? 0 });
  }
  return ts;
}
