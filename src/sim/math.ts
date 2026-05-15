// Leaf math module: Vec3, Mat3, Directional.
//
// Mat3 is exported only because Directional.terms exposes it; this file
// intentionally does not provide a Mat3 utility surface. Callers that need
// to manipulate Mat3s (transpose, scale, etc.) own those helpers locally.

/** 3-component column vector, row-major. */
export type Vec3 = [number, number, number];

/** 3×3 row-major matrix: [m00, m01, m02, m10, m11, m12, m20, m21, m22]. */
export type Mat3 = [
  number, number, number,
  number, number, number,
  number, number, number,
];

// Support function of a Minkowski sum of (possibly-degenerate) ellipsoids.
//
// For matrices {M_i}, evaluates
//
//   h(d)  :=  Σ_i | M_i · d |  =  sup_{x ∈ K} ⟨x, d⟩
//
// at unit d, where K = ⊕_i { M_i u : |u| ≤ 1 } is the Minkowski sum of the
// ellipsoids each M_i parameterizes. h is convex, positively-homogeneous-1.
//
// `at` evaluates h(d). `max` finds (d*, h(d*)) by gradient-power iteration
// on ∇h — exact to machine precision in a handful of iterations for any
// reasonable problem. `sample` returns a quasi-uniform grid on S².

export interface DirectionalSample {
  dir_unit: Vec3;
  value: number;
}

export interface Directional {
  /** The matrices {M_i} whose Σ |M_i · d| this Directional evaluates. */
  readonly terms: readonly Mat3[];
  /** h at `dir`; auto-normalizes. */
  at(dir: Vec3): number;
  /** Argmax direction and h(d*). */
  max(): { dir_unit: Vec3; value: number };
  /** Quasi-uniform sphere sample of h. */
  sample(nDirs: number): DirectionalSample[];
}

/**
 * Canonical evaluation of h on a *unit* direction. The lobe vertex shader is
 * a line-for-line GLSL twin of this function; the sparse cpuDelta spot-check
 * in the lobe geometry halo-paints any per-vertex disagreement, so the two
 * implementations must stay in sync.
 */
export function evaluate(d_unit: Vec3, terms: readonly Mat3[]): number {
  let s = 0;
  for (const M of terms) {
    const v = matVec(M, d_unit);
    s += Math.hypot(v[0], v[1], v[2]);
  }
  return s;
}

/** Build the h(d) = Σ |M·d| evaluator over a fixed matrix list. */
export function makeDirectional(ts: Mat3[]): Directional {
  function at(dir: Vec3): number {
    const dn = normalizeOrZero(dir);
    if (dn === null) return 0;
    return evaluate(dn, ts);
  }

  function findMax(): { dir_unit: Vec3; value: number } {
    if (ts.length === 0) return { dir_unit: [1, 0, 0], value: 0 };

    let best: { dir_unit: Vec3; value: number } | null = null;
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
        // ∇h(d) = Σ_i M_i^T · (M_i d) / |M_i d|
        const g: Vec3 = [0, 0, 0];
        for (const M of ts) {
          const v = matVec(M, d);
          const mag = Math.hypot(v[0], v[1], v[2]);
          if (mag < 1e-30) continue;
          const u: Vec3 = [v[0] / mag, v[1] / mag, v[2] / mag];
          const gradTerm = matVecT(M, u);
          g[0] += gradTerm[0];
          g[1] += gradTerm[1];
          g[2] += gradTerm[2];
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
      if (!best || val > best.value) best = { dir_unit: d, value: val };
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
      out.push({ dir_unit: d, value: at(d) });
    }
    return out;
  }

  return { terms: ts, at, max: findMax, sample };
}

/**
 * Truncate to at most `maxTerms` matrices for a fixed-size uniform array
 * (e.g. a GPU shader). Keep the largest by ‖M‖_F; fold the rest into one
 * residue term c·I with c = Σ_residue ‖M‖_F. Since |M·d| ≤ ‖M‖_F for unit
 * d, c upper-bounds the dropped contribution, and |c·I · d| = c reproduces
 * it. Net effect: the truncated h is a pointwise upper bound on the exact one.
 */
export function capTermsForUniform(ts: readonly Mat3[], maxTerms: number): readonly Mat3[] {
  if (ts.length <= maxTerms) return ts;
  const ranked = ts
    .map((M) => ({ M, w: frobenius(M) }))
    .sort((a, b) => b.w - a.w);
  const kept: Mat3[] = ranked.slice(0, maxTerms - 1).map((r) => r.M);
  let residue = 0;
  for (let i = maxTerms - 1; i < ranked.length; i++) residue += ranked[i]!.w;
  kept.push([residue, 0, 0, 0, residue, 0, 0, 0, residue]);
  return kept;
}

// ---------- private helpers ----------

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

function frobenius(M: Mat3): number {
  let s = 0;
  for (let i = 0; i < 9; i++) s += (M[i] as number) * (M[i] as number);
  return Math.sqrt(s);
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
