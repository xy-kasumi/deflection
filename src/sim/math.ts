// Leaf math module: Vec3, Mat3, ConvexEnvelope.
//
// ConvexEnvelope is a closed convex region K ⊂ ℝ³ containing the origin,
// stored as the Minkowski sum K = ⊕ᵢ { Mᵢ u : |u| ≤ 1 } of (possibly
// degenerate) ellipsoids. Each ellipsoid is centrally symmetric, so K is too.
// Consumers see the envelope, not the storage.
//
// The support function h_K(d) = sup_{p ∈ K} ⟨p, d⟩ has the closed form
// Σ |Mᵢᵀd| thanks to ⊕-additivity of h and h_{MB}(d) = |Mᵀd|. `support`,
// `furthest`, and (later) `boundary` all build on that one formula;
// `supportFromTerms` is the bare evaluator exposed to the lobe vertex shader.
//
// Mat3 is exported only because ConvexEnvelope.ellipsoidMats exposes it; this
// file intentionally does not provide a Mat3 utility surface. Callers that
// need to manipulate Mat3s (transpose, scale, etc.) own those helpers locally.

/** 3-component column vector, row-major. */
export type Vec3 = [number, number, number];

/** 3×3 row-major matrix: [m00, m01, m02, m10, m11, m12, m20, m21, m22]. */
export type Mat3 = [
  number, number, number,
  number, number, number,
  number, number, number,
];

/**
 * Convex region K ⊂ ℝ³, containing the origin.
 *
 * ConvexEnvelope is a Minkowski sum of (possibly degenerate) ellipsoids.
 * Thus it is symmetric about the origin.
 *
 * isInside(p) is expensive and thus not provided.
 */
export interface ConvexEnvelope {
  /**
   * Support function: max{p ∈ K} dot(p, x).
   * `x` must be a unit vector.
   *
   * Note: support(x) is NOT a radial distance from the origin along x.
   */
  support(x: Vec3): number;

  /**
   * Boundary point: returns the point on the surface of K whose outward
   * normal is `normal`. `normal` is auto-normalized.
   *
   * Note: returned point p is NOT parallel to `normal` (generally).
   * Note: at flat faces or corners of the surface (caused by degenerate
   *       ellipsoid components), an arbitrary extreme is returned.
   */
  boundary(normal: Vec3): Vec3;

  /** One of the points in K, furthest from the origin. */
  furthest(): { point: Vec3; distance: number };

  /** Underlying ellipsoids. K = ⊕ᵢ { Mᵢ u : |u| ≤ 1 }. */
  readonly ellipsoidMats: readonly Mat3[];
}

/**
 * Support function evaluator on raw ellipsoid matrices.
 * `d_unit` must already be a unit vector — no normalization here.
 */
export function supportFromTerms(d_unit: Vec3, ellipsoidMats: readonly Mat3[]): number {
  let s = 0;
  for (const M of ellipsoidMats) {
    const v = matVecT(M, d_unit);
    s += Math.hypot(v[0], v[1], v[2]);
  }
  return s;
}

/**
 * Boundary point evaluator on raw ellipsoid matrices.
 * `d_unit` should be a unit vector; the formula is scale-invariant in d so
 * non-unit input gives the same result, but no auto-normalization is
 * performed.
 *
 * The lobe vertex shader is a GLSL twin of this function; the two must stay
 * in sync. The sparse cpuBoundary spot-check halo-paints any per-vertex
 * disagreement (magenta) against the rendered surface position.
 */
export function boundaryFromTerms(d_unit: Vec3, ellipsoidMats: readonly Mat3[]): Vec3 {
  const out: Vec3 = [0, 0, 0];
  for (const M of ellipsoidMats) {
    const v = matVecT(M, d_unit);
    const mag = Math.hypot(v[0], v[1], v[2]);
    if (mag < 1e-30) continue;
    const inv = 1 / mag;
    const u: Vec3 = [v[0] * inv, v[1] * inv, v[2] * inv];
    const w = matVec(M, u);
    out[0] += w[0];
    out[1] += w[1];
    out[2] += w[2];
  }
  return out;
}

/** Build a ConvexEnvelope from a list of ellipsoid shape matrices. */
export function makeConvexEnvelope(ellipsoidMats: Mat3[]): ConvexEnvelope {
  function support(x: Vec3): number {
    return supportFromTerms(x, ellipsoidMats);
  }

  // Auto-scale-invariant in `normal` by construction: numerator and
  // denominator of (Mᵢᵀd)/|Mᵢᵀd| scale together. Degenerate term
  // (|Mᵢᵀd| ≈ 0) means Eᵢ's supporting hyperplane in direction d contains
  // the entire Eᵢ; contributing 0 picks the center, which is the natural
  // representative of the multi-valued argmax set.
  function boundary(normal: Vec3): Vec3 {
    return boundaryFromTerms(normal, ellipsoidMats);
  }

  function furthest(): { point: Vec3; distance: number } {
    if (ellipsoidMats.length === 0) return { point: [0, 0, 0], distance: 0 };

    // Power-iteration on ∇h to find the d* that maximizes h(d). Multiple
    // seeds to escape any flat region; convex max means any single seed
    // almost always converges, but seeding with axes is cheap insurance.
    let bestDir: Vec3 = [1, 0, 0];
    let bestVal = 0;
    const seeds: Vec3[] = [
      [1, 0, 0], [0, 1, 0], [0, 0, 1],
      normalize([1, 1, 1]),
    ];
    for (const seed of seeds) {
      let d = seed;
      let val = support(d);
      for (let iter = 0; iter < 60; iter++) {
        // ∇h(d) = Σ_i M_i · (M_iᵀ d) / |M_iᵀ d|
        const g: Vec3 = [0, 0, 0];
        for (const M of ellipsoidMats) {
          const v = matVecT(M, d);
          const mag = Math.hypot(v[0], v[1], v[2]);
          if (mag < 1e-30) continue;
          const u: Vec3 = [v[0] / mag, v[1] / mag, v[2] / mag];
          const gradTerm = matVec(M, u);
          g[0] += gradTerm[0];
          g[1] += gradTerm[1];
          g[2] += gradTerm[2];
        }
        const gn = normalizeOrZero(g);
        if (gn === null) break;
        const newVal = support(gn);
        if (newVal <= val + val * 1e-15 + 1e-30) {
          d = gn; val = newVal;
          break;
        }
        d = gn; val = newVal;
      }
      if (val > bestVal) { bestVal = val; bestDir = d; }
    }
    return {
      point: [bestDir[0] * bestVal, bestDir[1] * bestVal, bestDir[2] * bestVal],
      distance: bestVal,
    };
  }

  return { ellipsoidMats, support, boundary, furthest };
}

/**
 * Truncate to at most `maxTerms` matrices for a fixed-size uniform array
 * (e.g. a GPU shader). Keep the largest by ‖M‖_F; fold the rest into one
 * residue term c·I with c = Σ_residue ‖M‖_F. Since |Mᵀd| ≤ ‖M‖_F for unit
 * d, c upper-bounds the dropped contribution, and |c·I · d| = c reproduces
 * it. Net effect: the truncated support is a pointwise upper bound on the
 * exact one.
 */
export function capEllipsoidsForUniform(ellipsoidMats: readonly Mat3[], maxTerms: number): readonly Mat3[] {
  if (ellipsoidMats.length <= maxTerms) return ellipsoidMats;
  const ranked = ellipsoidMats
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
