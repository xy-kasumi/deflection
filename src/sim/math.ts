// Leaf math module: Vec3, Mat3, ConvexEnvelope.
//
// ConvexEnvelope is a closed convex region K ⊂ ℝ³ containing the origin,
// stored as the Minkowski sum K = ⊕ᵢ { Mᵢ u : |u| ≤ 1 } of (possibly
// degenerate) ellipsoids. Each ellipsoid is centrally symmetric, so K is too.
// Consumers see the envelope, not the storage.
//
// The support function h_K(d) = sup_{p ∈ K} ⟨p, d⟩ has the closed form
// Σ |Mᵢᵀd| thanks to ⊕-additivity of h and h_{MB}(d) = |Mᵀd|. `support`,
// `boundary`, and `furthest` all build on that one formula.
//
// This file intentionally does not provide a Mat3 utility surface; callers
// that need transpose/scale/etc. own those helpers locally.

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
}

/** Build a ConvexEnvelope from a list of ellipsoid shape matrices. */
export function makeConvexEnvelope(ellipsoidMats: Mat3[]): ConvexEnvelope {
  const n = ellipsoidMats.length;
  // Closure-local scratch: support and boundary never call each other within
  // a single envelope, so reusing these buffers across calls is safe.
  const _v: Vec3 = [0, 0, 0];
  const _u: Vec3 = [0, 0, 0];
  const _w: Vec3 = [0, 0, 0];

  function support(x: Vec3): number {
    let s = 0;
    for (let i = 0; i < n; i++) {
      const M = ellipsoidMats[i]!;
      matVecTInto(M, x, _v);
      s += Math.hypot(_v[0], _v[1], _v[2]);
    }
    return s;
  }

  // Auto-scale-invariant in `normal` by construction: numerator and
  // denominator of (Mᵢᵀd)/|Mᵢᵀd| scale together. Degenerate term
  // (|Mᵢᵀd| ≈ 0) means Eᵢ's supporting hyperplane in direction d contains
  // the entire Eᵢ; contributing 0 picks the center, which is the natural
  // representative of the multi-valued argmax set.
  function boundary(normal: Vec3): Vec3 {
    let ox = 0, oy = 0, oz = 0;
    for (let i = 0; i < n; i++) {
      const M = ellipsoidMats[i]!;
      matVecTInto(M, normal, _v);
      const mag = Math.hypot(_v[0], _v[1], _v[2]);
      if (mag < 1e-30) continue;
      const inv = 1 / mag;
      _u[0] = _v[0] * inv; _u[1] = _v[1] * inv; _u[2] = _v[2] * inv;
      matVecInto(M, _u, _w);
      ox += _w[0]; oy += _w[1]; oz += _w[2];
    }
    return [ox, oy, oz];
  }

  function furthest(): { point: Vec3; distance: number } {
    if (ellipsoidMats.length === 0) return { point: [0, 0, 0], distance: 0 };

    // Power-iteration on ∇h to find the d* that maximizes h(d). ∇h(d) equals
    // boundary(d) (differentiate Σ|Mᵢᵀd| termwise: ∇|Mᵢᵀd| = Mᵢ·(Mᵢᵀd)/|Mᵢᵀd|).
    // Multiple seeds escape any flat region; convex max means any single seed
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
        const gn = normalizeOrZero(boundary(d));
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

  return { support, boundary, furthest };
}

/**
 * Compute M · v into `out`. Returns `out` for chaining.
 * `out === v` is safe; v's components are captured before write.
 */
export function matVecInto(M: Mat3, v: Vec3, out: Vec3): Vec3 {
  const v0 = v[0], v1 = v[1], v2 = v[2];
  out[0] = M[0] * v0 + M[1] * v1 + M[2] * v2;
  out[1] = M[3] * v0 + M[4] * v1 + M[5] * v2;
  out[2] = M[6] * v0 + M[7] * v1 + M[8] * v2;
  return out;
}

/** Compute Mᵀ · v into `out`. `out === v` is safe. */
export function matVecTInto(M: Mat3, v: Vec3, out: Vec3): Vec3 {
  const v0 = v[0], v1 = v[1], v2 = v[2];
  out[0] = M[0] * v0 + M[3] * v1 + M[6] * v2;
  out[1] = M[1] * v0 + M[4] * v1 + M[7] * v2;
  out[2] = M[2] * v0 + M[5] * v1 + M[8] * v2;
  return out;
}

// ---------- private helpers ----------

function normalize(v: Vec3): Vec3 {
  const m = Math.hypot(v[0], v[1], v[2]);
  return [v[0] / m, v[1] / m, v[2] / m];
}

function normalizeOrZero(v: Vec3): Vec3 | null {
  const m = Math.hypot(v[0], v[1], v[2]);
  if (m < 1e-30) return null;
  return [v[0] / m, v[1] / m, v[2] / m];
}
