// Helpers for metamorphic property tests on sim/.
//
// Two kinds of helpers live here:
//   1. Problem transformations (rotate, scale loads, scale material, …) that
//      preserve some math invariant of the response. Each is a pure function.
//   2. Envelope comparison by support-function sampling. ConvexEnvelopes are
//      opaque convex sets; the support function is the only side that admits
//      a direct numerical equality check.

import type { Beam, Frame, Material, Problem } from '../../src/sim/problem';
import type { ConvexEnvelope, Mat3, Vec3 } from '../../src/sim/math';

// ---------- Vec3 / Mat3 ----------

export function vAdd(a: Vec3, b: Vec3): Vec3 { return [a[0] + b[0], a[1] + b[1], a[2] + b[2]]; }
export function vScale(a: Vec3, s: number): Vec3 { return [a[0] * s, a[1] * s, a[2] * s]; }
export function vDot(a: Vec3, b: Vec3): number { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
export function vNorm(a: Vec3): number { return Math.hypot(a[0], a[1], a[2]); }
export function vUnit(a: Vec3): Vec3 { const m = vNorm(a); return [a[0] / m, a[1] / m, a[2] / m]; }
export function vCross(a: Vec3, b: Vec3): Vec3 {
  return [a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]];
}

// Row-major Mat3 · Vec3.
export function mApply(R: Mat3, v: Vec3): Vec3 {
  return [
    R[0] * v[0] + R[1] * v[1] + R[2] * v[2],
    R[3] * v[0] + R[4] * v[1] + R[5] * v[2],
    R[6] * v[0] + R[7] * v[1] + R[8] * v[2],
  ];
}

// Axis-angle → row-major rotation matrix (Rodrigues).
export function rotMat(axis: Vec3, angle: number): Mat3 {
  const k = vUnit(axis);
  const c = Math.cos(angle), s = Math.sin(angle), oc = 1 - c;
  const [x, y, z] = k;
  return [
    c + x * x * oc,     x * y * oc - z * s, x * z * oc + y * s,
    y * x * oc + z * s, c + y * y * oc,     y * z * oc - x * s,
    z * x * oc - y * s, z * y * oc + x * s, c + z * z * oc,
  ];
}

// ---------- Problem transformations ----------

function rotateFrame(R: Mat3, f: Frame): Frame {
  return {
    origin_mm: mApply(R, f.origin_mm),
    ex: mApply(R, f.ex),
    ey: mApply(R, f.ey),
    axial: mApply(R, f.axial),
  };
}

/** Rigid rotation of the whole problem about the world origin. */
export function rotateProblem(p: Problem, R: Mat3): Problem {
  return {
    ...p,
    beams: p.beams.map((b) => ({ ...b, frame: rotateFrame(R, b.frame) })),
  };
}

/** Multiply every load's Fmax_N by α. */
export function scaleLoads(p: Problem, alpha: number): Problem {
  return { ...p, loads: p.loads.map((l) => ({ ...l, Fmax_N: l.Fmax_N * alpha })) };
}

/** Multiply every beam's E and G by α. */
export function scaleMaterial(p: Problem, alpha: number): Problem {
  return {
    ...p,
    beams: p.beams.map((b) => ({
      ...b,
      material: { E_MPa: b.material.E_MPa * alpha, G_MPa: b.material.G_MPa * alpha } as Material,
    })),
  };
}

/**
 * Split beam `beamIx` at offset `s` into two collinear sub-beams of length
 * s and L−s, sharing frame/section/material. Offsets of loads/queries on the
 * second half are rebased so their world positions don't move.
 *
 * Restriction: only the LAST beam can be split. The serial-chain rule says
 * beam[i].origin lies on beam[i-1]'s axis — that holds for the first split
 * piece (it inherits the original frame) but the next beam in the chain
 * (originally attached to beam[beamIx]) would have to re-attach to the
 * SECOND piece, and its attachment offset on the original beam may be
 * smaller than s, which puts it "behind" the second piece's origin. We sidestep
 * this entirely by only allowing splits at the tail.
 */
export function splitBeam(p: Problem, beamIx: number, s: number): Problem {
  const old = p.beams[beamIx];
  if (!old) throw new Error('splitBeam: bad beamIx');
  if (beamIx !== p.beams.length - 1) {
    throw new Error('splitBeam: only the last beam can be split (serial-chain constraint)');
  }
  const L = old.length_mm;
  if (s <= 0 || s >= L) throw new Error(`splitBeam: s=${s} out of (0, L=${L})`);

  const first: Beam = { ...old, length_mm: s };
  const second: Beam = {
    ...old,
    length_mm: L - s,
    frame: { ...old.frame, origin_mm: vAdd(old.frame.origin_mm, vScale(old.frame.axial, s)) },
  };

  const beams = [...p.beams.slice(0, beamIx), first, second];

  const remap = (oldBeamIx: number, offset_mm: number): { beamIx: number; offset_mm: number } => {
    if (oldBeamIx < beamIx) return { beamIx: oldBeamIx, offset_mm };
    return offset_mm <= s
      ? { beamIx, offset_mm }
      : { beamIx: beamIx + 1, offset_mm: offset_mm - s };
  };

  return {
    ...p,
    beams,
    loads: p.loads.map((l) => ({ ...remap(l.beamIx, l.offset_mm), Fmax_N: l.Fmax_N })),
    queries: p.queries.map((q) => remap(q.beamIx, q.offset_mm)),
  };
}

/**
 * Append a "force-less appendage" at the END of the chain: a new beam whose
 * origin sits on the last beam's axis (anywhere along it), with arbitrary
 * orientation, and which carries no loads or queries.
 *
 * Restriction: the appendage attaches to the LAST beam. We can't insert a
 * branch mid-chain because the serial-chain rule (beam[i] attaches to
 * beam[i-1]) forbids it; the original child of the parent beam would have
 * to re-attach to the new appendage's axis, and the appendage's axis is
 * generally not the same as the parent's.
 */
export function appendAppendage(
  p: Problem,
  s_on_parent: number,
  length_mm: number,
  rotateAxialAngleRad: number,
  section: { Ix_mm4: number; Iy_mm4: number; J_mm4: number },
  material: Material,
): Problem {
  const parent = p.beams[p.beams.length - 1];
  if (!parent) throw new Error('appendAppendage: empty chain');
  const origin_mm = vAdd(parent.frame.origin_mm, vScale(parent.frame.axial, s_on_parent));

  // New axial: rotate parent.ex about parent.axial by rotateAxialAngleRad.
  // This sweeps the new axial through the cross-section plane (parent.ex,
  // parent.ey), giving a beam that's perpendicular to the parent.
  const R = rotMat(parent.frame.axial, rotateAxialAngleRad);
  const newAxial = mApply(R, parent.frame.ex);
  // Orthonormal (ex', ey', axial'). Pick newEx from the parent.axial direction
  // projected to the plane perpendicular to newAxial.
  const newEx = vUnit(vCross(parent.frame.axial, newAxial));
  const newEy = vCross(newAxial, newEx);

  const newBeam: Beam = {
    frame: { origin_mm, ex: newEx, ey: newEy, axial: newAxial },
    length_mm,
    section,
    material,
  };

  return { ...p, beams: [...p.beams, newBeam] };
}

// ---------- Envelope comparison ----------

/**
 * Sampling directions for support-function equality. ±x/y/z + 8 octants of
 * (±1, ±1, ±1)/√3, plus 8 fixed pseudo-random unit vectors. Any centro-
 * symmetric convex body whose support agrees on enough directions to
 * distinguish principal axes is effectively pinned for our purposes.
 */
const SAMPLE_DIRS: Vec3[] = (() => {
  const ds: Vec3[] = [];
  for (const sx of [-1, 1]) for (const sy of [-1, 1]) for (const sz of [-1, 1]) {
    ds.push([sx, 0, 0], [0, sy, 0], [0, 0, sz]);
    ds.push(vUnit([sx, sy, sz]));
  }
  // Fixed seeded extras (irrational angles) for off-axis coverage.
  const extras: Vec3[] = ([
    [0.27, 0.83, -0.49], [-0.61, 0.12, 0.78], [0.44, -0.71, -0.55],
    [0.95, -0.18, 0.26], [-0.33, -0.77, 0.55], [0.12, 0.99, -0.05],
    [-0.50, 0.50, 0.70], [0.70, 0.20, -0.69],
  ] as Vec3[]).map(vUnit);
  ds.push(...extras);
  // Dedupe identical entries from ±0 + (sx,sy,sz)/√3 cube corners (already
  // distinct, so the set is fine — just don't bother).
  return ds;
})();

export const sampleDirections: readonly Vec3[] = SAMPLE_DIRS;

export interface CompareOpts {
  /** Optional rotation: compare a's frame with b's frame rotated by R. */
  rotate?: Mat3;
  absTol?: number;
  relTol?: number;
  label?: string;
}

/**
 * Returns null on success, else a human-readable failure description.
 *
 * Math: if b = R·a as convex sets, then support_b(d) = support_a(Rᵀ d).
 * Caller passes R (or omits it for plain equality).
 */
export function compareEnvelopes(
  a: ConvexEnvelope,
  b: ConvexEnvelope,
  opts: CompareOpts = {},
): string | null {
  const absTol = opts.absTol ?? 1e-7;
  const relTol = opts.relTol ?? 1e-9;
  const R = opts.rotate;

  for (const d of SAMPLE_DIRS) {
    const dRot: Vec3 = R
      ? [
          R[0] * d[0] + R[3] * d[1] + R[6] * d[2], // Rᵀ d
          R[1] * d[0] + R[4] * d[1] + R[7] * d[2],
          R[2] * d[0] + R[5] * d[1] + R[8] * d[2],
        ]
      : d;
    const va = a.support(dRot);
    const vb = b.support(d);
    const diff = Math.abs(va - vb);
    const scale = Math.max(Math.abs(va), Math.abs(vb), 1);
    if (diff > absTol && diff / scale > relTol) {
      return `${opts.label ?? 'envelope'} mismatch at d=[${d.map((x) => x.toFixed(3)).join(',')}]: a=${va.toExponential(6)} b=${vb.toExponential(6)} Δ=${diff.toExponential(3)}`;
    }
  }
  return null;
}

/** Scale-comparison: support_b(d) = α · support_a(d) for some α. */
export function compareEnvelopesScaled(
  a: ConvexEnvelope,
  b: ConvexEnvelope,
  alpha: number,
  opts: { absTol?: number; relTol?: number; label?: string } = {},
): string | null {
  const absTol = opts.absTol ?? 1e-7;
  const relTol = opts.relTol ?? 1e-9;
  for (const d of SAMPLE_DIRS) {
    const va = a.support(d) * alpha;
    const vb = b.support(d);
    const diff = Math.abs(va - vb);
    const scale = Math.max(Math.abs(va), Math.abs(vb), 1);
    if (diff > absTol && diff / scale > relTol) {
      return `${opts.label ?? 'envelope'} scale mismatch (α=${alpha}) at d=[${d.map((x) => x.toFixed(3)).join(',')}]: α·a=${va.toExponential(6)} b=${vb.toExponential(6)}`;
    }
  }
  return null;
}

export function vecsClose(a: Vec3, b: Vec3, opts: { absTol?: number; relTol?: number } = {}): boolean {
  const absTol = opts.absTol ?? 1e-9;
  const relTol = opts.relTol ?? 1e-10;
  for (let i = 0; i < 3; i++) {
    const diff = Math.abs((a[i] as number) - (b[i] as number));
    const scale = Math.max(Math.abs(a[i] as number), Math.abs(b[i] as number), 1);
    if (diff > absTol && diff / scale > relTol) return false;
  }
  return true;
}
