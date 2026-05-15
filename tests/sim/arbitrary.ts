// fast-check arbitraries for Problem.
//
// Goal: generate well-conditioned, legally-connected serial-chain problems
// that exercise the full range of compliance code paths (twist + 4 bending
// sub-modes, fixed-fixed compatibility, mid-attached children, multiple
// beams, multiple loads, multiple queries) without producing degenerate
// floating-point inputs that drown relative-tolerance checks.

import fc from 'fast-check';
import type { Beam, DeflectionQuery, Frame, Load, Material, Problem, Section } from '../../src/sim/problem';
import type { Vec3 } from '../../src/sim/math';
import { vAdd, vCross, vScale, vUnit, rotMat, mApply } from './invariants';

// ---------- Fixed pools ----------

const STEEL: Material = { E_MPa: 200_000, G_MPa: 79_000 };
const ALU: Material = { E_MPa: 70_000, G_MPa: 26_000 };
const MATERIALS: Material[] = [STEEL, ALU];

function rectSection(W: number, H: number): Section {
  const a = Math.max(W, H), b = Math.min(W, H), r = b / a;
  return {
    Ix_mm4: (W * H ** 3) / 12,
    Iy_mm4: (W ** 3 * H) / 12,
    J_mm4: a * b ** 3 * (1 / 3 - 0.21 * r * (1 - r ** 4 / 12)),
  };
}

// Square + a rectangular ⇒ Ix ≠ Iy on at least one section, exercising
// bendIx vs bendIy independently.
const SECTIONS: Section[] = [
  rectSection(10, 10),
  rectSection(20, 20),
  rectSection(8, 16),
];

// ---------- Primitive arbs ----------

const arbAngle = fc.double({ min: -Math.PI, max: Math.PI, noNaN: true });

/** Uniform unit vector on S². */
const arbUnit: fc.Arbitrary<Vec3> = fc.tuple(
  fc.double({ min: -1, max: 1, noNaN: true }),  // cos θ
  fc.double({ min: 0, max: 2 * Math.PI, noNaN: true }), // φ
).map(([cosTheta, phi]) => {
  const sinTheta = Math.sqrt(Math.max(0, 1 - cosTheta * cosTheta));
  return [sinTheta * Math.cos(phi), sinTheta * Math.sin(phi), cosTheta] as Vec3;
});

/** Right-handed orthonormal frame (ex, ey, axial) at a given origin. */
function frameAt(origin_mm: Vec3, axial: Vec3, rollAngle: number): Frame {
  // Pick a tangent perpendicular to axial. Use a stable "smallest |comp|" trick.
  const ax = Math.abs(axial[0]), ay = Math.abs(axial[1]), az = Math.abs(axial[2]);
  const seed: Vec3 = ax <= ay && ax <= az ? [1, 0, 0] : ay <= az ? [0, 1, 0] : [0, 0, 1];
  const t0 = vUnit(vCross(axial, seed));
  // Roll the (ex, ey) pair about axial by rollAngle.
  const R = rotMat(axial, rollAngle);
  const ex = mApply(R, t0);
  const ey = vCross(axial, ex);
  return { origin_mm, ex, ey, axial };
}

const arbMaterial = fc.constantFrom<Material>(...MATERIALS);
const arbSection = fc.constantFrom<Section>(...SECTIONS);
const arbLength = fc.double({ min: 30, max: 250, noNaN: true });

// log-uniform Fmax in [0.1, 1000] N
const arbFmax = fc.double({ min: Math.log(0.1), max: Math.log(1000), noNaN: true }).map(Math.exp);

// ---------- Problem arbitrary ----------

export interface ProblemArbOpts {
  nBeams?: { min: number; max: number };
  /** Whether child beams attach at parent.tip ('tip'), mid-axis ('mid'), or either ('mixed'). */
  attachment?: 'tip' | 'mid' | 'mixed';
  /** Forced support kind; default mixes both. */
  support?: 'single' | 'both' | 'mixed';
  /** Loads: range of count. */
  nLoads?: { min: number; max: number };
  /** Queries: range of count. */
  nQueries?: { min: number; max: number };
}

/**
 * Build a random Problem with valid serial-chain geometry. Frames, sections,
 * materials, and load/query placements are all randomised; structural
 * preconditions (root at origin, beam[i>0] on beam[i-1]'s axis) hold by
 * construction so `simulate` never returns SimError on these inputs.
 */
export function arbProblem(opts: ProblemArbOpts = {}): fc.Arbitrary<Problem> {
  const nBeamsRange = opts.nBeams ?? { min: 1, max: 4 };
  const attachment = opts.attachment ?? 'mixed';
  const support = opts.support ?? 'mixed';
  const nLoadsRange = opts.nLoads ?? { min: 0, max: 3 };
  const nQueriesRange = opts.nQueries ?? { min: 1, max: 3 };

  return fc.integer(nBeamsRange).chain((nBeams) =>
    fc.tuple(
      // Per beam: axial unit vector, roll, length, section, material
      fc.array(
        fc.tuple(arbUnit, arbAngle, arbLength, arbSection, arbMaterial),
        { minLength: nBeams, maxLength: nBeams },
      ),
      // Per (beam i > 0): fraction along parent axis where this beam attaches
      fc.array(
        fc.double({ min: 0, max: 1, noNaN: true }),
        { minLength: Math.max(0, nBeams - 1), maxLength: Math.max(0, nBeams - 1) },
      ),
      support === 'mixed'
        ? fc.constantFrom<'single' | 'both'>('single', 'both')
        : fc.constant(support),
      fc.integer(nLoadsRange),
      fc.integer(nQueriesRange),
      // Per load: (beam-fraction-index, offset-fraction, log-Fmax) → resolved below
      fc.array(fc.tuple(fc.double({ min: 0, max: 1, noNaN: true }), fc.double({ min: 0, max: 1, noNaN: true }), arbFmax), { minLength: nLoadsRange.max, maxLength: nLoadsRange.max }),
      fc.array(fc.tuple(fc.double({ min: 0, max: 1, noNaN: true }), fc.double({ min: 0, max: 1, noNaN: true })), { minLength: nQueriesRange.max, maxLength: nQueriesRange.max }),
    ).map(([beamParams, attachFracs, supportKind, nL, nQ, loadParams, queryParams]) => {
      const beams: Beam[] = [];

      // Build root beam at origin.
      const [axial0, roll0, len0, sec0, mat0] = beamParams[0]!;
      beams.push({ frame: frameAt([0, 0, 0], axial0, roll0), length_mm: len0, section: sec0, material: mat0 });

      for (let i = 1; i < beamParams.length; i++) {
        const parent = beams[i - 1]!;
        const frac = attachment === 'tip' ? 1
                   : attachment === 'mid' ? 0.1 + 0.8 * attachFracs[i - 1]!
                   : attachFracs[i - 1]!;
        const s = frac * parent.length_mm;
        const origin = vAdd(parent.frame.origin_mm, vScale(parent.frame.axial, s));
        const [axial, roll, length, section, material] = beamParams[i]!;
        beams.push({ frame: frameAt(origin, axial, roll), length_mm: length, section, material });
      }

      const pickBeam = (frac: number) => Math.min(beams.length - 1, Math.floor(frac * beams.length));
      const loads: Load[] = [];
      for (let i = 0; i < nL; i++) {
        const [bFrac, oFrac, fmax] = loadParams[i]!;
        const bIx = pickBeam(bFrac);
        loads.push({ beamIx: bIx, offset_mm: oFrac * beams[bIx]!.length_mm, Fmax_N: fmax });
      }
      const queries: DeflectionQuery[] = [];
      for (let i = 0; i < nQ; i++) {
        const [bFrac, oFrac] = queryParams[i]!;
        const bIx = pickBeam(bFrac);
        queries.push({ beamIx: bIx, offset_mm: oFrac * beams[bIx]!.length_mm });
      }

      return { beams, loads, queries, support: supportKind };
    }),
  );
}

// Convenience for P9: an arb that picks a random (parentBeamIx, s_frac, length, roll, section, material).
export const arbAppendageSpec = fc.tuple(
  fc.double({ min: 0.05, max: 0.95, noNaN: true }),  // s_frac on parent
  arbAngle,                                           // rotateAxialAngleRad
  arbLength,                                          // length_mm
  arbSection,
  arbMaterial,
  fc.double({ min: 0, max: 1, noNaN: true }),         // parentBeamIx selector frac
);
