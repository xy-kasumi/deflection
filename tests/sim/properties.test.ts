// Metamorphic property tests for `simulate: Problem → SimOutcome`.
//
// Each P# is a single mathematical invariant. fast-check generates random
// well-conditioned Problems (see arbitrary.ts), applies a transformation,
// re-simulates, and compares the result envelopes via support-function
// samples (see invariants.ts).
//
// Run:  npx tsx tests/sim/properties.test.ts
// Failure mode: fast-check prints the minimal shrunk counterexample and the
// runner exits with code 1.

import fc from 'fast-check';
import { simulate, type SimOutcome, type SimResult, type DeflectionQueryResult } from '../../src/sim/simulate';
import type { Problem } from '../../src/sim/problem';
import { arbProblem, arbAppendageSpec } from './arbitrary';
import {
  compareEnvelopes, compareEnvelopesScaled,
  rotateProblem, scaleLoads, scaleMaterial, splitBeam, appendAppendage,
  rotMat, mApply, vecsClose,
  type CompareOpts,
} from './invariants';
import type { Mat3, Vec3 } from '../../src/sim/math';

// ---------- Runner plumbing ----------

let testsRun = 0;
let testsFailed = 0;

function prop(name: string, runs: number, body: () => fc.IPropertyWithHooks<unknown>): void {
  const start = performance.now();
  testsRun++;
  try {
    fc.assert(body(), { numRuns: runs, verbose: false });
    const ms = (performance.now() - start).toFixed(0);
    console.log(`  ok  ${name}  (${runs} runs, ${ms}ms)`);
  } catch (e) {
    testsFailed++;
    console.log(`  FAIL ${name}`);
    console.log(String(e).split('\n').slice(0, 20).map((l) => `       ${l}`).join('\n'));
  }
}

function mustOk(out: SimOutcome, label: string): SimResult | null {
  if (out.kind === 'error') {
    // Some inputs (e.g. 'both' support on degenerate geometry) trip support-
    // singular. That's a legitimate sim error, not a property violation — skip.
    if (out.code === 'support-singular') return null;
    throw new Error(`${label}: unexpected SimError ${out.code}: ${out.message}`);
  }
  return out;
}

function compareQueries(
  a: SimResult, b: SimResult,
  opts: CompareOpts & { posRotate?: Mat3 } = {},
): string | null {
  if (a.queryResults.length !== b.queryResults.length) {
    return `queryResults.length: a=${a.queryResults.length} b=${b.queryResults.length}`;
  }
  for (let i = 0; i < a.queryResults.length; i++) {
    const qa = a.queryResults[i]!;
    const qb = b.queryResults[i]!;
    const expectedPosB = opts.posRotate ? mApply(opts.posRotate, qa.pos_mm) : qa.pos_mm;
    if (!vecsClose(expectedPosB, qb.pos_mm, { absTol: 1e-7, relTol: 1e-9 })) {
      return `q${i} pos: expected=[${expectedPosB.join(',')}] got=[${qb.pos_mm.join(',')}]`;
    }
    const ed = compareEnvelopes(qa.deflection_mm, qb.deflection_mm, { ...opts, label: `q${i}.deflection` });
    if (ed) return ed;
    const er = compareEnvelopes(qa.rotation_rad, qb.rotation_rad, { ...opts, label: `q${i}.rotation` });
    if (er) return er;
  }
  return null;
}

function compareQueriesScaled(
  a: SimResult, b: SimResult, alpha: number,
  opts: { absTol?: number; relTol?: number } = {},
): string | null {
  if (a.queryResults.length !== b.queryResults.length) return `length mismatch`;
  for (let i = 0; i < a.queryResults.length; i++) {
    const qa = a.queryResults[i]!, qb = b.queryResults[i]!;
    const ed = compareEnvelopesScaled(qa.deflection_mm, qb.deflection_mm, alpha, { ...opts, label: `q${i}.deflection` });
    if (ed) return ed;
    const er = compareEnvelopesScaled(qa.rotation_rad, qb.rotation_rad, alpha, { ...opts, label: `q${i}.rotation` });
    if (er) return er;
  }
  return null;
}

// Throwing form for use inside fc.property predicates.
function passOrThrow(msg: string | null): true { if (msg) throw new Error(msg); return true; }

// ---------- Properties ----------

const RUNS = 200;

console.log('\n== sim/ metamorphic properties ==');

// P1. Rotation equivariance: rotate the whole problem by R; outputs are R·original.
prop('P1 rotation equivariance', RUNS, () =>
  fc.property(
    arbProblem(),
    fc.tuple(
      fc.double({ min: -1, max: 1, noNaN: true }),
      fc.double({ min: 0, max: 2 * Math.PI, noNaN: true }),
      fc.double({ min: -Math.PI, max: Math.PI, noNaN: true }),
    ),
    (problem: Problem, [cosTheta, phi, ang]) => {
      const sinTheta = Math.sqrt(Math.max(0, 1 - cosTheta * cosTheta));
      const axis: Vec3 = [sinTheta * Math.cos(phi), sinTheta * Math.sin(phi), cosTheta];
      const R = rotMat(axis, ang);
      const a = mustOk(simulate(problem), 'original');
      const b = mustOk(simulate(rotateProblem(problem, R)), 'rotated');
      if (!a || !b) return true;
      return passOrThrow(compareQueries(a, b, { rotate: R, posRotate: R, absTol: 1e-6, relTol: 1e-8 }));
    },
  ),
);

// P2. Force scaling: Fmax × α  ⇒ envelope.support(d) × α.
prop('P2 force scaling', RUNS, () =>
  fc.property(
    arbProblem({ nLoads: { min: 1, max: 3 } }),
    fc.double({ min: 0.01, max: 100, noNaN: true }),
    (problem, alpha) => {
      const a = mustOk(simulate(problem), 'orig');
      const b = mustOk(simulate(scaleLoads(problem, alpha)), 'scaled');
      if (!a || !b) return true;
      return passOrThrow(compareQueriesScaled(a, b, alpha, { absTol: 1e-7, relTol: 1e-9 }));
    },
  ),
);

// P3. Material scaling: E,G × α ⇒ envelope.support(d) × 1/α.
prop('P3 material scaling', RUNS, () =>
  fc.property(
    arbProblem({ nLoads: { min: 1, max: 3 } }),
    fc.double({ min: 0.01, max: 100, noNaN: true }),
    (problem, alpha) => {
      const a = mustOk(simulate(problem), 'orig');
      const b = mustOk(simulate(scaleMaterial(problem, alpha)), 'scaled');
      if (!a || !b) return true;
      return passOrThrow(compareQueriesScaled(a, b, 1 / alpha, { absTol: 1e-7, relTol: 1e-9 }));
    },
  ),
);

// P4. Query permutation: queryResults reorder identically.
prop('P4 query permutation equivariance', RUNS, () =>
  fc.property(
    arbProblem({ nQueries: { min: 2, max: 4 } }),
    fc.infiniteStream(fc.double({ min: 0, max: 1, noNaN: true })),
    (problem, stream) => {
      const n = problem.queries.length;
      const perm = Array.from({ length: n }, (_, i) => i);
      const it = stream[Symbol.iterator]();
      for (let i = n - 1; i > 0; i--) {
        const r = it.next().value as number;
        const j = Math.min(i, Math.floor(r * (i + 1)));
        const t = perm[i]!; perm[i] = perm[j]!; perm[j] = t;
      }
      const permuted: Problem = { ...problem, queries: perm.map((i) => problem.queries[i]!) };
      const a = mustOk(simulate(problem), 'orig');
      const b = mustOk(simulate(permuted), 'permuted');
      if (!a || !b) return true;
      for (let i = 0; i < n; i++) {
        const qaOrig = a.queryResults[perm[i]!]!;
        const qbNew = b.queryResults[i]!;
        const ed = compareEnvelopes(qaOrig.deflection_mm, qbNew.deflection_mm, { label: `q${i}.deflection` });
        if (ed) throw new Error(ed);
      }
      return true;
    },
  ),
);

// P5. Load permutation invariance: envelopes unchanged (Minkowski sum is commutative).
prop('P5 load permutation invariance', RUNS, () =>
  fc.property(
    arbProblem({ nLoads: { min: 2, max: 4 } }),
    fc.infiniteStream(fc.double({ min: 0, max: 1, noNaN: true })),
    (problem, stream) => {
      const n = problem.loads.length;
      const perm = Array.from({ length: n }, (_, i) => i);
      const it = stream[Symbol.iterator]();
      for (let i = n - 1; i > 0; i--) {
        const r = it.next().value as number;
        const j = Math.min(i, Math.floor(r * (i + 1)));
        const t = perm[i]!; perm[i] = perm[j]!; perm[j] = t;
      }
      const permuted: Problem = { ...problem, loads: perm.map((i) => problem.loads[i]!) };
      const a = mustOk(simulate(problem), 'orig');
      const b = mustOk(simulate(permuted), 'permuted');
      if (!a || !b) return true;
      return passOrThrow(compareQueries(a, b));
    },
  ),
);

// P6. Zero-load no-op: adding a load with Fmax_N = 0 changes nothing.
prop('P6 zero-load no-op', RUNS, () =>
  fc.property(
    arbProblem(),
    fc.tuple(fc.double({ min: 0, max: 1, noNaN: true }), fc.double({ min: 0, max: 1, noNaN: true })),
    (problem, [bFrac, oFrac]) => {
      const bIx = Math.min(problem.beams.length - 1, Math.floor(bFrac * problem.beams.length));
      const padded: Problem = {
        ...problem,
        loads: [...problem.loads, { beamIx: bIx, offset_mm: oFrac * problem.beams[bIx]!.length_mm, Fmax_N: 0 }],
      };
      const a = mustOk(simulate(problem), 'orig');
      const b = mustOk(simulate(padded), 'padded');
      if (!a || !b) return true;
      return passOrThrow(compareQueries(a, b));
    },
  ),
);

// P7. Query insertion no-op: pre-existing queryResults are unchanged when a new
// query is appended.
prop('P7 query insertion no-op', RUNS, () =>
  fc.property(
    arbProblem(),
    fc.tuple(fc.double({ min: 0, max: 1, noNaN: true }), fc.double({ min: 0, max: 1, noNaN: true })),
    (problem, [bFrac, oFrac]) => {
      const bIx = Math.min(problem.beams.length - 1, Math.floor(bFrac * problem.beams.length));
      const extended: Problem = {
        ...problem,
        queries: [...problem.queries, { beamIx: bIx, offset_mm: oFrac * problem.beams[bIx]!.length_mm }],
      };
      const a = mustOk(simulate(problem), 'orig');
      const bAll = mustOk(simulate(extended), 'extended');
      if (!a || !bAll) return true;
      // Compare only the pre-existing queries (the new one is at index n).
      const b: SimResult = { kind: 'ok', queryResults: bAll.queryResults.slice(0, a.queryResults.length) as DeflectionQueryResult[] };
      return passOrThrow(compareQueries(a, b));
    },
  ),
);

// P8. Beam-split equivalence: replace the LAST beam with two collinear segments.
// (Splits mid-chain are excluded by the serial-chain rule; see splitBeam comment.)
prop('P8 beam split equivalence', RUNS, () =>
  fc.property(
    arbProblem(),
    fc.double({ min: 0.1, max: 0.9, noNaN: true }),
    (problem, splitFrac) => {
      const bIx = problem.beams.length - 1;
      // Skip if 'both' support + only one beam: splitting beam[0] would move
      // the second clamp to the second sub-beam's tip, which legitimately
      // changes the structure under the 'second clamp = beam[0].tip' convention.
      if (problem.support === 'both' && bIx === 0) return true;
      const L = problem.beams[bIx]!.length_mm;
      const s = splitFrac * L;
      const split = splitBeam(problem, bIx, s);
      const a = mustOk(simulate(problem), 'orig');
      const b = mustOk(simulate(split), 'split');
      if (!a || !b) return true;
      return passOrThrow(compareQueries(a, b, { absTol: 1e-6, relTol: 1e-8 }));
    },
  ),
);

// P9. Free-appendage invariance: appending a force/query-less beam at the tail
// of the chain (anywhere on the last beam's axis, any orientation) doesn't
// change the response at existing queries. The serial-chain rule forbids
// mid-chain branches, so this is the strongest structural form we can express.
prop('P9 free-appendage invariance', RUNS, () =>
  fc.property(
    arbProblem({ support: 'single' }),
    arbAppendageSpec,
    (problem, [sFrac, rollAng, len, section, material, _parentFrac]) => {
      const last = problem.beams[problem.beams.length - 1]!;
      const s = sFrac * last.length_mm;
      const appended = appendAppendage(problem, s, len, rollAng, section, material);
      const a = mustOk(simulate(problem), 'orig');
      const b = mustOk(simulate(appended), 'appended');
      if (!a || !b) return true;
      return passOrThrow(compareQueries(a, b, { absTol: 1e-7, relTol: 1e-9 }));
    },
  ),
);

// P10. Envelope sign symmetry: support(d) = support(-d) for every query, every direction.
prop('P10 envelope sign symmetry', RUNS, () =>
  fc.property(
    arbProblem({ nLoads: { min: 1, max: 3 } }),
    (problem) => {
      const a = mustOk(simulate(problem), 'orig');
      if (!a) return true;
      for (let i = 0; i < a.queryResults.length; i++) {
        const env = a.queryResults[i]!.deflection_mm;
        for (const d of [[1, 0, 0], [0, 1, 0], [0, 0, 1], [0.6, 0.8, 0], [0.5, 0.5, 0.7071]] as Vec3[]) {
          const dNeg: Vec3 = [-d[0], -d[1], -d[2]];
          const vp = env.support(d), vn = env.support(dNeg);
          const diff = Math.abs(vp - vn);
          const scale = Math.max(Math.abs(vp), Math.abs(vn), 1);
          if (diff > 1e-9 && diff / scale > 1e-12) {
            throw new Error(`q${i}: support(d)=${vp} support(-d)=${vn} d=[${d.join(',')}]`);
          }
        }
      }
      return true;
    },
  ),
);

// P11. Empty cases.
prop('P11 empty problem', 1, () =>
  fc.property(fc.constant(null), () => {
    const out = simulate({ beams: [], loads: [], queries: [], support: 'single' });
    if (out.kind !== 'ok' || out.queryResults.length !== 0) {
      throw new Error('expected ok with no queryResults');
    }
    return true;
  }),
);

prop('P11 zero loads → zero envelope', RUNS, () =>
  fc.property(
    arbProblem({ nLoads: { min: 0, max: 0 }, nQueries: { min: 1, max: 2 } }),
    (problem) => {
      const out = mustOk(simulate(problem), 'noLoads');
      if (!out) return true;
      for (const qr of out.queryResults) {
        for (const d of [[1, 0, 0], [0, 1, 0], [0, 0, 1]] as Vec3[]) {
          if (qr.deflection_mm.support(d) > 1e-12) {
            throw new Error(`q${qr.queryIx}: expected zero envelope, support([${d}])=${qr.deflection_mm.support(d)}`);
          }
        }
      }
      return true;
    },
  ),
);

// ---------- Summary ----------

console.log(`\n${testsFailed === 0 ? 'all properties passed' : `${testsFailed} of ${testsRun} properties failed`}`);
process.exit(testsFailed === 0 ? 0 : 1);
