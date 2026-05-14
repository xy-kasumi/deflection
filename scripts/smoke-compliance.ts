// Sim-only smoke test. Hand-builds `Problem`s and checks the deflection math
// against closed-form cantilever formulas — no DSL, no walker, no state. That
// it runs at all proves sim/ stands alone as a library.
// Run with: npx tsx scripts/smoke-compliance.ts

import type { Beam, Problem, Vec3 } from '../src/sim/problem';
import { buildCompliances } from '../src/sim/compliance';
import { decompose } from '../src/sim/decompose';
import { simulate } from '../src/sim/simulate';

const STEEL = { E_MPa: 200_000, G_MPa: 79_000 };
const KGF_TO_N = 9.80665;

// rect(W10 H10): Ix = Iy = W·H³/12; J via the Roark solid-rectangle formula.
const RECT_10 = (() => {
  const W = 10, H = 10;
  const a = Math.max(W, H), b = Math.min(W, H), r = b / a;
  return {
    Ix_mm4: (W * H ** 3) / 12,
    Iy_mm4: (W ** 3 * H) / 12,
    J_mm4: a * b ** 3 * (1 / 3 - 0.21 * r * (1 - r ** 4 / 12)),
  };
})();

let failed = false;
function check(name: string, actual: number, expected: number, tolRel = 1e-12): void {
  const rel = Math.abs(actual - expected) / (Math.abs(expected) + 1e-30);
  const ok = rel < tolRel;
  if (!ok) failed = true;
  console.log(
    `${ok ? 'ok  ' : 'FAIL'} ${name}: got ${actual.toExponential(6)}, ` +
      `want ${expected.toExponential(6)} (rel ${rel.toExponential(2)})`,
  );
}
function checkNear0(name: string, actual: number, tolAbs = 1e-9): void {
  const ok = Math.abs(actual) < tolAbs;
  if (!ok) failed = true;
  console.log(`${ok ? 'ok  ' : 'FAIL'} ${name}: got ${actual.toExponential(6)} (want ~0)`);
}
function expect(name: string, cond: boolean): void {
  if (!cond) failed = true;
  console.log(`${cond ? 'ok  ' : 'FAIL'} ${name}`);
}

// A 'horz' cantilever beam: axial=+X, ex=-Z, ey=+Y (the walker horz root frame).
function horzBeam(origin_mm: Vec3, length_mm: number): Beam {
  return {
    frame: { origin_mm, ex: [0, 0, -1], ey: [0, 1, 0], axial: [1, 0, 0] },
    length_mm,
    section: RECT_10,
    material: STEEL,
  };
}

const L = 100;
const EIx = STEEL.E_MPa * RECT_10.Ix_mm4;
const EIy = STEEL.E_MPa * RECT_10.Iy_mm4;

function singleCantilever(): Problem {
  return {
    beams: [horzBeam([0, 0, 0], L)],
    loads: [{ beamIx: 0, offset_mm: L, Fmax_N: KGF_TO_N }],
    queries: [{ beamIx: 0, offset_mm: L }],
    support: 'single',
  };
}

// ---- Test 1: single horz cantilever, tip load — C_tot diagonal ----
{
  console.log('-- Test 1: single horz cantilever, C_tot --');
  const c = buildCompliances(singleCantilever());
  if ('kind' in c) {
    expect('buildCompliances ok', false);
  } else {
    const C = c.totals.find((x) => x.queryIx === 0 && x.loadIx === 0)!.C;
    checkNear0('C_tot[x,x] = 0 (axially rigid)', C[0]!);
    check('C_tot[y,y] = L³/(3·E·Ix)', C[4]!, L ** 3 / (3 * EIx));
    check('C_tot[z,z] = L³/(3·E·Iy)', C[8]!, L ** 3 / (3 * EIy));
    checkNear0('C_tot[x,y] = 0', C[1]!);
  }
}

// ---- Test 2: horz→up two-beam chain, tip load — cross-beam transport ----
{
  console.log('\n-- Test 2: horz→up two-beam chain, C_tot --');
  // Beam1 is beam0's 'up' turn: origin at beam0's tip, axial=+Y.
  const beam1: Beam = {
    frame: { origin_mm: [L, 0, 0], ex: [0, 0, -1], ey: [-1, 0, 0], axial: [0, 1, 0] },
    length_mm: L,
    section: RECT_10,
    material: STEEL,
  };
  const problem: Problem = {
    beams: [horzBeam([0, 0, 0], L), beam1],
    loads: [{ beamIx: 1, offset_mm: L, Fmax_N: KGF_TO_N }],
    queries: [{ beamIx: 1, offset_mm: L }],
    support: 'single',
  };
  const c = buildCompliances(problem);
  if ('kind' in c) {
    expect('buildCompliances ok', false);
  } else {
    // F_y at the tip: beam1 is chord-aligned (rigid), beam0 bends. So tip u_y
    // is L³/(3·E·Ix) from beam0 alone.
    const C = c.totals.find((x) => x.queryIx === 0 && x.loadIx === 0)!.C;
    check('C_tot[tip][load][y,y] = L³/(3·E·Ix)', C[4]!, L ** 3 / (3 * EIx), 1e-10);
  }
}

// ---- Test 3: simulate() end-to-end on the single cantilever ----
{
  console.log('\n-- Test 3: simulate single cantilever --');
  const out = simulate(singleCantilever());
  if (out.kind !== 'ok') {
    expect('simulate ok', false);
  } else {
    const q = out.queryResults[0]!;
    const expBend = (KGF_TO_N * L ** 3) / (3 * EIx);
    check('δ(+y) = F·L³/(3·E·Ix)', q.deflection_mm.at([0, 1, 0]), expBend);
    checkNear0('δ(+x) = 0 (axially rigid)', q.deflection_mm.at([1, 0, 0]));
    const { dir, value } = q.deflection_mm.max();
    check('δ_max = F·L³/(3·E·Ix)', value, expBend);
    expect('d* lies in the YZ plane (chord is rigid)', Math.abs(dir[0]) < 1e-3);
    check('Σ load contributions = δ_max', q.loads.reduce((s, l) => s + l.delta_mm, 0), value, 1e-9);
    check('Σ beam contributions = δ_max', q.beams.reduce((s, b) => s + b.delta_mm, 0), value, 1e-9);
  }
}

// ---- Test 4: decompose at an explicit d=+y — pure bendIx ----
{
  console.log('\n-- Test 4: decompose at d=+y --');
  const c = buildCompliances(singleCantilever());
  if ('kind' in c) {
    expect('buildCompliances ok', false);
  } else {
    const dec = decompose(c, 0, [0, 1, 0]);
    expect('1 load contribution', dec.perLoad.length === 1);
    expect('1 beam contribution', dec.perBeam.length === 1);
    if (dec.perLoad.length === 1) {
      check('load0 delta = F·L³/(3·E·Ix)', dec.perLoad[0]!.delta_mm, (KGF_TO_N * L ** 3) / (3 * EIx));
    }
    if (dec.perBeam.length === 1) {
      const b = dec.perBeam[0]!;
      check('beam0 total = bendIx (pure bend at d=+y)', b.total_mm, b.bendIx_mm);
      checkNear0('beam0 bendIy = 0', b.bendIy_mm);
      checkNear0('beam0 torsion = 0', b.torsionJ_mm);
    }
  }
}

// ---- Test 5: support(both) solves; one reported query ----
{
  console.log('\n-- Test 5: support(both) single beam, mid load --');
  const problem: Problem = {
    beams: [horzBeam([0, 0, 0], L)],
    loads: [{ beamIx: 0, offset_mm: L / 2, Fmax_N: KGF_TO_N }],
    queries: [{ beamIx: 0, offset_mm: L / 2 }],
    support: 'both',
  };
  const out = simulate(problem);
  expect('simulate succeeds', out.kind === 'ok');
  if (out.kind === 'ok') {
    expect('one query result (clamp query not reported)', out.queryResults.length === 1);
  }
}

// ---- Test 6: input validation → SimError ----
{
  console.log('\n-- Test 6: input validation --');
  const offRoot = simulate({
    beams: [horzBeam([5, 0, 0], L)],
    loads: [],
    queries: [],
    support: 'single',
  });
  expect(
    'root not at origin → SimError(root-not-at-origin)',
    offRoot.kind === 'error' && offRoot.code === 'root-not-at-origin',
  );

  const disconnected = simulate({
    beams: [horzBeam([0, 0, 0], L), horzBeam([0, 50, 0], L)],
    loads: [],
    queries: [],
    support: 'single',
  });
  expect(
    'beam off parent axis → SimError(beams-disconnected)',
    disconnected.kind === 'error' && disconnected.code === 'beams-disconnected',
  );
}

console.log(failed ? '\nSMOKE FAILED' : '\nsmoke ok');
process.exitCode = failed ? 1 : 0;
