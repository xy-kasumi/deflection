// Sim-only smoke test. Hand-builds `Problem`s and checks the deflection math
// against closed-form cantilever formulas — no DSL, no walker, no state. That
// it runs at all proves sim/ stands alone as a library.
// Run with: npx tsx scripts/smoke-compliance.ts

import type { Beam, Problem } from '../src/sim/problem';
import type { Vec3 } from '../src/sim/math';
import { buildCompliances } from '../src/sim/compliance';
import { simulate } from '../src/sim/simulate';
import type { Mode } from '../src/sim/compliance';

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
    check('δ(+y) = F·L³/(3·E·Ix)', q.deflection_mm.support([0, 1, 0]), expBend);
    checkNear0('δ(+x) = 0 (axially rigid)', q.deflection_mm.support([1, 0, 0]));
    const { point, distance } = q.deflection_mm.furthest();
    check('δ_max = F·L³/(3·E·Ix)', distance, expBend);
    expect('d* lies in the YZ plane (chord is rigid)', Math.abs(point[0]) < 1e-3 * distance);
    const inv = 1 / distance;
    const dStar: Vec3 = [point[0] * inv, point[1] * inv, point[2] * inv];

    // boundary(d*) is the surface point with outer normal d*. At the
    // argmax direction this point equals furthest().point exactly
    // (the supporting hyperplane is tangent to K at the furthest point).
    const bdy = q.deflection_mm.boundary(dStar);
    check('|boundary(d*)| = δ_max', Math.hypot(bdy[0], bdy[1], bdy[2]), distance, 1e-9);
    expect(
      'boundary(d*) = furthest().point',
      Math.abs(bdy[0] - point[0]) +
      Math.abs(bdy[1] - point[1]) +
      Math.abs(bdy[2] - point[2]) < 1e-9 * distance + 1e-12,
    );

    // δ_max decomposition invariants. At a single direction d*, summing the
    // single-load envelopes' support over all (b, m, p) must reproduce
    // δ(d*) = δ_max exactly (linear support function in F).
    let perBeamSum = 0, perLoadSum = 0;
    const perLoad = new Map<number, number>();
    for (const b of q.beamDeflections) {
      let bsum = 0;
      for (const bm of b.byMode) {
        for (const pl of bm.perLoad) {
          const v = pl.deflection_mm.support(dStar);
          bsum += v;
          perLoad.set(pl.loadIx, (perLoad.get(pl.loadIx) ?? 0) + v);
        }
      }
      perBeamSum += bsum;
    }
    for (const v of perLoad.values()) perLoadSum += v;
    check('Σ_(b,m,p) F·|C^T d*| = δ_max (by beam)', perBeamSum, distance, 1e-9);
    check('Σ_(b,m,p) F·|C^T d*| = δ_max (by load)', perLoadSum, distance, 1e-9);
  }
}

// ---- Test 4: per-mode δ at d=+y — pure bendIxTrans on the load beam ----
{
  console.log('\n-- Test 4: per-mode δ at d=+y --');
  const out = simulate(singleCantilever());
  if (out.kind !== 'ok') {
    expect('simulate ok', false);
  } else {
    const q = out.queryResults[0]!;
    expect('1 beam in beamDeflections', q.beamDeflections.length === 1);
    const b = q.beamDeflections[0]!;
    const at_y = (m: Mode): number =>
      b.byMode.find((x) => x.mode === m)?.deflection_mm.support([0, 1, 0]) ?? 0;
    const expBend = (KGF_TO_N * L ** 3) / (3 * EIx);
    check('bendIxTrans @ d=+y = F·L³/(3·E·Ix)', at_y('bendIxTrans'), expBend);
    checkNear0('bendIyTrans @ d=+y = 0', at_y('bendIyTrans'));
    // *Rot modes and twist are zero on-beam: arm = 0 ⇒ no transport.
    checkNear0('bendIxRot @ d=+y = 0', at_y('bendIxRot'));
    checkNear0('bendIyRot @ d=+y = 0', at_y('bendIyRot'));
    checkNear0('twist @ d=+y = 0', at_y('twist'));
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

// ---- Test 7: non-tip attachment (child branches mid-axis of parent) ----
// Regression: the compliance solver used to substitute parent.length_mm
// everywhere it actually meant "where the chain exits this beam." For
// tip-to-root chains the two coincide, but a `mid:` (or `100:`, etc.) child
// makes them differ — the parent's segment past the attachment is unloaded
// and an on-beam query at the attach point has arm = 0 from where the chain
// exits, so the parent's bend-rotation and twist must contribute nothing to
// translation there.
{
  console.log('\n-- Test 7: mid-attached child branch --');
  const L0 = 300;
  const Lattach = 100;
  const L1 = 100;
  // beam1: right turn off horz beam0 at offset Lattach. Walker convention
  // ex↔left, ey↔up, axial↔fwd. Horz parent: F=+X, L=-Z, U=+Y. Right turn:
  // F'=+Z, L'=+X, U'=+Y. So beam1: axial=+Z, ex=+X, ey=+Y.
  const beam1: Beam = {
    frame: {
      origin_mm: [Lattach, 0, 0],
      ex: [1, 0, 0], ey: [0, 1, 0], axial: [0, 0, 1],
    },
    length_mm: L1,
    section: RECT_10,
    material: STEEL,
  };
  const problem: Problem = {
    beams: [horzBeam([0, 0, 0], L0), beam1],
    loads: [{ beamIx: 1, offset_mm: L1, Fmax_N: KGF_TO_N }],
    queries: [{ beamIx: 1, offset_mm: 0 }],
    support: 'single',
  };
  const out = simulate(problem);
  if (out.kind !== 'ok') {
    expect('simulate ok', false);
  } else {
    const q = out.queryResults[0]!;
    const b0 = q.beamDeflections.find((b) => b.beamIx === 0);
    expect('beam0 in beamDeflections', !!b0);
    if (b0) {
      const at = (m: Mode, d: Vec3): number =>
        b0.byMode.find((x) => x.mode === m)?.deflection_mm.support(d) ?? 0;
      // δ_y from beam0 at the attach point comes purely from bendIxTrans,
      // computed against Lattach (not L0). Pre-fix this used L0 — about
      // 8× the correct value — and bendIxRot/twist were spuriously nonzero
      // via a bogus arm = queryPos - beam0_tip.
      const expBend = (KGF_TO_N * Lattach ** 3) / (3 * EIx);
      check('beam0 bendIxTrans @ +y = F·L_attach³/(3·E·Ix)', at('bendIxTrans', [0, 1, 0]), expBend, 1e-9);
      checkNear0('beam0 bendIxRot @ +y = 0 (arm=0)', at('bendIxRot', [0, 1, 0]));
      checkNear0('beam0 bendIyRot @ +y = 0 (arm=0)', at('bendIyRot', [0, 1, 0]));
      checkNear0('beam0 twist     @ +y = 0 (arm=0)', at('twist',       [0, 1, 0]));
    }
  }
}

console.log(failed ? '\nSMOKE FAILED' : '\nsmoke ok');
process.exitCode = failed ? 1 : 0;
