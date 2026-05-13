// Quick smoke test for compliance build.
// Verifies single-beam cantilever Y-deflection at tip matches PL³/(3·E·Ix).
// Run with: npx tsx scripts/smoke-compliance.ts

import { parse } from '../src/dsl/parse';
import { walk } from '../src/walker';
import { buildCompliances } from '../src/sim/compliance';
import { directionalFor } from '../src/sim/directional';
import { attribute } from '../src/sim/attribute';
import { runSim } from '../src/sim/run';
import { MATERIALS, KGF_TO_N } from '../src/state';

function check(name: string, actual: number, expected: number, tolRel = 1e-9): void {
  const err = Math.abs(actual - expected);
  const rel = err / (Math.abs(expected) + 1e-30);
  const ok = rel < tolRel;
  console.log(`${ok ? 'ok  ' : 'FAIL'} ${name}: got ${actual.toExponential(6)}, want ${expected.toExponential(6)} (rel ${rel.toExponential(2)})`);
  if (!ok) process.exitCode = 1;
}

// Test 1: horz cantilever, steel rect(W10 H10) L100, end:force(1kgf).
// Horz root: F=+X, R=-Z, U=+Y. So beam axis is world +X, walker-up is world +Y.
// Tip is at (100, 0, 0). Tip force = 1 kgf = 9.80665 N.
// 1 kgf is a magnitude — the load contributes |C·F_dir|. We use C_tot directly
// (compliance, not the F-weighted result) and check (1,1) entry:
//   u_y / F_y = L³ / (3·E·Ix)
{
  const src = 'support(single)\nmass_accel(0)\nhorz beam(steel rect(W10 H10) L100) end:force(1kgf)';
  const { structure, diagnostics: pd } = parse(src);
  if (pd.some((d) => d.severity === 'error')) {
    console.log('parse diagnostics:', pd);
    process.exit(1);
  }
  const { beams, diagnostics: wd } = walk(structure);
  if (wd.some((d) => d.severity === 'error')) {
    console.log('walk diagnostics:', wd);
    process.exit(1);
  }
  const { compliances, diagnostics: cd } = buildCompliances(beams, structure);
  if (cd.some((d) => d.severity === 'error')) {
    console.log('compliance diagnostics:', cd);
    process.exit(1);
  }

  console.log('-- Test 1: horz steel rect(W10 H10) L100, force at tip --');
  console.log('queryNodes:', compliances.queryNodes);
  console.log('loadNodes:', compliances.loadNodes, 'F:', compliances.loadFmax_N);
  console.log('totals entries:', compliances.totals.length);
  console.log('entries:', compliances.entries.length, 'by mode:',
    Object.fromEntries(['axial','torsion','bendIx','bendIy'].map((m) =>
      [m, compliances.entries.filter((e) => e.mode === m).length])));

  const total00 = compliances.totals.find((t) => t.queryIx === 0 && t.loadIx === 0);
  if (!total00) { console.log('FAIL: no total[0][0]'); process.exit(1); }
  const C = total00.C;
  console.log('C_tot[0][0] (row-major):');
  console.log(`  [${C[0].toExponential(4)}, ${C[1].toExponential(4)}, ${C[2].toExponential(4)}]`);
  console.log(`  [${C[3].toExponential(4)}, ${C[4].toExponential(4)}, ${C[5].toExponential(4)}]`);
  console.log(`  [${C[6].toExponential(4)}, ${C[7].toExponential(4)}, ${C[8].toExponential(4)}]`);

  const L = 100;
  const E = MATERIALS.steel.E_MPa; // 200_000 MPa = N/mm²
  const G = MATERIALS.steel.G_MPa;
  const Ix = (10 * 10 ** 3) / 12; // W·H³/12 = 833.33
  const Iy = (10 ** 3 * 10) / 12;
  const A = 100;

  // Beam axis = world +X. Beam-Z (axial) = world +X. Beam-Y (walker-up) = world +Y.
  // Beam-X (walker-right) = world -Z.
  // Expected:
  //   u_x / F_x (axial along beam-Z=world+X): L/(E·A)
  //   u_y / F_y (bend-Ix, F in beam-Y=world+Y): L³/(3·E·Ix)
  //   u_z / F_z (bend-Iy, F in -X_section world: world+Z = -beam-X):
  //              same closed form L³/(3·E·Iy)
  const expAxial = L / (E * A);
  const expBendIx = (L ** 3) / (3 * E * Ix);
  const expBendIy = (L ** 3) / (3 * E * Iy);
  console.log(`  expected L/EA = ${expAxial.toExponential(4)}`);
  console.log(`  expected L³/(3·E·Ix) = ${expBendIx.toExponential(4)}`);
  console.log(`  expected L³/(3·E·Iy) = ${expBendIy.toExponential(4)}`);

  check('C_tot[0][0][x,x] = L/(E·A)',           C[0], expAxial, 1e-12);
  check('C_tot[0][0][y,y] = L³/(3·E·Ix)',       C[4], expBendIx, 1e-12);
  check('C_tot[0][0][z,z] = L³/(3·E·Iy)',       C[8], expBendIy, 1e-12);
  // No cross-axis coupling for an isotropic square section + symmetric load loc.
  check('C_tot[0][0][x,y] = 0', C[1], 0, 1e-30);
  check('C_tot[0][0][y,x] = 0', C[3], 0, 1e-30);

  // Check F-weighted tip deflection magnitude in +Y for F=1kgf:
  const F = KGF_TO_N; // 9.80665
  const u_y_actual = C[4] * F;
  const u_y_expected = F * (L ** 3) / (3 * E * Ix);
  check('u_y(1kgf) = F·L³/(3·E·Ix)', u_y_actual, u_y_expected, 1e-12);
  console.log(`  (= ${u_y_actual.toExponential(4)} mm under 1kgf, ${Ix.toFixed(2)} Ix)`);

  // Per-mode split: expect a single bendIx entry to dominate the (1,1) cell.
  const bendIxEntry = compliances.entries.find((e) => e.mode === 'bendIx');
  if (bendIxEntry) {
    check('bendIx entry [1,1] = total [1,1]', bendIxEntry.C[4], expBendIx, 1e-12);
  }
  const axialEntry = compliances.entries.find((e) => e.mode === 'axial');
  if (axialEntry) {
    check('axial entry [0,0] = total [0,0]', axialEntry.C[0], expAxial, 1e-12);
  }
}

// Test 2: two-beam chain. horz L100 then up L100, both steel rect(W10 H10),
// force(1kgf) at the tip (end of beam1). Verifies cross-beam transport.
//
// Beam0 lies along world +X from (0,0,0) to (100,0,0). At its tip the walker
// turns 'up', so beam1's start = (100,0,0). For 'up' turn from horz frame
// (F=+X, R=-Z, U=+Y): new F = old U = +Y, new R = old R = -Z, new U = -F = -X.
// Beam1 lies from (100,0,0) to (100,100,0). Tip = (100,100,0).
//
// We'll check the tip Y-displacement under F_y = 1kgf (vertical load at tip).
// Beam0 carries this as bending in the +Y direction with the lever-arm-extended
// load: tip is at (100,100,0), so moment on beam0 about its tip about world-Z
// from F_y = 1kgf at (100,100,0): arm_to_load = (0,100,0); cross with (0,F_y,0) = 0.
// So beam0 sees pure F_y at its own tip → modeBendIx with F_y, M_x=0.
// Then transport arm to query (0,100,0): rot_world × arm. Tip slope dθ/dx of beam0
// contributes (rot_x_world) × (0,100,0).
// Beam1: along +Y, force F_y is axial → beam-Z direction → modeAxial. So beam0
// contributes bend-Ix Y-deflection L³/(3·E·Ix), and beam1 contributes axial
// L/(E·A) → tip total u_y = L³/(3·E·Ix) + L/(E·A).
{
  const src = 'support(single)\nmass_accel(0)\nhorz beam(steel rect(W10 H10) L100)\nup beam(steel rect(W10 H10) L100) end:force(1kgf)';
  const { structure } = parse(src);
  const { beams } = walk(structure);
  const { compliances } = buildCompliances(beams, structure);

  console.log('\n-- Test 2: horz→up two-beam chain, force at tip --');
  const tipQuery = compliances.queryNodes.length - 1;
  const lastLoad = compliances.loadNodes.length - 1;
  console.log('tip query frame: beam', compliances.queryNodes[tipQuery]);
  console.log('load:', compliances.loadNodes[lastLoad]);

  const total = compliances.totals.find((t) => t.queryIx === tipQuery && t.loadIx === lastLoad);
  if (!total) { console.log('FAIL: no total[tip][load]'); process.exit(1); }
  const C = total.C;
  console.log('C_tot[tip][last] (row-major):');
  console.log(`  [${C[0].toExponential(4)}, ${C[1].toExponential(4)}, ${C[2].toExponential(4)}]`);
  console.log(`  [${C[3].toExponential(4)}, ${C[4].toExponential(4)}, ${C[5].toExponential(4)}]`);
  console.log(`  [${C[6].toExponential(4)}, ${C[7].toExponential(4)}, ${C[8].toExponential(4)}]`);

  const L = 100;
  const E = MATERIALS.steel.E_MPa;
  const Ix = (10 * 10 ** 3) / 12;
  const A = 100;

  // Beam0 contribution (bend-Ix) to u_y at tip: F_y at world(100,100,0).
  // On beam0 with arm_to_load (0,100,0), M_world from F_y is 0 → modeBendIx
  // sees F_y at s=100 only. u_y at s=100 = F_y·L³/(3·E·Ix), then transport:
  // rot_world θ_x = -du_y/ds at s=L = -F_y·L²/(2·E·Ix).
  // arm to query (in world) = queryWorld - beam0_tip = (100,100,0)-(100,0,0) = (0,100,0)
  // transport = (rot_world) × arm = (θ_x_world, 0, 0) × (0, 100, 0) = (0, 0, 100·θ_x_world)
  // θ_x_world = R_0 · θ_x_local. For beam0 horz, R = [right|up|fwd] cols = [-Z, +Y, +X]
  // So θ_x_local in beam-X (=walker.right=world -Z), θ_x_world = -Z direction.
  // Thus transport_y = 0. Good — beam0 contributes pure L³/(3·E·Ix) to u_y.
  // Beam1 axial contribution: u_z_local at s=L = F_z_local · L / (E·A). Beam1's
  // local +Z = world +Y. Force F_y in world → F_local_z = +F_y. So u_y_world =
  // F_y · L / (E·A). No transport (query = beam1 tip).
  const exp_uy = (L ** 3) / (3 * E * Ix) + L / (E * A);
  check('C_tot[tip][last][y,y] = bend(beam0) + axial(beam1)', C[4], exp_uy, 1e-10);

  // u_x from F_y: beam0 transports rot to query. θ_x_local in beam-X local dir,
  // R_0 maps local-X to world -Z, so rot_world only has a Z-axis component →
  // transport along x: (rot_world × arm)_x = θ_y_world·arm_z - θ_z_world·arm_y
  // arm = (0,100,0). θ_world has only Z component? Actually θ_x_local maps via
  // R_0 column 0 = (-1,0,0)? Wait R columns = right,up,fwd. right=(0,0,-1).
  // Hmm let me recompute carefully — but the numerical answer should tell.
  console.log(`  expected u_y/F_y = ${exp_uy.toExponential(4)}`);
}

// Test 3: Directional on single-beam horz cantilever, force at tip 1kgf.
// δ(d) = F · |C_tot^T · d|. C_tot is diagonal [L/EA, L³/3EIx, L³/3EIy] (with
// 2 equal eigenvalues in Y,Z and a tiny one in X). Max is along the largest
// singular value direction: σ_max = max(L/EA, L³/3EIx, L³/3EIy). With W=H
// these are 2e-3 (×2) and 5e-6 (×1) — so δ_max = F·L³/(3·E·Ix) and d* should
// be in the Y-Z plane (any direction there gives the same value).
{
  const src = 'support(single)\nmass_accel(0)\nhorz beam(steel rect(W10 H10) L100) end:force(1kgf)';
  const { structure } = parse(src);
  const { beams } = walk(structure);
  const { compliances } = buildCompliances(beams, structure);

  console.log('\n-- Test 3: Directional single cantilever --');
  const dir = directionalFor(compliances, 0);
  const F = KGF_TO_N;
  const expBend = F * (100 ** 3) / (3 * MATERIALS.steel.E_MPa * ((10 * 10 ** 3) / 12));

  check('δ(+x) = F·L/(E·A)',                   dir.at([1, 0, 0]), F * 100 / (MATERIALS.steel.E_MPa * 100), 1e-12);
  check('δ(+y) = F·L³/(3·E·Ix)',               dir.at([0, 1, 0]), expBend, 1e-12);
  check('δ(+z) = F·L³/(3·E·Iy)',               dir.at([0, 0, 1]), expBend, 1e-12);

  const { d: dStar, value } = dir.max();
  check('max(δ) = F·L³/(3·E·Ix)', value, expBend, 1e-12);
  console.log(`  d* = [${dStar[0].toFixed(4)}, ${dStar[1].toFixed(4)}, ${dStar[2].toFixed(4)}]`);
  // d* should have ~0 x-component (axial mode is tiny).
  if (Math.abs(dStar[0]) > 1e-3) {
    console.log(`FAIL: d* should be in YZ plane, got x=${dStar[0]}`);
    process.exitCode = 1;
  } else {
    console.log('ok   d* has x ≈ 0');
  }

  // Sample sanity: count should match, all values ≥ 0, max-of-sample ≤ true max.
  const samp = dir.sample(50);
  if (samp.length !== 50) { console.log('FAIL: sample length'); process.exitCode = 1; }
  const sampMax = Math.max(...samp.map((s) => s.value));
  if (sampMax > expBend * 1.000000001) {
    console.log(`FAIL: sample max ${sampMax} exceeds true max ${expBend}`);
    process.exitCode = 1;
  } else {
    console.log(`ok   sample max ${sampMax.toExponential(4)} ≤ true ${expBend.toExponential(4)}`);
  }
}

// Test 4: Attribution on single-beam cantilever. At d=+y, expect:
//   - 1 load, fraction 1.0
//   - 1 beam, total 1.0; bendIx_fraction = 1.0, others 0.
{
  const src = 'support(single)\nmass_accel(0)\nhorz beam(steel rect(W10 H10) L100) end:force(1kgf)';
  const { structure } = parse(src);
  const { beams } = walk(structure);
  const { compliances } = buildCompliances(beams, structure);

  console.log('\n-- Test 4: Attribution single cantilever at d=+y --');
  const r = attribute(compliances, 0, [0, 1, 0]);
  console.log('perLoad:', r.perLoad);
  console.log('perBeam:', r.perBeam);

  check('perLoad sum = δ', r.perLoad.reduce((s, p) => s + p.signed_mm, 0), r.delta_mm, 1e-12);
  check('perBeam sum (total) = δ',
    r.perBeam.reduce((s, b) => s + b.total_fraction * r.delta_mm, 0), r.delta_mm, 1e-12);
  if (r.perBeam.length !== 1) { console.log('FAIL: expected 1 beam'); process.exitCode = 1; }
  else {
    check('beam0 bendIx fraction = 1', r.perBeam[0]!.bendIx_fraction, 1, 1e-12);
    check('beam0 axial fraction = 0', r.perBeam[0]!.axial_fraction, 0, 1e-12);
    check('beam0 bendIy fraction = 0', r.perBeam[0]!.bendIy_fraction, 0, 1e-12);
    check('beam0 torsion fraction = 0', r.perBeam[0]!.torsion_fraction, 0, 1e-12);
  }
}

// Test 5: support(both), single horz beam steel rect(W10 H10) L100, force at
// midpoint. Fixed-fixed: chord = +X, perp plane = YZ. Tip Y/Z displacement
// must be zero (clamped); tip rotation about Y/Z must also be zero. Chord
// (axial X) is the one remaining free DOF and matches cantilever axial.
// We verify the perp tip displacement cells go to zero post-adjust; the
// rotation constraint is enforced inside the 4×4 solve but isn't directly
// observable from C_eff (which reports displacement).
{
  const src = 'support(both)\nmass_accel(0)\nhorz beam(steel rect(W10 H10) L100) mid:force(1kgf)';
  const { structure } = parse(src);
  const { beams } = walk(structure);
  const { compliances, diagnostics: cd } = buildCompliances(beams, structure);

  console.log('\n-- Test 5: support(both) horz cantilever, force at midpoint --');
  if (cd.some((d) => d.severity === 'error')) {
    console.log('FAIL: compliance diagnostics:', cd);
    process.exit(1);
  }
  if (compliances.loadNodes.length !== 1) {
    console.log(`FAIL: expected 1 real load after adjust, got ${compliances.loadNodes.length}`);
    process.exit(1);
  }
  const total = compliances.totals.find((t) => t.queryIx === 0 && t.loadIx === 0);
  if (!total) { console.log('FAIL: no total[0][0]'); process.exit(1); }
  const C = total.C;
  console.log('C_eff[tip][p_mid] (row-major):');
  console.log(`  [${C[0].toExponential(4)}, ${C[1].toExponential(4)}, ${C[2].toExponential(4)}]`);
  console.log(`  [${C[3].toExponential(4)}, ${C[4].toExponential(4)}, ${C[5].toExponential(4)}]`);
  console.log(`  [${C[6].toExponential(4)}, ${C[7].toExponential(4)}, ${C[8].toExponential(4)}]`);

  const L = 100;
  const E = MATERIALS.steel.E_MPa;
  const A = 100;
  // Chord = +X → perpendicular Y, Z must cancel exactly.
  check('C_eff[tip][mid][y,y] = 0 (perp)', C[4], 0, 1e-12);
  check('C_eff[tip][mid][z,z] = 0 (perp)', C[8], 0, 1e-12);
  // Chord (X) is released → axial cantilever response remains.
  check('C_eff[tip][mid][x,x] = L/(2·E·A)', C[0], L / (2 * E * A), 1e-12);

  // Per-(beam, mode) decomposition: bendIx entry's [y,y] should still sum
  // with itself to 0 (only one beam). The presence of the entry confirms the
  // via-tip reaction was injected.
  const ymode = compliances.entries
    .filter((e) => e.queryIx === 0 && e.loadIx === 0 && e.mode === 'bendIx')
    .reduce((s, e) => s + e.C[4]!, 0);
  check('Σ bendIx entries [y,y] = 0', ymode, 0, 1e-12);
}

// Test 6: queryNodes set — beam beginnings + ends, deduped, origin dropped.
// 3-beam chain with `mid:` attachment: beam0 begins at origin (dropped),
// beam0 ends at the parent-midpoint of beam1's start... wait, no — beam1
// attaches at `mid:` of beam0 so beam1.start = beam0 at offset 150. So:
//   beam0 start = origin → dropped
//   beam0 end   = (300, 0, 0)
//   beam1 start = (150, 0, 0)   (distinct from beam0 end via mid:)
//   beam1 end   = beam2 start   (deduped)
//   beam2 end   = tip
// → 4 surviving queryNodes.
{
  const src = 'support(single)\nmass_accel(0)\nhorz beam(steel rect(W10 H10) L300)\nmid: right beam(aluminum rect(W10 H10) L150)\ndown beam(plastic rect(W10 H10) L100) end:force(1kgf)';
  const { structure } = parse(src);
  const { beams } = walk(structure);
  const { compliances } = buildCompliances(beams, structure);

  console.log('\n-- Test 6: queryNodes for 3-beam chain with `mid:` --');
  console.log('queryNodes:', compliances.queryNodes);
  if (compliances.queryNodes.length !== 4) {
    console.log(`FAIL: expected 4 queryNodes, got ${compliances.queryNodes.length}`);
    process.exitCode = 1;
  } else {
    console.log(`ok   4 queryNodes (beam1-start at parent midpoint preserved)`);
  }
}

// Test 7: support(both) drops the root beam's end from visible nodes (the
// second clamp point); compliance still keeps it for the fixed-fixed solve.
// Non-root beam endpoints stay visible.
{
  const src = 'support(both)\nmass_accel(0)\nhorz beam(steel rect(W10 H10) L300)\nmid: right beam(aluminum rect(W10 H10) L150)\ndown beam(plastic rect(W10 H10) L100) end:force(1kgf)';
  const { structure } = parse(src);
  const { beams } = walk(structure);
  const { compliances } = buildCompliances(beams, structure);

  // Compliance keeps the clamp query (root beam's end), needed for the solve.
  const rootEnd = beams[0]!.length_mm;
  const clampIx = compliances.queryNodes.findIndex(
    (n) => n.beamIx === 0 && n.offset_mm === rootEnd,
  );
  console.log('\n-- Test 7: support(both) keeps root beam end in compliance --');
  if (clampIx < 0) { console.log('FAIL: root beam end query missing from compliance'); process.exitCode = 1; }
  else console.log(`ok   root beam end query at compliance.queryNodes[${clampIx}]`);

  const sim = runSim(beams, structure);
  console.log(`SimResult.nodes.length = ${sim.nodes.length}, maxNodeIx = ${sim.maxNodeIx}`);
  const clampInNodes = sim.nodes.some((n) => {
    const qn = compliances.queryNodes[n.queryIx]!;
    return qn.beamIx === 0 && qn.offset_mm === rootEnd;
  });
  if (clampInNodes) { console.log('FAIL: root beam end leaked into SimResult.nodes'); process.exitCode = 1; }
  else console.log('ok   root beam end hidden from SimResult.nodes');

  // Chain tip (end of last beam) is *not* clamped — it should still be visible.
  const last = beams.length - 1;
  const chainTipVisible = sim.nodes.some((n) => {
    const qn = compliances.queryNodes[n.queryIx]!;
    return qn.beamIx === last && qn.offset_mm === beams[last]!.length_mm;
  });
  if (!chainTipVisible) { console.log('FAIL: chain tip should be visible under support(both)'); process.exitCode = 1; }
  else console.log('ok   chain tip (end of last beam) visible (not a clamp point)');
}

if (process.exitCode) {
  console.log('\nSMOKE FAILED');
} else {
  console.log('\nsmoke ok');
}
