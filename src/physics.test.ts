import { test } from 'node:test';
import assert from 'node:assert/strict';
import { computeCrossSection } from './physics';

function makeRng(seed: number) {
  let s = seed >>> 0;
  return () => {
    s = (s * 1664525 + 1013904223) >>> 0;
    return s / 0x100000000;
  };
}

// Cruciform branch has a closed-form thin-walled inverse, so the round-trip is
// exact within floating-point precision.
test('cruciform: forward(inverse(target)) reproduces (Ix, Iy, J) exactly', () => {
  const tol = 1e-9;
  const cases: Array<[number, number, number]> = [
    [1000, 1000, 100],
    [1000, 1000, 500],
    [500, 800, 200],
    [3000, 1500, 600],
    [100, 50, 10],
    [10000, 10000, 1000],
  ];
  for (const [Ix, Iy, J] of cases) {
    const s = computeCrossSection(Ix, Iy, J);
    assert.equal(s.type, 'cruciform', `expected cruciform for (${Ix}, ${Iy}, ${J}), got ${s.type}`);
    if (s.type !== 'cruciform') continue;
    const fIx = (s.t_mm * s.h_mm ** 3) / 12;
    const fIy = (s.t_mm * s.w_mm ** 3) / 12;
    const fJ = ((s.t_mm ** 3) * (s.w_mm + s.h_mm)) / 3;
    assert.ok(Math.abs(fIx - Ix) / Ix < tol, `Ix off: ${fIx} vs ${Ix}`);
    assert.ok(Math.abs(fIy - Iy) / Iy < tol, `Iy off: ${fIy} vs ${Iy}`);
    assert.ok(Math.abs(fJ - J) / J < tol, `J off: ${fJ} vs ${J}`);
  }
});

// Box branch doesn't claim exact (Ix, Iy, J) match — only that the geometry is
// well-formed for any positive input and exactly hits Ix.
test('hollowBox: always well-formed, Ix matched to scale', () => {
  const cases: Array<[number, number, number]> = [
    [1000, 1000, 1300],   // inside box reach
    [1000, 1000, 1500],   // on the boundary (asymptote in any solver)
    [1000, 1000, 1800],   // beyond — J clamped to nearest reachable
    [2000, 1000, 1200],   // asymmetric
    [500, 500, 700],
    [1000, 1000, 2000],
  ];
  for (const [Ix, Iy, J] of cases) {
    const s = computeCrossSection(Ix, Iy, J);
    assert.notEqual(s.type, 'unreachable', `unreachable for (${Ix}, ${Iy}, ${J})`);
    if (s.type !== 'hollowBox') continue;
    assert.ok(s.W_mm > 0 && Number.isFinite(s.W_mm), `W bad: ${s.W_mm}`);
    assert.ok(s.H_mm > 0 && Number.isFinite(s.H_mm), `H bad: ${s.H_mm}`);
    assert.ok(s.t_mm > 0 && Number.isFinite(s.t_mm), `t bad: ${s.t_mm}`);
    assert.ok(2 * s.t_mm < s.W_mm, `2t ≥ W for (${Ix}, ${Iy}, ${J})`);
    assert.ok(2 * s.t_mm < s.H_mm, `2t ≥ H for (${Ix}, ${Iy}, ${J})`);
    // Forward-check that Ix landed where claimed (the one quantity we pin).
    const Wi = s.W_mm - 2 * s.t_mm;
    const Hi = s.H_mm - 2 * s.t_mm;
    const fIx = (s.W_mm * s.H_mm ** 3 - Wi * Hi ** 3) / 12;
    assert.ok(
      Math.abs(fIx - Ix) / Ix < 1e-6,
      `Ix not matched for (${Ix}, ${Iy}, ${J}): got ${fIx}`,
    );
  }
});

// Fuzz: any positive input produces a finite, well-formed shape.
test('fuzz: any positive (Ix, Iy, J) gives a well-formed shape', () => {
  const rng = makeRng(42);
  const N = 4000;
  for (let i = 0; i < N; i++) {
    const Ix = Math.exp(rng() * 14 - 2);
    const Iy = Math.exp(rng() * 14 - 2);
    const J = Math.exp(rng() * 14 - 2);
    const s = computeCrossSection(Ix, Iy, J);
    if (s.type === 'cruciform') {
      assert.ok(s.w_mm > 0 && Number.isFinite(s.w_mm), `crucif w bad for ${Ix}/${Iy}/${J}`);
      assert.ok(s.h_mm > 0 && Number.isFinite(s.h_mm), `crucif h bad for ${Ix}/${Iy}/${J}`);
      assert.ok(s.t_mm > 0 && Number.isFinite(s.t_mm), `crucif t bad for ${Ix}/${Iy}/${J}`);
    } else if (s.type === 'hollowBox') {
      assert.ok(s.W_mm > 0 && Number.isFinite(s.W_mm), `box W bad for ${Ix}/${Iy}/${J}`);
      assert.ok(s.H_mm > 0 && Number.isFinite(s.H_mm), `box H bad for ${Ix}/${Iy}/${J}`);
      assert.ok(s.t_mm > 0 && Number.isFinite(s.t_mm), `box t bad for ${Ix}/${Iy}/${J}`);
      assert.ok(2 * s.t_mm < s.W_mm, `2t≥W for ${Ix}/${Iy}/${J}: W=${s.W_mm}, t=${s.t_mm}`);
      assert.ok(2 * s.t_mm < s.H_mm, `2t≥H for ${Ix}/${Iy}/${J}: H=${s.H_mm}, t=${s.t_mm}`);
    }
  }
});
