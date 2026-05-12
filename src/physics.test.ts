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
// well-formed and exactly hits Ix.
test('hollowBox: well-formed and Ix matched (J/I below thin-walled ceiling)', () => {
  const cases: Array<[number, number, number]> = [
    [1000, 1000, 1300],   // square, J/I = 1.3
    [1000, 1000, 1400],   // square, J/I = 1.4 (just below ceiling)
    [500, 500, 700],      // square, J/I = 1.4
    [1000, 500, 1000],    // asymmetric, J/I = 1.0 (just below ceiling)
  ];
  for (const [Ix, Iy, J] of cases) {
    const s = computeCrossSection(Ix, Iy, J);
    assert.equal(s.type, 'hollowBox', `expected hollowBox for (${Ix}, ${Iy}, ${J}), got ${s.type}`);
    if (s.type !== 'hollowBox') continue;
    assert.ok(s.W_mm > 0 && Number.isFinite(s.W_mm), `W bad: ${s.W_mm}`);
    assert.ok(s.H_mm > 0 && Number.isFinite(s.H_mm), `H bad: ${s.H_mm}`);
    assert.ok(s.t_mm > 0 && Number.isFinite(s.t_mm), `t bad: ${s.t_mm}`);
    assert.ok(2 * s.t_mm < s.W_mm, `2t ≥ W for (${Ix}, ${Iy}, ${J})`);
    assert.ok(2 * s.t_mm < s.H_mm, `2t ≥ H for (${Ix}, ${Iy}, ${J})`);
    const Wi = s.W_mm - 2 * s.t_mm;
    const Hi = s.H_mm - 2 * s.t_mm;
    const fIx = (s.W_mm * s.H_mm ** 3 - Wi * Hi ** 3) / 12;
    assert.ok(
      Math.abs(fIx - Ix) / Ix < 1e-6,
      `Ix not matched for (${Ix}, ${Iy}, ${J}): got ${fIx}`,
    );
  }
});

// Filled rectangle: matches Ix and Iy exactly when J is too large for a
// hollow box of the implied aspect ratio.
test('filledRect: matches Ix and Iy exactly when J/I exceeds box ceiling', () => {
  // (Ix=50, Iy=300, J=150) ≈ a 10×4 solid rectangle — sanity-check the
  // back-computed (b, h) lands there and that Ix/Iy from the rendered shape
  // matches the input.
  const cases: Array<[number, number, number, number, number]> = [
    // Ix,  Iy,  J,    expected b≈, expected h≈
    [   50,  300, 150,        10,            4],
    [ 1000, 1000, 1500,    10.47,        10.47],   // J/I = 1.5 (boundary)
    [ 1000, 1000, 2000,    10.47,        10.47],   // beyond
  ];
  for (const [Ix, Iy, J, b_expected, h_expected] of cases) {
    const s = computeCrossSection(Ix, Iy, J);
    assert.equal(s.type, 'filledRect', `expected filledRect for (${Ix}, ${Iy}, ${J}), got ${s.type}`);
    if (s.type !== 'filledRect') continue;
    assert.ok(s.b_mm > 0 && s.h_mm > 0);
    assert.ok(
      Math.abs(s.b_mm - b_expected) / b_expected < 0.05,
      `b off: ${s.b_mm} vs expected ${b_expected}`,
    );
    assert.ok(
      Math.abs(s.h_mm - h_expected) / h_expected < 0.05,
      `h off: ${s.h_mm} vs expected ${h_expected}`,
    );
    // Forward: Ix and Iy must match the input exactly (1e-9).
    const fIx = (s.b_mm * Math.pow(s.h_mm, 3)) / 12;
    const fIy = (Math.pow(s.b_mm, 3) * s.h_mm) / 12;
    assert.ok(Math.abs(fIx - Ix) / Ix < 1e-9, `Ix off: ${fIx} vs ${Ix}`);
    assert.ok(Math.abs(fIy - Iy) / Iy < 1e-9, `Iy off: ${fIy} vs ${Iy}`);
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
      // Reject "unrealistically thin walls" — those would be the bug this
      // refactor was meant to prevent.
      assert.ok(s.t_mm > 0.005 * Math.min(s.W_mm, s.H_mm),
        `box has unrealistically thin walls for ${Ix}/${Iy}/${J}: t/min=${s.t_mm/Math.min(s.W_mm,s.H_mm)}`);
    } else if (s.type === 'filledRect') {
      assert.ok(s.b_mm > 0 && Number.isFinite(s.b_mm), `rect b bad for ${Ix}/${Iy}/${J}`);
      assert.ok(s.h_mm > 0 && Number.isFinite(s.h_mm), `rect h bad for ${Ix}/${Iy}/${J}`);
    }
  }
});
