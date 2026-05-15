// Shared assertion primitives for anchor + segment smoke files.
//
// One module-level counter per process. Each `.test.ts` script imports the
// helpers it needs, runs its checks, then calls `finish(label)` once at the
// end to print the summary and set the exit code.

let total = 0;
let failed = 0;

/** Boolean predicate. */
export function expect(name: string, cond: boolean): void {
  total++;
  if (!cond) failed++;
  console.log(`${cond ? 'ok  ' : 'FAIL'} ${name}`);
}

/** Relative-tolerance float check; `expected` may be ~0 thanks to the +1e-30 guard. */
export function check(name: string, actual: number, expected: number, tolRel = 1e-12): void {
  const rel = Math.abs(actual - expected) / (Math.abs(expected) + 1e-30);
  const ok = rel < tolRel;
  total++;
  if (!ok) failed++;
  console.log(
    `${ok ? 'ok  ' : 'FAIL'} ${name}: got ${actual.toExponential(6)}, ` +
      `want ${expected.toExponential(6)} (rel ${rel.toExponential(2)})`,
  );
}

/** Absolute-tolerance check around zero. */
export function checkNear0(name: string, actual: number, tolAbs = 1e-9): void {
  const ok = Math.abs(actual) < tolAbs;
  total++;
  if (!ok) failed++;
  console.log(`${ok ? 'ok  ' : 'FAIL'} ${name}: got ${actual.toExponential(6)} (want ~0)`);
}

/** Integer equality. */
export function eqInt(name: string, actual: number, want: number): void {
  expect(`${name}: got ${actual}, want ${want}`, actual === want);
}

/** Absolute-tolerance scalar check (default 1e-9). */
export function near(name: string, actual: number, want: number, eps = 1e-9): void {
  expect(`${name}: got ${actual}, want ${want}`, Math.abs(actual - want) < eps);
}

/** Print summary, set process.exitCode. Call once at end of a test file. */
export function finish(label: string): void {
  const ok = failed === 0;
  console.log(`\n${ok ? `${label} ok (${total} checks)` : `${label} FAILED: ${failed} of ${total}`}`);
  process.exitCode = ok ? 0 : 1;
}
