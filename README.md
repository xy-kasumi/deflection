# deflection

WIP

In-browser calculator for back-of-the-envelope deflection calculation
of serial chain of beams.

## Build / Tests

```bash
npm install
npm run typecheck                     # tsc --noEmit
npx tsx scripts/smoke-compliance.ts   # sim vs known closed formula
npm run dev                           # vite dev server
npm run build                         # static bundle in dist/
```

## Scope (what the tool does *not* model)

Slender Euler-Bernoulli beams, bending + torsion only.
Axial extension or buckling is not modeled.

Means: No thin wire pulling, No ultra-fat (compared to length) beams

## Development

See [`DSL.md`](DSL.md), [`vocab.md`](vocab.md).

`window.dbg` exposes live `scene` / `sim` / `scale` / `selectedKey` for devtools-console inspection.
