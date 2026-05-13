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

## Development

See [`DSL.md`](DSL.md), [`vocab.md`](vocab.md).
