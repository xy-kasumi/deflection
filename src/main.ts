import { Editor } from './editor';
import { Scene } from './scene';
import { parse, type BeamDef } from './dsl/parse';
import { semcheck } from './dsl/semcheck';
import { walk } from './walker';
import { buildLoadSystem, simErrorToDiagnostic } from './loadsystem';
import { simulate, type SimResult } from './sim/simulate';
import { renderBreakdown, type DecompFormat, type HoverHandlers } from './breakdown';
import type { Mode } from './sim/compliance';
import type { HoverSegment } from './scene/lobe';

const INITIAL_SRC = `support(single)
mass_accel(2G)
horz beam(steel rect(W10 H10) L300)
mid: right beam(aluminum rect(W10 H10) L150)
down beam(plastic rect(W10 H10) L100) end:load(1kgf)
`;

const editorEl = document.getElementById('editor') as HTMLElement;
const infoEl = document.getElementById('info') as HTMLElement;
const canvas = document.getElementById('canvas') as HTMLCanvasElement;
const scaleOverlayEl = document.getElementById('scale-overlay') as HTMLElement;

let currentScale = 1;
let lastSrc = INITIAL_SRC;
let cursorOffset = 0;
let editorFocused = false;
let selectedKey: { beamIx: number; offset_mm: number } | null = null;
let lastSim: SimResult | null = null;
let decompFormat: DecompFormat = 'pct';
// Cursor over breakdown cell(s) — one (b, m) for a sub-mode cell, five for a
// per-beam total. Drives the scene's hover overlay; null when cursor is off.
let hoverKeys: { beamIx: number; mode: Mode }[] | null = null;

// Last full render's parse/sim outputs — redraw() draws from this without
// re-parsing. render() refreshes it; redraw()/redrawScene() consume it.
let last:
  | {
      beams: ReturnType<typeof walk>['beams'];
      ls: ReturnType<typeof buildLoadSystem>;
      out: ReturnType<typeof simulate>;
      currentBeamIx: number | null;
    }
  | null = null;

const scene = new Scene(canvas, (nodeIx) => {
  const n = lastSim?.queryResults[nodeIx];
  if (!n) return;
  selectedKey = { beamIx: n.query.beamIx, offset_mm: n.query.offset_mm };
  // Auto-pick the biggest non-overflown δ-exag for this node so a click is
  // also a "show me this node clearly" gesture. Initial tip selection stays
  // at ×1 (this callback only fires on user picks, not on default-select).
  const recommended = scene.recommendDisplayScale(n.deflection_mm.max().value);
  if (recommended !== currentScale) {
    currentScale = recommended;
    updateScaleButtons();
    scene.setDisplayScale(recommended);
  }
  redraw();
});

const hoverHandlers: HoverHandlers = {
  enter(keys) {
    hoverKeys = keys;
    scene.setHoverSegment(computeHoverSegments());
  },
  leave() {
    hoverKeys = null;
    scene.setHoverSegment(null);
  },
};

// Resolve the hovered (beam, mode) keys to concrete segments at *every* query
// node in the chain. Each sub-mode is rank-1, so each (query, beam, mode)
// gives a single direction + magnitude; pessimistic decomposition has no
// shared loads / shared direction across queries, so we emit them all.
function computeHoverSegments(): HoverSegment[] | null {
  if (!hoverKeys || !lastSim) return null;
  const segs: HoverSegment[] = [];
  for (const node of lastSim.queryResults) {
    for (const key of hoverKeys) {
      const bd = node.beamDeflections.find((b) => b.beamIx === key.beamIx);
      if (!bd) continue;
      const bm = bd.byMode.find((m) => m.mode === key.mode);
      if (!bm) continue;
      const { dir_unit, value } = bm.deflection_mm.max();
      if (value === 0) continue;
      segs.push({
        origin_mm: node.pos_mm,
        vector_mm: [dir_unit[0] * value, dir_unit[1] * value, dir_unit[2] * value],
      });
    }
  }
  return segs.length ? segs : null;
}

function resolveSelectedNodeIx(sim: SimResult, tipQueryIx: number): number {
  if (selectedKey) {
    const ix = sim.queryResults.findIndex(
      (n) =>
        n.query.beamIx === selectedKey!.beamIx && n.query.offset_mm === selectedKey!.offset_mm,
    );
    if (ix >= 0) return ix;
  }
  return tipQueryIx;
}

// Full pass: parse → simulate. Caches the result in `last`, then draws.
function render() {
  const { structure, diagnostics: pd } = parse(lastSrc);
  const sd = semcheck(structure);
  const { beams, diagnostics: wd } = walk(structure);
  const ls = buildLoadSystem(structure, beams);
  const out = simulate(ls.problem);
  const baseDiags = [...pd, ...sd, ...wd, ...ls.diagnostics];

  const currentBeamIx = editorFocused
    ? findBeamAtOffset(structure.beams, cursorOffset)
    : null;

  if (out.kind === 'error') {
    lastSim = null;
    editor.setDiagnostics([...baseDiags, simErrorToDiagnostic(out, ls)]);
  } else {
    lastSim = out;
    editor.setDiagnostics(baseDiags);
  }

  last = { beams, ls, out, currentBeamIx };
  redraw();
}

// Redraw scene + breakdown from the last render — no re-parse / re-simulate.
function redraw() {
  redrawScene();
  redrawBreakdown();
}

function redrawBreakdown() {
  if (!last) return;
  // The breakdown DOM is wiped & replaced below; old cells' mouseleave won't
  // fire on detached elements. Reset hover ourselves; mouseenter on new cells
  // re-establishes it.
  hoverKeys = null;
  scene.setHoverSegment(null);
  const { ls, out } = last;
  if (out.kind === 'error') {
    renderBreakdown(
      infoEl, null, ls.problem.loads, ls.loadProvenance, -1, -1, lastSrc,
      decompFormat, onFormatChange,
    );
  } else {
    const selectedNodeIx = resolveSelectedNodeIx(out, ls.tipQueryIx);
    renderBreakdown(
      infoEl, out, ls.problem.loads, ls.loadProvenance, selectedNodeIx, ls.tipQueryIx, lastSrc,
      decompFormat, onFormatChange, hoverHandlers,
    );
  }
  updateScaleButtons();
}

function onFormatChange(next: DecompFormat) {
  if (next === decompFormat) return;
  decompFormat = next;
  redrawBreakdown();
}

function redrawScene() {
  if (!last) return;
  const { beams, ls, out, currentBeamIx } = last;
  if (out.kind === 'error') {
    scene.update(beams, undefined, ls.supportKind, { currentBeamIx, focused: editorFocused }, -1);
  } else {
    const selectedNodeIx = resolveSelectedNodeIx(out, ls.tipQueryIx);
    scene.update(
      beams, out, ls.supportKind, { currentBeamIx, focused: editorFocused }, selectedNodeIx,
    );
  }
}

function findBeamAtOffset(defs: BeamDef[], offset: number): number | null {
  for (let i = 0; i < defs.length; i++) {
    const { start, end } = defs[i]!.span;
    if (offset >= start && offset <= end) return i;
  }
  return null;
}

function updateScaleButtons() {
  for (const btn of scaleOverlayEl.querySelectorAll('button')) {
    const s = Number(btn.dataset['scale']);
    btn.classList.toggle('active', s === currentScale);
  }
}

scaleOverlayEl.addEventListener('click', (e) => {
  const btn = (e.target as HTMLElement).closest('button');
  if (!btn) return;
  const s = Number(btn.dataset['scale']);
  if (s === currentScale) return;
  currentScale = s;
  updateScaleButtons();
  scene.setDisplayScale(s);
});

const editor = new Editor(
  editorEl,
  INITIAL_SRC,
  (src) => {
    lastSrc = src;
    render();
  },
  ({ offset, focused }) => {
    cursorOffset = offset;
    editorFocused = focused;
    render();
  },
);

render();

// Console hook: window.dbg.{scene, sim, scale, ...}. Live objects (not a
// snapshot) so devtools can drill into THREE internals via reflection.
(window as unknown as { dbg: unknown }).dbg = {
  get scene() { return scene; },
  get sim()   { return lastSim; },
  get scale() { return currentScale; },
  get selectedKey() { return selectedKey; },
  get hoverKeys() { return hoverKeys; },
  get decompFormat() { return decompFormat; },
};
