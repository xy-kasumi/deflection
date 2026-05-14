import { Editor } from './editor';
import { Scene } from './scene';
import { parse, type BeamDef } from './dsl/parse';
import { semcheck } from './dsl/semcheck';
import { walk } from './walker';
import { buildLoadSystem, simErrorToDiagnostic } from './loadsystem';
import { simulate, type SimResult } from './sim/simulate';
import type { Vec3 } from './sim/problem';
import { renderBreakdown, type DisplayMode, type HoverKey } from './breakdown';

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
const modeToggleEl = document.getElementById('mode-toggle') as HTMLElement;

let currentScale = 1;
let lastSrc = INITIAL_SRC;
let cursorOffset = 0;
let editorFocused = false;
let selectedKey: { beamIx: number; offset_mm: number } | null = null;
let lastSim: SimResult | null = null;
let mode: DisplayMode = 'realistic';
// Hovered breakdown component (Simple mode only) — swaps the selected node's
// lobe to that component's isolated δ.
let hovered: HoverKey | null = null;
// User-picked direction for the selected node (Realistic mode) — null means
// decompose at the auto worst-case d*. Reset whenever the selection changes.
let selectedDir: Vec3 | null = null;

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
  const recommended = scene.recommendDisplayScale(n.deflection.max().value);
  if (recommended !== currentScale) {
    currentScale = recommended;
    updateScaleButtons();
    scene.setDisplayScale(recommended);
  }
  redraw();
});

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
  hovered = null;
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
// Used when only display state changed (selection, mode).
function redraw() {
  if (!last) return;
  redrawScene();
  const { ls, out } = last;
  if (out.kind === 'error') {
    renderBreakdown(
      infoEl, null, ls.problem.loads, ls.loadProvenance, -1, -1, lastSrc, mode, onHover, selectedDir,
    );
  } else {
    const selectedNodeIx = resolveSelectedNodeIx(out, ls.tipQueryIx);
    renderBreakdown(
      infoEl, out, ls.problem.loads, ls.loadProvenance, selectedNodeIx, ls.tipQueryIx, lastSrc, mode,
      onHover, selectedDir,
    );
  }
  updateScaleButtons();
}

// Just the 3D scene. The hover path uses this — it leaves the breakdown pane
// in place and only swaps the selected node's lobe.
function redrawScene() {
  if (!last) return;
  const { beams, ls, out, currentBeamIx } = last;
  if (out.kind === 'error') {
    scene.update(beams, undefined, ls.supportKind, { currentBeamIx, focused: editorFocused }, -1, mode, null);
  } else {
    const selectedNodeIx = resolveSelectedNodeIx(out, ls.tipQueryIx);
    scene.update(
      beams, out, ls.supportKind, { currentBeamIx, focused: editorFocused }, selectedNodeIx, mode, hovered,
    );
  }
}

// Hover handler passed to renderBreakdown. Dedups (pointermove fires often),
// then refreshes only the scene — the breakdown pane stays put.
function onHover(h: HoverKey | null) {
  if (sameHover(h, hovered)) return;
  hovered = h;
  redrawScene();
}

function sameHover(a: HoverKey | null, b: HoverKey | null): boolean {
  if (a === b) return true;
  if (!a || !b) return false;
  return a.beamIx === b.beamIx && a.mode === b.mode;
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

function updateModeButtons() {
  for (const btn of modeToggleEl.querySelectorAll('button')) {
    btn.classList.toggle('active', btn.dataset['mode'] === mode);
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

modeToggleEl.addEventListener('click', (e) => {
  const btn = (e.target as HTMLElement).closest('button');
  if (!btn) return;
  const m = btn.dataset['mode'] as DisplayMode;
  if (m === mode) return;
  mode = m;
  hovered = null;
  updateModeButtons();
  redraw();
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

updateModeButtons();
render();

// Console hook: window.dbg.{scene, sim, scale, ...}. Live objects (not a
// snapshot) so devtools can drill into THREE internals via reflection.
(window as unknown as { dbg: unknown }).dbg = {
  get scene() { return scene; },
  get sim()   { return lastSim; },
  get scale() { return currentScale; },
  get mode()  { return mode; },
  get hovered() { return hovered; },
  get selectedKey() { return selectedKey; },
};
