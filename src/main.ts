import { Editor } from './editor';
import { Scene } from './scene';
import { parse, type BeamDef } from './dsl/parse';
import { semcheck } from './dsl/semcheck';
import { walk } from './walker';
import { buildLoadSystem, simErrorToDiagnostic } from './loadsystem';
import { simulate, type SimResult } from './sim/simulate';
import { renderBreakdown } from './breakdown';

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
  render();
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
    scene.update(beams, undefined, ls.supportKind, { currentBeamIx, focused: editorFocused }, -1);
    renderBreakdown(infoEl, null, ls.loadProvenance, -1, -1, lastSrc);
    updateScaleButtons();
    return;
  }

  lastSim = out;
  editor.setDiagnostics(baseDiags);

  const selectedNodeIx = resolveSelectedNodeIx(out, ls.tipQueryIx);
  scene.update(
    beams,
    out,
    ls.supportKind,
    { currentBeamIx, focused: editorFocused },
    selectedNodeIx,
  );
  renderBreakdown(infoEl, out, ls.loadProvenance, selectedNodeIx, ls.tipQueryIx, lastSrc);
  updateScaleButtons();
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
};
