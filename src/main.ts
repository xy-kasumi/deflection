import { Editor } from './editor';
import { Scene } from './scene';
import { parse, type BeamDef } from './dsl/parse';
import { semcheck } from './dsl/semcheck';
import { walk } from './walker';
import { runSim, type SimResult } from './sim/run';
import { getSupportKind } from './sim/compliance';
import { renderReadout } from './readout';

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
  const n = lastSim?.nodes[nodeIx];
  if (!n) return;
  selectedKey = { beamIx: n.beamIx, offset_mm: n.offset_mm };
  render();
});

function resolveSelectedNodeIx(sim: SimResult): number {
  if (selectedKey) {
    const ix = sim.nodes.findIndex(
      (n) => n.beamIx === selectedKey!.beamIx && n.offset_mm === selectedKey!.offset_mm,
    );
    if (ix >= 0) return ix;
  }
  return sim.tipNodeIx;
}

function render() {
  const { structure, diagnostics: pd } = parse(lastSrc);
  const sd = semcheck(structure);
  const { beams, diagnostics: wd } = walk(structure);
  const sim = runSim(beams, structure);
  lastSim = sim;
  editor.setDiagnostics([...pd, ...sd, ...wd, ...sim.diagnostics]);

  const currentBeamIx = editorFocused
    ? findBeamAtOffset(structure.beams, cursorOffset)
    : null;
  const selectedNodeIx = resolveSelectedNodeIx(sim);
  scene.update(
    beams,
    sim,
    getSupportKind(structure),
    { currentBeamIx, focused: editorFocused },
    selectedNodeIx,
  );
  renderReadout(infoEl, sim, selectedNodeIx);
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
