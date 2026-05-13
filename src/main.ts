import { Editor } from './editor';
import { Scene } from './scene';
import { parse } from './dsl/parse';
import { semcheck } from './dsl/semcheck';
import { walk } from './walker';
import { runSim, type DisplayScale } from './sim/run';
import { renderReadout } from './readout';

const INITIAL_SRC = `support(single)
horz beam(steel rect(W10 H10) L300)
mid: right beam(aluminum rect(W10 H10) L150)
down beam(plastic rect(W10 H10) L100) end:force(1kgf)
`;

const editorEl = document.getElementById('editor') as HTMLElement;
const infoEl = document.getElementById('info') as HTMLElement;
const canvas = document.getElementById('canvas') as HTMLCanvasElement;
const scaleOverlayEl = document.getElementById('scale-overlay') as HTMLElement;

const scene = new Scene(canvas);

let currentScale: DisplayScale = 1;
let lastSrc = INITIAL_SRC;

function render(src: string) {
  lastSrc = src;
  const { structure, diagnostics: pd } = parse(src);
  const sd = semcheck(structure);
  const { beams, diagnostics: wd } = walk(structure);
  const sim = runSim(beams, structure);
  sim.display_scale = currentScale;
  editor.setDiagnostics([...pd, ...sd, ...wd, ...sim.diagnostics]);
  scene.update(beams, sim);
  renderReadout(infoEl, sim);
  updateScaleButtons();
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
  const s = Number(btn.dataset['scale']) as DisplayScale;
  if (s === currentScale) return;
  currentScale = s;
  render(lastSrc);
});

const editor = new Editor(editorEl, INITIAL_SRC, render);

render(INITIAL_SRC);
