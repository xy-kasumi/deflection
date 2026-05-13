import { Editor } from './editor';
import { Scene } from './scene';
import { parse } from './dsl/parse';
import { semcheck } from './dsl/semcheck';
import { walk } from './walker';
import { runSim } from './sim/run';
import { renderReadout } from './readout';

const INITIAL_SRC = `support(single)
horz beam(steel rect(W10 H10) L300)
mid: right beam(aluminum rect(W10 H10) L150)
down beam(plastic rect(W10 H10) L100) end:force(1kgf)
`;

const editorEl = document.getElementById('editor') as HTMLElement;
const readoutEl = document.getElementById('readout') as HTMLElement;
const canvas = document.getElementById('canvas') as HTMLCanvasElement;

const scene = new Scene(canvas);

function render(src: string) {
  const { structure, diagnostics: pd } = parse(src);
  const sd = semcheck(structure);
  const { beams, diagnostics: wd } = walk(structure);
  const sim = runSim(beams, structure);
  editor.setDiagnostics([...pd, ...sd, ...wd, ...sim.diagnostics]);
  scene.update(beams, sim);
  renderReadout(readoutEl, sim);
}

const editor = new Editor(editorEl, INITIAL_SRC, render);

render(INITIAL_SRC);
