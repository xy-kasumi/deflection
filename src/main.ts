import { Editor } from './editor';
import { Scene } from './scene';
import { parse } from './dsl/parse';
import { semcheck } from './dsl/semcheck';
import { walk } from './walker';

const INITIAL_SRC = `support(both)
horz beam(aluminum rect(W30 H20 T2) L100) mid:force(0.5) end:force(0.5kgf)
right beam(plastic moment(Ix1000 Iy1e6 J1000) L200) mid:force(0.3)
up beam(steel rect(W7 H5) L50) force(0.2)
`;

const editorEl = document.getElementById('editor') as HTMLElement;
const canvas = document.getElementById('canvas') as HTMLCanvasElement;

const scene = new Scene(canvas);

function render(src: string) {
  const { structure, diagnostics: pd } = parse(src);
  const sd = semcheck(structure);
  const { beams, diagnostics: wd } = walk(structure);
  editor.setDiagnostics([...pd, ...sd, ...wd]);
  scene.update(beams);
}

const editor = new Editor(editorEl, INITIAL_SRC, render);

render(INITIAL_SRC);
