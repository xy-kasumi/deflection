import { Editor } from './editor';
import { Scene } from './scene';
import { parse } from './dsl/parse';
import { semcheck } from './dsl/semcheck';
import { walk } from './walker';

const INITIAL_SRC = `support(both)
horz beam(L300)
mid: right beam(L150)
down beam(L100)
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
