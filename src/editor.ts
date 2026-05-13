import { EditorView, keymap } from '@codemirror/view';
import { EditorState } from '@codemirror/state';
import { defaultKeymap, history, historyKeymap } from '@codemirror/commands';
import { setDiagnostics, type Diagnostic as CMDiagnostic } from '@codemirror/lint';
import type { Diagnostic } from './dsl/diagnostics';

// Debounced doc-change notifier so the parse/render pipeline doesn't run on
// every keystroke.
function debounce<T extends () => void>(fn: T, ms: number): () => void {
  let h: ReturnType<typeof setTimeout> | null = null;
  return () => {
    if (h) clearTimeout(h);
    h = setTimeout(fn, ms);
  };
}

export class Editor {
  private view: EditorView;

  constructor(
    parent: HTMLElement,
    initialSrc: string,
    onChange: (src: string) => void,
  ) {
    const fire = debounce(() => onChange(this.view.state.doc.toString()), 150);

    this.view = new EditorView({
      parent,
      state: EditorState.create({
        doc: initialSrc,
        extensions: [
          history(),
          keymap.of([...defaultKeymap, ...historyKeymap]),
          // setDiagnostics() lazily installs the lint state when first called,
          // so no upfront linter() extension is needed.
          EditorView.updateListener.of((u) => {
            if (u.docChanged) fire();
          }),
        ],
      }),
    });
  }

  setDiagnostics(diagnostics: Diagnostic[]): void {
    const cmDiags: CMDiagnostic[] = diagnostics.map((d) => ({
      from: d.span.start,
      to: d.span.end,
      severity: d.severity,
      message: d.message,
    }));
    this.view.dispatch(setDiagnostics(this.view.state, cmDiags));
  }

  getDoc(): string {
    return this.view.state.doc.toString();
  }
}
