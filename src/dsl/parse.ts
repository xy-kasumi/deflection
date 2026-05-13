import type { Diagnostic, Span } from './diagnostics';
import { lex, splitLines, type Dir, type LocKw, type Token } from './lex';

// ---------- AST ----------

export interface Quantity {
  prefix?: string;
  value: number;
  unit?: string;
  span: Span;
}

export type LocSpec =
  | { kind: 'kw'; name: LocKw; span: Span }
  | { kind: 'quantity'; quantity: Quantity; span: Span };

export type Param =
  | { kind: 'quantity'; quantity: Quantity; span: Span }
  | { kind: 'ident'; name: string; params?: Param[]; span: Span };

export interface Attachment {
  loc?: LocSpec;
  name: string;
  params: Param[];
  span: Span;
}

export interface BeamDef {
  loc?: LocSpec;
  dir: Dir;
  params: Param[];
  attachments: Attachment[];
  span: Span;
}

export interface EnvDef {
  name: string;
  params: Param[];
  span: Span;
}

export interface Structure {
  envs: EnvDef[];
  beams: BeamDef[];
}

export interface ParseResult {
  structure: Structure;
  diagnostics: Diagnostic[];
}

// ---------- Parser ----------

class Parser {
  private tokens: Token[];
  private pos = 0;
  diagnostics: Diagnostic[] = [];

  constructor(tokens: Token[]) {
    this.tokens = tokens;
  }

  peek(offset = 0): Token | undefined {
    return this.tokens[this.pos + offset];
  }

  private consume(): Token | undefined {
    return this.tokens[this.pos++];
  }

  private fail(message: string, span: Span): null {
    this.diagnostics.push({ severity: 'error', message, span });
    return null;
  }

  parseDef(): EnvDef | BeamDef | null {
    const t0 = this.peek();
    if (!t0) return null;

    // beam-def: KW_DIR, or (KW_LOC | QUANTITY) followed by LOC_SEP.
    if (t0.kind === 'KW_DIR') return this.parseBeamDef();
    if (
      (t0.kind === 'KW_LOC' || t0.kind === 'QUANTITY') &&
      this.peek(1)?.kind === 'LOC_SEP'
    ) {
      return this.parseBeamDef();
    }

    // env-def: IDENT paramlist.
    if (t0.kind === 'IDENT') return this.parseEnvDef();

    return this.fail(`unexpected '${t0.text}'`, t0.span);
  }

  private parseLocSpec(): LocSpec | null {
    const t = this.peek();
    if (!t) return null;
    let loc: LocSpec;
    if (t.kind === 'KW_LOC') {
      this.consume();
      loc = { kind: 'kw', name: t.locKw as LocKw, span: t.span };
    } else if (t.kind === 'QUANTITY') {
      this.consume();
      loc = {
        kind: 'quantity',
        quantity: { prefix: t.prefix, value: t.value as number, unit: t.unit, span: t.span },
        span: t.span,
      };
    } else {
      return null;
    }
    const sep = this.peek();
    if (!sep || sep.kind !== 'LOC_SEP') {
      return this.fail("expected ':' after location", t.span);
    }
    this.consume();
    return loc;
  }

  private parseBeamDef(): BeamDef | null {
    let loc: LocSpec | undefined;
    const t0 = this.peek();
    if (
      t0 &&
      (t0.kind === 'KW_LOC' || t0.kind === 'QUANTITY') &&
      this.peek(1)?.kind === 'LOC_SEP'
    ) {
      const ls = this.parseLocSpec();
      if (!ls) return null;
      loc = ls;
    }

    const dirTok = this.consume();
    if (!dirTok || dirTok.kind !== 'KW_DIR') {
      return this.fail(
        'expected direction (horz, left, right, up, or down)',
        dirTok?.span ?? loc?.span ?? { start: 0, end: 0 },
      );
    }

    const beamTok = this.consume();
    if (!beamTok || beamTok.kind !== 'KW_BEAM') {
      return this.fail("expected 'beam'", beamTok?.span ?? dirTok.span);
    }

    const params = this.parseParamList();
    if (!params) return null;

    const attachments: Attachment[] = [];
    while (this.peek()) {
      const a = this.parseAttachment();
      if (!a) return null;
      attachments.push(a);
    }

    const startSpan = loc?.span ?? dirTok.span;
    const endSpan =
      attachments.length > 0 ? attachments[attachments.length - 1]!.span : params.span;

    return {
      loc,
      dir: dirTok.dir as Dir,
      params: params.params,
      attachments,
      span: { start: startSpan.start, end: endSpan.end },
    };
  }

  private parseAttachment(): Attachment | null {
    let loc: LocSpec | undefined;
    const t0 = this.peek();
    if (
      t0 &&
      (t0.kind === 'KW_LOC' || t0.kind === 'QUANTITY') &&
      this.peek(1)?.kind === 'LOC_SEP'
    ) {
      const ls = this.parseLocSpec();
      if (!ls) return null;
      loc = ls;
    }

    const nameTok = this.consume();
    if (!nameTok || nameTok.kind !== 'IDENT') {
      return this.fail(
        'expected attachment name (identifier)',
        nameTok?.span ?? loc?.span ?? { start: 0, end: 0 },
      );
    }

    const params = this.parseParamList();
    if (!params) return null;

    const start = loc?.span.start ?? nameTok.span.start;
    return {
      loc,
      name: nameTok.name as string,
      params: params.params,
      span: { start, end: params.span.end },
    };
  }

  private parseEnvDef(): EnvDef | null {
    const nameTok = this.consume();
    if (!nameTok || nameTok.kind !== 'IDENT') {
      return this.fail('expected identifier', nameTok?.span ?? { start: 0, end: 0 });
    }
    const params = this.parseParamList();
    if (!params) return null;
    return {
      name: nameTok.name as string,
      params: params.params,
      span: { start: nameTok.span.start, end: params.span.end },
    };
  }

  private parseParamList(): { params: Param[]; span: Span } | null {
    const lp = this.consume();
    if (!lp || lp.kind !== 'LP') {
      return this.fail("expected '('", lp?.span ?? { start: 0, end: 0 });
    }
    const start = lp.span.start;
    const params: Param[] = [];
    while (true) {
      const t = this.peek();
      if (!t) {
        return this.fail("expected ')' before end of line", lp.span);
      }
      if (t.kind === 'RP') {
        const rp = this.consume() as Token;
        return { params, span: { start, end: rp.span.end } };
      }
      const p = this.parseParam();
      if (!p) return null;
      params.push(p);
    }
  }

  private parseParam(): Param | null {
    const t = this.peek();
    if (!t) return null;

    if (t.kind === 'QUANTITY') {
      this.consume();
      return {
        kind: 'quantity',
        quantity: { prefix: t.prefix, value: t.value as number, unit: t.unit, span: t.span },
        span: t.span,
      };
    }

    if (t.kind === 'IDENT') {
      this.consume();
      const next = this.peek();
      if (next?.kind === 'LP') {
        const inner = this.parseParamList();
        if (!inner) return null;
        return {
          kind: 'ident',
          name: t.name as string,
          params: inner.params,
          span: { start: t.span.start, end: inner.span.end },
        };
      }
      return { kind: 'ident', name: t.name as string, span: t.span };
    }

    return this.fail(`unexpected '${t.text}' in arguments`, t.span);
  }
}

// ---------- Top-level ----------

export function parse(src: string): ParseResult {
  const envs: EnvDef[] = [];
  const beams: BeamDef[] = [];
  const diagnostics: Diagnostic[] = [];

  for (const line of splitLines(src)) {
    const { tokens, diagnostics: lexDiags } = lex(line.text, line.start);
    diagnostics.push(...lexDiags);
    if (tokens.length === 0) continue;

    const p = new Parser(tokens);
    const def = p.parseDef();
    diagnostics.push(...p.diagnostics);

    if (def === null) continue;

    const leftover = p.peek();
    if (leftover) {
      diagnostics.push({
        severity: 'error',
        message: `unexpected '${leftover.text}' at end of line`,
        span: leftover.span,
      });
      continue;
    }

    if ('dir' in def) beams.push(def);
    else envs.push(def);
  }

  return { structure: { envs, beams }, diagnostics };
}
