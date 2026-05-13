import type { Diagnostic, Span } from './diagnostics';

export type TokenKind =
  | 'LP'
  | 'RP'
  | 'LOC_SEP'
  | 'KW_DIR'
  | 'KW_BEAM'
  | 'KW_LOC'
  | 'IDENT'
  | 'QUANTITY';

export type Dir = 'horz' | 'right' | 'left' | 'up' | 'down';
export type LocKw = 'end' | 'mid';

export interface Token {
  kind: TokenKind;
  text: string;
  span: Span;
  // QUANTITY:
  prefix?: string;
  value?: number;
  unit?: string;
  // KW_DIR:
  dir?: Dir;
  // KW_LOC:
  locKw?: LocKw;
  // IDENT / KW_BEAM:
  name?: string;
}

const DIRS: Record<string, Dir> = {
  horz: 'horz',
  right: 'right',
  left: 'left',
  up: 'up',
  down: 'down',
};

const LOC_KWS: Record<string, LocKw> = {
  end: 'end',
  mid: 'mid',
};

const isDigit = (c: string) => c >= '0' && c <= '9';
const isLower = (c: string) => c >= 'a' && c <= 'z';
const isUpper = (c: string) => c >= 'A' && c <= 'Z';
const isLetter = (c: string) => isLower(c) || isUpper(c);
const isUnitChar = (c: string) => isLetter(c) || isDigit(c) || c === '/';

// Lex one logical line. `baseOffset` is the byte offset of the start of this
// line in the full source, so all token spans are absolute.
export function lex(line: string, baseOffset: number): {
  tokens: Token[];
  diagnostics: Diagnostic[];
} {
  const tokens: Token[] = [];
  const diagnostics: Diagnostic[] = [];
  let i = 0;

  while (i < line.length) {
    const c = line[i] as string;

    // Whitespace inside a line: skip.
    if (c === ' ' || c === '\t' || c === '\r') {
      i++;
      continue;
    }

    // Single-char tokens.
    if (c === '(' || c === ')' || c === ':') {
      const kind: TokenKind = c === '(' ? 'LP' : c === ')' ? 'RP' : 'LOC_SEP';
      tokens.push({
        kind,
        text: c,
        span: { start: baseOffset + i, end: baseOffset + i + 1 },
      });
      i++;
      continue;
    }

    // Lowercase word: IDENT or KW_DIR / KW_BEAM / KW_LOC. Idents may contain
    // '_' (e.g. `mass_accel`); keywords don't.
    if (isLower(c) || c === '_') {
      const start = i;
      while (i < line.length && (isLower(line[i] as string) || line[i] === '_')) i++;
      const text = line.slice(start, i);
      const span: Span = { start: baseOffset + start, end: baseOffset + i };
      if (text === 'beam') {
        tokens.push({ kind: 'KW_BEAM', text, span, name: text });
      } else if (DIRS[text]) {
        tokens.push({ kind: 'KW_DIR', text, span, dir: DIRS[text] });
      } else if (LOC_KWS[text]) {
        tokens.push({ kind: 'KW_LOC', text, span, locKw: LOC_KWS[text] });
      } else {
        tokens.push({ kind: 'IDENT', text, span, name: text });
      }
      continue;
    }

    // QUANTITY:
    //   [ PREFIX ] NUMBER [ UNIT ]
    //   PREFIX = /[A-Z][A-Za-z]*/
    //   NUMBER = digits ('.' digits)? ([eE][+-]? digits)?
    //   UNIT   = letter (letter | digit | '/')*    (cannot start with digit)
    if (isUpper(c) || isDigit(c)) {
      const start = i;

      // Prefix (optional, only if starts with uppercase).
      let prefix: string | undefined;
      if (isUpper(c)) {
        const pStart = i;
        while (i < line.length && isLetter(line[i] as string)) i++;
        prefix = line.slice(pStart, i);
        if (i >= line.length || !isDigit(line[i] as string)) {
          // Prefix not followed by a digit — not a quantity. Emit error.
          diagnostics.push({
            severity: 'error',
            message: `expected number after prefix '${prefix}'`,
            span: { start: baseOffset + pStart, end: baseOffset + i },
          });
          continue;
        }
      }

      // Number: digits ('.' digits)? ([eE] [+-]? digits)?
      const nStart = i;
      while (i < line.length && isDigit(line[i] as string)) i++;
      if (i < line.length && line[i] === '.') {
        i++;
        while (i < line.length && isDigit(line[i] as string)) i++;
      }
      if (i < line.length && (line[i] === 'e' || line[i] === 'E')) {
        const save = i;
        i++;
        if (i < line.length && (line[i] === '+' || line[i] === '-')) i++;
        if (i < line.length && isDigit(line[i] as string)) {
          while (i < line.length && isDigit(line[i] as string)) i++;
        } else {
          // No digits after 'e' — back up; the 'e' starts a unit instead.
          i = save;
        }
      }
      const numText = line.slice(nStart, i);
      const value = Number(numText);
      if (!Number.isFinite(value)) {
        diagnostics.push({
          severity: 'error',
          message: `invalid number '${numText}'`,
          span: { start: baseOffset + nStart, end: baseOffset + i },
        });
        continue;
      }

      // Unit (optional). Must start with a letter; can include digits and '/'.
      let unit: string | undefined;
      if (i < line.length && isLetter(line[i] as string)) {
        const uStart = i;
        while (i < line.length && isUnitChar(line[i] as string)) i++;
        unit = line.slice(uStart, i);
      }

      const text = line.slice(start, i);
      tokens.push({
        kind: 'QUANTITY',
        text,
        span: { start: baseOffset + start, end: baseOffset + i },
        prefix,
        value,
        unit,
      });
      continue;
    }

    // Anything else: unrecognized.
    const errStart = i;
    while (
      i < line.length &&
      line[i] !== ' ' && line[i] !== '\t' && line[i] !== '\r' &&
      line[i] !== '(' && line[i] !== ')' && line[i] !== ':'
    ) {
      i++;
    }
    diagnostics.push({
      severity: 'error',
      message: `unexpected '${line.slice(errStart, i)}'`,
      span: { start: baseOffset + errStart, end: baseOffset + i },
    });
  }

  return { tokens, diagnostics };
}

// Split source into lines with their byte offsets. Preserves trailing empty
// line iff the source ends with a newline (so spans line up).
export function splitLines(src: string): { text: string; start: number }[] {
  const out: { text: string; start: number }[] = [];
  let start = 0;
  for (let i = 0; i <= src.length; i++) {
    if (i === src.length || src[i] === '\n') {
      out.push({ text: src.slice(start, i), start });
      start = i + 1;
    }
  }
  return out;
}
