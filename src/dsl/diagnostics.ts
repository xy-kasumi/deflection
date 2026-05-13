export interface Span {
  start: number;
  end: number;
}

export interface Diagnostic {
  severity: 'error' | 'warning';
  message: string;
  span: Span;
}
