import type { SimResult, DeflectionQueryResult } from './sim/simulate';
import type { Load } from './sim/problem';
import type { Mode } from './sim/compliance';
import type { LoadProvenance } from './loadsystem';

export type DecompFormat = 'pct' | 'abs';

// Cursor over a breakdown cell. `keys` is one entry for a single (beam, mode)
// cell, or all five modes for a per-beam total cell. The scene draws one
// segment per query node per key.
export interface HoverHandlers {
  enter: (keys: Array<{ beamIx: number; mode: Mode }>) => void;
  leave: () => void;
}

// Pessimistic decomposition: Σ_{b,m} max_d δ_{b,m}(d). Each (b, m) is rank-1
// in our 5-mode scheme, so its argmax direction & magnitude are uniquely
// defined and the cell value is a literal mm number along a literal world
// direction. Direction-independent; loosely upper-bounds the realistic δ.
//
// Per-load decomposition: at each (b, m)'s own argmax d*_{b,m},
//   δ_{b,m}(d*) = Σ_p F_p · |C_{b,m,p}^T d*_{b,m}|
// load p's contribution to the bound is Σ_{b,m} F_p · |C_{b,m,p}^T d*_{b,m}|.
// All non-negative; per-load and per-(b,m) sums both equal `deltaMax` exactly.
interface BreakdownRows {
  deltaMax: number;
  perBeam: {
    beamIx: number;
    bendIxTrans: number;
    bendIxRot: number;
    bendIyTrans: number;
    bendIyRot: number;
    twist: number;
    total: number;
  }[];
  perLoad: { loadIx: number; delta_mm: number }[];
}

// Order matches column layout in the rendered grid: translation pair, then
// rotation-transport pair, then twist. Translation columns are the "own-tip
// motion" of the named beam; rotation columns are how its slope-at-tip
// propagates downstream through arm. On-beam queries have zero in the three
// rotation/twist columns (arm = 0 ⇒ no transport).
const MODE_COLUMNS: Mode[] = [
  'bendIxTrans',
  'bendIyTrans',
  'bendIxRot',
  'bendIyRot',
  'twist',
];
const MODE_HEADERS = ['bV(tran)', 'bH(tran)', 'bV(rot)', 'bH(rot)', 'tw'];

export function renderBreakdown(
  el: HTMLElement,
  sim: SimResult | null,
  loads: Load[],
  loadProvenance: LoadProvenance[],
  selectedNodeIx: number,
  tipNodeIx: number,
  src: string,
  format: DecompFormat,
  onFormatChange: (next: DecompFormat) => void,
  hover?: HoverHandlers,
): void {
  el.innerHTML = '';

  if (!sim || sim.queryResults.length === 0) {
    el.innerHTML = '<span class="bd-empty">no chain</span>';
    return;
  }

  const sel = sim.queryResults[selectedNodeIx];
  if (!sel) return;

  // Headline — realistic (`≈`) translation and rotation worst cases. These are
  // the true values; the decomposition below them is the pessimistic upper
  // bound that splits into rank-1 attributable pieces.
  const headline = document.createElement('div');
  headline.className = 'bd-headline';
  headline.textContent =
    `δ ≈ ${formatMm(sel.deflection.max().value)}` +
    `    δθ ≈ ${formatAngle(sel.rotation.max().value)}`;
  el.appendChild(headline);

  const subhead = document.createElement('div');
  subhead.className = 'bd-subhead';
  subhead.textContent = selectedNodeIx === tipNodeIx
    ? `tip (beam${sel.query.beamIx})`
    : `${nodeLoc(sel.query.offset_mm)} of beam${sel.query.beamIx}`;
  el.appendChild(subhead);

  const rows = decompose(sel);

  // Decomposition section header + pessimistic total + format toggle.
  const decompHeader = document.createElement('div');
  decompHeader.className = 'bd-section bd-section-toolbar';
  const decompTitle = document.createElement('span');
  decompTitle.textContent = `decomposition  tot ≲ ${formatMm(rows.deltaMax)}`;
  decompHeader.append(decompTitle, formatToggle(format, onFormatChange));
  el.appendChild(decompHeader);

  // Beams grid. Five sub-mode columns + total. Each sub-mode cell is one
  // (beamIx, Mode) pair; the total cell hover fires all five modes of that
  // beam at once.
  if (rows.perBeam.length > 0) {
    const grid = document.createElement('div');
    grid.className = 'bd-beams';
    grid.append(headerCell(''));
    for (const text of MODE_HEADERS) grid.append(headerCell(text));
    grid.append(headerCell('total'));

    for (const b of rows.perBeam) {
      const name = document.createElement('span');
      name.textContent = `beam${b.beamIx}`;
      grid.append(name);

      const modeValues: number[] = [
        b.bendIxTrans, b.bendIyTrans,
        b.bendIxRot,   b.bendIyRot,
        b.twist,
      ];
      modeValues.forEach((val, i) => {
        const cell = valueCell(val, rows.deltaMax, format);
        if (hover) {
          const m = MODE_COLUMNS[i]!;
          cell.classList.add('hoverable');
          cell.addEventListener('mouseenter', () => hover.enter([{ beamIx: b.beamIx, mode: m }]));
          cell.addEventListener('mouseleave', () => hover.leave());
        }
        grid.append(cell);
      });

      const totalCell = valueCell(b.total, rows.deltaMax, format, true);
      if (hover) {
        totalCell.classList.add('hoverable');
        const keys = MODE_COLUMNS.map((mode) => ({ beamIx: b.beamIx, mode }));
        totalCell.addEventListener('mouseenter', () => hover.enter(keys));
        totalCell.addEventListener('mouseleave', () => hover.leave());
      }
      grid.append(totalCell);
    }
    el.appendChild(grid);
  }

  // Loads section. Per-load contributions to the same pessimistic total. No
  // hover here — load attribution doesn't have a clean single direction in
  // the 5-mode decomposition (its contribution decomposes across (b, m)
  // again, each with its own direction).
  if (rows.perLoad.length > 0) {
    el.appendChild(sectionHeader('loads'));
    const grid = document.createElement('div');
    grid.className = 'bd-loads';
    const loadRows = [...rows.perLoad].sort((a, b) => b.delta_mm - a.delta_mm);
    for (const r of loadRows) {
      const load = loads[r.loadIx];
      const loc = loadLocEl(load, loadProvenance[r.loadIx], src);
      const force = document.createElement('span');
      force.className = 'force';
      force.textContent = formatLoad(load?.Fmax_N ?? 0);
      const delta = document.createElement('span');
      delta.className = 'delta';
      delta.textContent = formatMm(r.delta_mm);
      const frac = document.createElement('span');
      frac.className = 'frac';
      frac.textContent = formatPct(fractionOf(r.delta_mm, rows.deltaMax));
      grid.append(loc, force, delta, frac);
    }
    el.appendChild(grid);
  }
}

function decompose(sel: DeflectionQueryResult): BreakdownRows {
  const perLoadMap = new Map<number, number>();
  const perBeam = sel.beamDeflections.map((b) => {
    // Each sub-mode is rank-1, so max() returns its single argmax direction
    // and magnitude. Per-load attribution evaluates at that argmax.
    const at = (mode: Mode): number => {
      const bm = b.byMode.find((x) => x.mode === mode);
      if (!bm) return 0;
      const { dir, value } = bm.deflection.max();
      for (const pl of bm.perLoad) {
        perLoadMap.set(pl.loadIx, (perLoadMap.get(pl.loadIx) ?? 0) + pl.deflection.at(dir));
      }
      return value;
    };
    const bendIxTrans = at('bendIxTrans');
    const bendIxRot   = at('bendIxRot');
    const bendIyTrans = at('bendIyTrans');
    const bendIyRot   = at('bendIyRot');
    const twist       = at('twist');
    return {
      beamIx: b.beamIx,
      bendIxTrans, bendIxRot, bendIyTrans, bendIyRot, twist,
      total: bendIxTrans + bendIxRot + bendIyTrans + bendIyRot + twist,
    };
  });
  const deltaMax = perBeam.reduce((s, b) => s + b.total, 0);
  const perLoad = [...perLoadMap.entries()]
    .sort(([a], [z]) => a - z)
    .map(([loadIx, delta_mm]) => ({ loadIx, delta_mm }));
  return { deltaMax, perBeam, perLoad };
}

function headerCell(text: string): HTMLElement {
  const h = document.createElement('span');
  h.className = 'hdr';
  h.textContent = text;
  return h;
}

function valueCell(value_mm: number, total_mm: number, format: DecompFormat, isTotal = false): HTMLElement {
  const cell = document.createElement('span');
  cell.className = isTotal ? 'frac total' : 'frac';
  cell.textContent = format === 'pct'
    ? formatPct(fractionOf(value_mm, total_mm))
    : formatMm(value_mm);
  return cell;
}

function formatToggle(current: DecompFormat, onChange: (next: DecompFormat) => void): HTMLElement {
  const wrap = document.createElement('span');
  wrap.className = 'bd-fmt-toggle';
  const opts: { val: DecompFormat; label: string }[] = [
    { val: 'pct', label: 'pct' },
    { val: 'abs', label: 'abs' },
  ];
  for (const o of opts) {
    const btn = document.createElement('button');
    btn.textContent = o.label;
    btn.className = o.val === current ? 'active' : '';
    btn.addEventListener('click', () => onChange(o.val));
    wrap.appendChild(btn);
  }
  return wrap;
}

function sectionHeader(text: string): HTMLElement {
  const hdr = document.createElement('div');
  hdr.className = 'bd-section';
  hdr.textContent = text;
  return hdr;
}

// Semantic name for a query node's offset along its beam. Query nodes sit at
// beam boundaries (see buildQueryNodes), so a non-tip node is effectively
// always a beam end; the 'start' branch is a defensive guard.
function nodeLoc(offset_mm: number): string {
  return offset_mm <= 0 ? 'start' : 'end';
}

// Label element for a load: mass_accel body loads read "mass(<kg>)" so the
// compute basis is visible; explicit loads show their verbatim `load(...)`
// DSL text (whitespace-compacted). Both carry a distinct beam tag.
function loadLocEl(load: Load | undefined, prov: LoadProvenance | undefined, src: string): HTMLElement {
  const el = document.createElement('span');
  const label = prov?.source === 'mass_accel'
    ? `mass(${formatMass(prov.mass_kg ?? 0)})`
    : prov?.sourceSpan ? compact(src.slice(prov.sourceSpan.start, prov.sourceSpan.end)) : 'load';
  el.append(`${label} `);
  const tag = document.createElement('span');
  tag.className = 'bd-beamtag';
  tag.textContent = `beam${load?.beamIx ?? '?'}`;
  el.append(tag);
  return el;
}

// Mass label: 2 significant figures, kg as the home unit. Drop to grams below
// 0.1 kg, where kg would otherwise force leading-zero noise like "0.04".
function formatMass(kg: number): string {
  return kg < 0.1 ? `${sig2(kg * 1000)}g` : `${sig2(kg)}kg`;
}

function sig2(v: number): string {
  if (v === 0) return '0';
  const exp = Math.floor(Math.log10(Math.abs(v)));
  return v.toFixed(Math.max(0, 1 - exp));
}

// Collapse whitespace runs to a single space and trim — turns multi-line or
// loosely-spaced DSL into a compact single-line form.
function compact(s: string): string {
  return s.replace(/\s+/g, ' ').trim();
}

export function formatMm(v: number): string {
  if (v === 0) return '0';
  const a = Math.abs(v);
  const exp = Math.floor(Math.log10(a));
  if (exp < -4) return `${v.toExponential(1)} mm`;
  const decimals = 1 - exp;
  const scale = 10 ** decimals;
  const rounded = Math.round(v * scale) / scale;
  return decimals > 0
    ? `${rounded.toFixed(decimals)} mm`
    : `${rounded} mm`;
}

// Linearized rotation in degrees with 2 significant figures. The underlying
// math is in rad; deg is the display unit because that's what users name
// rotation tolerances in (mrad would be the alternative for optics).
function formatAngle(rad: number): string {
  if (rad === 0) return '0°';
  const deg = rad * (180 / Math.PI);
  return `${sig2(deg)}°`;
}

function formatLoad(N: number): string {
  return `${N.toFixed(2)} N`;
}

function fractionOf(part: number, whole: number): number {
  return whole > 1e-30 ? part / whole : 0;
}

function formatPct(frac: number): string {
  return `${(frac * 100).toFixed(0)}%`;
}
