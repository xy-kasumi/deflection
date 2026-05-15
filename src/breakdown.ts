import type { SimResult, DeflectionQueryResult } from './sim/simulate';
import type { Load } from './sim/problem';
import type { Mode } from './sim/compliance';
import type { LoadProvenance } from './loadsystem';

export type DecompFormat = 'pct' | 'abs';

// Length unit auto-picked per breakdown to keep numbers in the "1.2 … 123"
// range. Sub-mm totals would otherwise render with leading-zero noise.
type LengthUnit = 'mm' | 'µm';

// Cursor over a breakdown cell. `keys` is one entry for a single (beam, mode)
// cell, or all five modes for a per-beam total cell. The scene draws one
// stick per query node per key.
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
const MODE_HEADERS = ['↕Δ', '↔Δ', '↕θ', '↔θ', '⟳'];
// Tooltip wording scopes "translation"/"rotation" inside the bending mode
// (the two Euler-Bernoulli contributions: u and du/ds), not the world frame
// — the resulting cell value is mm-at-the-query, whose world direction
// depends on this beam's pose, not on the column's symbol.
const MODE_TOOLTIPS = [
  "Translation part of this beam's ↕ bending. Resisted by Ix.",
  "Translation part of this beam's ↔ bending. Resisted by Iy.",
  "Rotation part of this beam's ↕ bending.",
  "Rotation part of this beam's ↔ bending.",
  "Twist of this beam.",
];

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
    `δ ≈ ${formatMm(sel.deflection_mm.furthest().distance)}` +
    `    δθ ≈ ${formatAngle(sel.rotation_rad.furthest().distance)}`;
  el.appendChild(headline);

  const subhead = document.createElement('div');
  subhead.className = 'bd-subhead';
  subhead.textContent = selectedNodeIx === tipNodeIx
    ? `tip (beam${sel.query.beamIx})`
    : `${nodeLoc(sel.query.offset_mm)} of beam${sel.query.beamIx}`;
  el.appendChild(subhead);

  const rows = decompose(sel);

  // The section's unit is picked from the pessimistic total — drops to µm
  // when even the budget is sub-mm so cells don't read as "0.0012 …".
  const unit: LengthUnit = rows.deltaMax >= 1 ? 'mm' : 'µm';

  // Decomposition section header + pessimistic total + format toggle.
  const decompHeader = document.createElement('div');
  decompHeader.className = 'bd-section bd-section-toolbar';
  const decompTitle = document.createElement('span');
  decompTitle.append(`decomposition of δ (tot ≲ ${formatLen(rows.deltaMax, unit)}) `);
  const help = document.createElement('span');
  help.className = 'bd-help';
  help.textContent = '?';
  help.title = 'Pessimistic upper bound. (Worst case per beam/mode, summed.)';
  decompTitle.appendChild(help);
  decompHeader.append(decompTitle, formatToggle(format, unit, onFormatChange));
  el.appendChild(decompHeader);

  // Beams grid. Five sub-mode columns + total. Each sub-mode cell is one
  // (beamIx, Mode) pair; the total cell hover fires all five modes of that
  // beam at once.
  if (rows.perBeam.length > 0) {
    const grid = document.createElement('div');
    grid.className = 'bd-beams';
    grid.append(headerCell(''));
    MODE_HEADERS.forEach((text, i) => grid.append(headerCell(text, MODE_TOOLTIPS[i])));
    grid.append(headerCell('total'));

    // Heatmap: cells fade by their underlying mm value, regardless of pct/abs
    // display. Sub-modes and totals normalize separately — totals have a
    // larger range, so sharing one max would wash sub-mode cells out.
    const subModeMax = Math.max(
      0,
      ...rows.perBeam.flatMap((b) => [
        b.bendIxTrans, b.bendIyTrans, b.bendIxRot, b.bendIyRot, b.twist,
      ]),
    );
    const totalsMax = Math.max(0, ...rows.perBeam.map((b) => b.total));

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
        const cell = valueCell(
          fmtBeamCell(val, rows.deltaMax, format, unit),
          intensityOf(val, subModeMax),
        );
        if (hover) {
          const m = MODE_COLUMNS[i]!;
          cell.classList.add('hoverable');
          cell.addEventListener('mouseenter', () => hover.enter([{ beamIx: b.beamIx, mode: m }]));
          cell.addEventListener('mouseleave', () => hover.leave());
        }
        grid.append(cell);
      });

      const totalCell = valueCell(
        fmtBeamCell(b.total, rows.deltaMax, format, unit),
        intensityOf(b.total, totalsMax),
        true,
      );
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
    const loadsMax = loadRows.reduce((m, r) => Math.max(m, r.delta_mm), 0);
    for (const r of loadRows) {
      const load = loads[r.loadIx];
      const loc = loadLocEl(load, loadProvenance[r.loadIx], src);
      const force = document.createElement('span');
      force.className = 'force';
      force.textContent = formatLoad(load?.Fmax_N ?? 0);
      const value = valueCell(
        fmtLoadCell(r.delta_mm, rows.deltaMax, format, unit),
        intensityOf(r.delta_mm, loadsMax),
      );
      grid.append(loc, force, value);
    }
    el.appendChild(grid);
  }
}

function decompose(sel: DeflectionQueryResult): BreakdownRows {
  const perLoadMap = new Map<number, number>();
  const perBeam = sel.beamDeflections.map((b) => {
    // Each sub-mode is rank-1, so furthest() returns its single furthest-point
    // and distance. Per-load attribution evaluates support at that point.
    const at = (mode: Mode): number => {
      const bm = b.byMode.find((x) => x.mode === mode);
      if (!bm) return 0;
      const { point, distance } = bm.deflection_mm.furthest();
      if (distance === 0) return 0;
      const inv = 1 / distance;
      const d: [number, number, number] = [point[0] * inv, point[1] * inv, point[2] * inv];
      for (const pl of bm.perLoad) {
        perLoadMap.set(pl.loadIx, (perLoadMap.get(pl.loadIx) ?? 0) + pl.deflection_mm.support(d));
      }
      return distance;
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

function headerCell(text: string, title?: string): HTMLElement {
  const h = document.createElement('span');
  h.className = 'hdr';
  h.textContent = text;
  if (title) h.title = title;
  return h;
}

// `intensity` ∈ [0, 1] from the caller; opacity floors at 0.2 so 0-value
// cells stay legible while max-value cells read at full strength.
function valueCell(text: string, intensity: number, isTotal = false): HTMLElement {
  const cell = document.createElement('span');
  cell.className = isTotal ? 'frac total' : 'frac';
  cell.textContent = text;
  cell.style.opacity = String(0.2 + 0.8 * clamp01(intensity));
  return cell;
}

// Beam cells: bare number, since the toggle button advertises the unit and
// every cell in the grid shares it.
function fmtBeamCell(value_mm: number, total_mm: number, format: DecompFormat, unit: LengthUnit): string {
  return format === 'pct'
    ? formatPct(fractionOf(value_mm, total_mm))
    : formatNum(value_mm, unit);
}

// Load cells: keep the unit suffix. Loads rows mix kinds (force in N, then a
// length) and the explicit "mm"/"µm" prevents the value from looking like a
// dimensionless companion to the N.
function fmtLoadCell(value_mm: number, total_mm: number, format: DecompFormat, unit: LengthUnit): string {
  return format === 'pct'
    ? formatPct(fractionOf(value_mm, total_mm))
    : formatLen(value_mm, unit);
}

function intensityOf(value: number, max: number): number {
  return max > 1e-30 ? value / max : 0;
}

function clamp01(x: number): number {
  return Math.max(0, Math.min(1, x));
}

function formatToggle(
  current: DecompFormat,
  unit: LengthUnit,
  onChange: (next: DecompFormat) => void,
): HTMLElement {
  const wrap = document.createElement('span');
  wrap.className = 'bd-fmt-toggle';
  const opts: { val: DecompFormat; label: string }[] = [
    { val: 'pct', label: '%' },
    { val: 'abs', label: unit },
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

// Below 1 nm everything is FP noise — collapse to exact 0.
const ZERO_EPS_MM = 1e-6;

export function formatMm(v: number): string {
  if (Math.abs(v) < ZERO_EPS_MM) return '0';
  const exp = Math.floor(Math.log10(Math.abs(v)));
  if (exp < -4) return `${v.toExponential(1)} mm`;
  const decimals = 1 - exp;
  const scale = 10 ** decimals;
  const rounded = Math.round(v * scale) / scale;
  return decimals > 0
    ? `${rounded.toFixed(decimals)} mm`
    : `${rounded} mm`;
}

// Bare number, scaled to the section's unit.
function formatNum(v_mm: number, unit: LengthUnit): string {
  if (Math.abs(v_mm) < ZERO_EPS_MM) return '0';
  const v = unit === 'mm' ? v_mm : v_mm * 1000;
  const exp = Math.floor(Math.log10(Math.abs(v)));
  if (exp < -4) return v.toExponential(1);
  const decimals = Math.max(0, 1 - exp);
  return v.toFixed(decimals);
}

function formatLen(v_mm: number, unit: LengthUnit): string {
  return `${formatNum(v_mm, unit)} ${unit}`;
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
