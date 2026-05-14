import type { SimResult, DeflectionQueryResult } from './sim/simulate';
import { sumDirectionals, type Directional } from './sim/directional';
import type { Load, Vec3 } from './sim/problem';
import type { Mode } from './sim/compliance';
import type { LoadProvenance } from './loadsystem';

export type DisplayMode = 'simple' | 'realistic';

// The Directional a query node's lobe represents under the current mode.
// Realistic: the true worst-case δ. Simple: the pessimistic sum of the
// per-(beam,mode) parts — an upper bound on δ (Σ ≥ δ, triangle inequality).
export function displayDirectional(n: DeflectionQueryResult, mode: DisplayMode): Directional {
  if (mode === 'realistic') return n.deflection;
  return sumDirectionals(n.beamDeflections.flatMap((b) => b.byMode.map((m) => m.deflection)));
}

// Renders the deflection breakdown pane into #info. Plain DOM, no framework.
// Keep this file impl-agnostic about how SimResult was built: it only reads.
// `src` is the DSL source — used to label explicit loads with their verbatim
// `load(...)` text.

export function renderBreakdown(
  el: HTMLElement,
  sim: SimResult | null,
  loads: Load[],
  loadProvenance: LoadProvenance[],
  selectedNodeIx: number,
  tipNodeIx: number,
  src: string,
  mode: DisplayMode,
): void {
  el.innerHTML = '';

  if (!sim || sim.queryResults.length === 0) {
    el.innerHTML = '<span class="bd-empty">no chain</span>';
    return;
  }

  const sel = sim.queryResults[selectedNodeIx];
  if (!sel) return;
  const { dir, value: deltaMax } = sel.deflection.max();

  // The breakdown is the deflection resolved in the worst-case direction d*:
  // apply that direction's worst-case forces, then read off the determined,
  // exactly-additive per-beam / per-load contributions (projected onto d*).
  const dd = sim.under(sel.forcesAt(dir)).queryResults[selectedNodeIx];

  // Header: the δ value, then where it's measured. Simple mode shows the
  // pessimistic bound it renders (δ ≲ …); the beams/loads below stay the
  // exact d* decomposition regardless of mode.
  const headline = document.createElement('div');
  headline.className = 'bd-headline';
  const headlineDelta = mode === 'simple' ? displayDirectional(sel, mode).max().value : deltaMax;
  headline.textContent = `δ ${mode === 'simple' ? '≲' : '≈'} ${formatMm(headlineDelta)}`;
  el.appendChild(headline);

  const subhead = document.createElement('div');
  subhead.className = 'bd-subhead';
  subhead.textContent = selectedNodeIx === tipNodeIx
    ? `tip (beam${sel.query.beamIx})`
    : `${nodeLoc(sel.query.offset_mm)} of beam${sel.query.beamIx}`;
  el.appendChild(subhead);

  // Beams section — before loads: "which beam to stiffen" is the actionable
  // question. Per-mode fractions can be negative (a mode that opposes d*).
  // The two bend modes carry an arrow glyph for the two orthogonal bending
  // planes; torsion is the plain word "twist" — it has no chirality to point.
  if (dd && dd.perBeam.length > 0) {
    el.appendChild(sectionHeader('beams'));
    const grid = document.createElement('div');
    grid.className = 'bd-beams';
    for (const text of ['', 'bend↕', 'bend↔', 'twist', 'total']) {
      const h = document.createElement('span');
      h.className = 'hdr';
      h.textContent = text;
      grid.append(h);
    }
    for (const b of dd.perBeam) {
      const name = document.createElement('span');
      name.textContent = `beam${b.beamIx}`;
      grid.append(
        name,
        fracCell(fractionOf(modeAlong(b.byMode, 'bendIx', dir), deltaMax)),
        fracCell(fractionOf(modeAlong(b.byMode, 'bendIy', dir), deltaMax)),
        fracCell(fractionOf(modeAlong(b.byMode, 'torsionJ', dir), deltaMax)),
        fracCell(fractionOf(dot(dir, b.vector_mm), deltaMax), true),
      );
    }
    el.appendChild(grid);
  }

  // Loads section. One shared grid so columns align across rows.
  if (dd && dd.perLoad.length > 0) {
    el.appendChild(sectionHeader('loads'));
    const grid = document.createElement('div');
    grid.className = 'bd-loads';
    const rows = dd.perLoad
      .map((pl) => ({ loadIx: pl.loadIx, delta_mm: dot(dir, pl.vector_mm) }))
      .sort((a, b) => b.delta_mm - a.delta_mm);
    for (const r of rows) {
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
      frac.textContent = formatPct(fractionOf(r.delta_mm, deltaMax));
      grid.append(loc, force, delta, frac);
    }
    el.appendChild(grid);
  }
}

// Signed contribution of one mode along `dir` — 0 if the beam doesn't excite it.
function modeAlong(
  byMode: { mode: Mode; vector_mm: Vec3 }[],
  mode: Mode,
  dir: Vec3,
): number {
  const m = byMode.find((x) => x.mode === mode);
  return m ? dot(dir, m.vector_mm) : 0;
}

function dot(a: Vec3, b: Vec3): number {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
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

// One right-aligned percentage cell in the beams grid. `total` cells render
// muted, matching the header.
function fracCell(frac: number, total = false): HTMLElement {
  const cell = document.createElement('span');
  cell.className = total ? 'frac total' : 'frac';
  cell.textContent = formatPct(frac);
  return cell;
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

function formatLoad(N: number): string {
  return `${N.toFixed(2)} N`;
}

// Display fraction of a signed contribution against the query's δ_max.
function fractionOf(part: number, whole: number): number {
  return whole > 1e-30 ? part / whole : 0;
}

function formatPct(frac: number): string {
  return `${(frac * 100).toFixed(0)}%`;
}
