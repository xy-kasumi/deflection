import type { LoadContribution, SimResult } from './sim/simulate';
import type { LoadProvenance } from './loadsystem';

// Renders the deflection breakdown pane into #info. Plain DOM, no framework.
// Keep this file impl-agnostic about how SimResult was built: it only reads.
// `src` is the DSL source — used to label explicit loads with their verbatim
// `load(...)` text.

export function renderBreakdown(
  el: HTMLElement,
  sim: SimResult | null,
  loadProvenance: LoadProvenance[],
  selectedNodeIx: number,
  tipNodeIx: number,
  src: string,
): void {
  el.innerHTML = '';

  if (!sim || sim.queryResults.length === 0) {
    el.innerHTML = '<span class="bd-empty">no chain</span>';
    return;
  }

  const sel = sim.queryResults[selectedNodeIx];
  if (!sel) return;
  const deltaMax = sel.deflection.max().value;

  // Header: the δ value, then where it's measured.
  const headline = document.createElement('div');
  headline.className = 'bd-headline';
  headline.textContent = `δ ≈ ${formatMm(deltaMax)}`;
  el.appendChild(headline);

  const subhead = document.createElement('div');
  subhead.className = 'bd-subhead';
  subhead.textContent = selectedNodeIx === tipNodeIx
    ? `tip (beam${sel.query.beamIx})`
    : `${nodeLoc(sel.query.offset_mm)} of beam${sel.query.beamIx}`;
  el.appendChild(subhead);

  // Beams section — before loads: "which beam to stiffen" is the actionable
  // question. Per-mode fractions can be negative (a mode that opposes d*).
  if (sel.beams.length > 0) {
    el.appendChild(sectionHeader('beams'));
    for (const b of sel.beams) {
      const bx = fractionOf(b.delta_mm_bendIx, deltaMax);
      const by = fractionOf(b.delta_mm_bendIy, deltaMax);
      const tor = fractionOf(b.delta_mm_torsion, deltaMax);
      const row = document.createElement('div');
      row.className = 'bd-row';
      const left = document.createElement('span');
      left.append(modeChip('bx', bx));
      left.append(modeChip('by', by));
      left.append(modeChip('tor', tor));
      const lbl = document.createElement('span');
      lbl.textContent = `beam${b.beamIx}  bx ${formatPct(bx)}  by ${formatPct(by)}  tor ${formatPct(tor)}`;
      left.appendChild(lbl);
      const right = document.createElement('span');
      right.className = 'frac';
      right.textContent = formatPct(fractionOf(b.delta_mm, deltaMax));
      row.append(left, right);
      el.appendChild(row);
    }
  }

  // Loads section. One shared grid so columns align across rows.
  if (sel.loads.length > 0) {
    el.appendChild(sectionHeader('loads'));
    const grid = document.createElement('div');
    grid.className = 'bd-loads';
    for (const c of sel.loads) {
      const loc = loadLocEl(c, loadProvenance[c.loadIx], src);
      const force = document.createElement('span');
      force.className = 'force';
      force.textContent = formatLoad(c.load.Fmax_N);
      const delta = document.createElement('span');
      delta.className = 'delta';
      delta.textContent = formatMm(c.delta_mm);
      const frac = document.createElement('span');
      frac.className = 'frac';
      frac.textContent = formatPct(fractionOf(c.delta_mm, deltaMax));
      grid.append(loc, force, delta, frac);
    }
    el.appendChild(grid);
  }
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
function loadLocEl(c: LoadContribution, prov: LoadProvenance | undefined, src: string): HTMLElement {
  const el = document.createElement('span');
  const label = prov?.source === 'mass_accel'
    ? `mass(${formatMass(prov.mass_kg ?? 0)})`
    : prov?.sourceSpan ? compact(src.slice(prov.sourceSpan.start, prov.sourceSpan.end)) : 'load';
  el.append(`${label} `);
  const tag = document.createElement('span');
  tag.className = 'bd-beamtag';
  tag.textContent = `beam${c.load.beamIx}`;
  el.append(tag);
  return el;
}

function formatMass(kg: number): string {
  return `${kg.toPrecision(3)}kg`;
}

// Collapse whitespace runs to a single space and trim — turns multi-line or
// loosely-spaced DSL into a compact single-line form.
function compact(s: string): string {
  return s.replace(/\s+/g, ' ').trim();
}

function modeChip(cls: string, frac: number): HTMLElement {
  const bar = document.createElement('span');
  bar.className = `bd-bar ${cls}`;
  const width = Math.max(0, Math.min(1, Math.abs(frac))) * 24 + 2;
  bar.style.width = `${width}px`;
  return bar;
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
