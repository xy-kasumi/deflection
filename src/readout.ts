import type { SimResult, SimLoad } from './sim/run';

// Renders the deflection breakdown into #info. Plain DOM, no framework.
// Keep this file impl-agnostic about how SimResult was built: it only reads.
// `src` is the DSL source — used to label explicit loads with their verbatim
// `load(...)` text.

export function renderReadout(el: HTMLElement, sim: SimResult, selectedNodeIx: number, src: string): void {
  el.innerHTML = '';

  if (sim.nodes.length === 0) {
    el.innerHTML = '<span class="ro-empty">no chain</span>';
    return;
  }

  const sel = sim.nodes[selectedNodeIx];
  if (!sel) return;

  // Header: the δ value, then where it's measured.
  const headline = document.createElement('div');
  headline.className = 'ro-headline';
  headline.textContent = `δ ≈ ${formatMm(sel.delta_max_mm)}`;
  el.appendChild(headline);

  const subhead = document.createElement('div');
  subhead.className = 'ro-subhead';
  subhead.textContent = selectedNodeIx === sim.tipNodeIx
    ? `tip (beam${sel.beamIx})`
    : `${nodeLoc(sel.offset_mm)} of beam${sel.beamIx}`;
  el.appendChild(subhead);

  // Beams section — before loads: "which beam to stiffen" is the actionable
  // question. Per-mode fractions can be negative (a mode that opposes d*).
  if (sel.beams.length > 0) {
    el.appendChild(sectionHeader('beams'));
    for (const b of sel.beams) {
      const row = document.createElement('div');
      row.className = 'ro-row';
      const left = document.createElement('span');
      left.append(modeChip('bx', b.bendIx_fraction));
      left.append(modeChip('by', b.bendIy_fraction));
      left.append(modeChip('tor', b.torsion_fraction));
      const lbl = document.createElement('span');
      lbl.textContent = `beam${b.beamIx}  bx ${formatPct(b.bendIx_fraction)}  by ${formatPct(b.bendIy_fraction)}  tor ${formatPct(b.torsion_fraction)}`;
      left.appendChild(lbl);
      const right = document.createElement('span');
      right.className = 'frac';
      right.textContent = formatPct(b.total_fraction);
      row.append(left, right);
      el.appendChild(row);
    }
  }

  // Loads section. One shared grid so columns align across rows.
  if (sel.loads.length > 0) {
    el.appendChild(sectionHeader('loads'));
    const grid = document.createElement('div');
    grid.className = 'ro-loads';
    for (const f of sel.loads) {
      const loc = loadLocEl(f, src);
      const force = document.createElement('span');
      force.className = 'force';
      force.textContent = formatLoad(f.Fmax_N);
      const delta = document.createElement('span');
      delta.className = 'delta';
      delta.textContent = formatMm(f.delta_mm);
      const frac = document.createElement('span');
      frac.className = 'frac';
      frac.textContent = formatPct(f.fraction);
      grid.append(loc, force, delta, frac);
    }
    el.appendChild(grid);
  }
}

function sectionHeader(text: string): HTMLElement {
  const hdr = document.createElement('div');
  hdr.className = 'ro-section';
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
function loadLocEl(f: SimLoad, src: string): HTMLElement {
  const el = document.createElement('span');
  const label = f.source === 'mass_accel'
    ? `mass(${formatMass(f.mass_kg ?? 0)})`
    : f.sourceSpan ? compact(src.slice(f.sourceSpan.start, f.sourceSpan.end)) : 'load';
  el.append(`${label} `);
  const tag = document.createElement('span');
  tag.className = 'ro-beamtag';
  tag.textContent = `beam${f.beamIx}`;
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
  bar.className = `ro-bar ${cls}`;
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

function formatPct(frac: number): string {
  return `${(frac * 100).toFixed(0)}%`;
}
