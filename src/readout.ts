import type { SimResult } from './sim/run';
import { KGF_TO_N } from './state';

// Renders the deflection breakdown into #readout. Plain DOM, no framework.
// Keep this file impl-agnostic about how SimResult was built: it only reads.

export function renderReadout(el: HTMLElement, sim: SimResult, selectedNodeIx: number): void {
  el.innerHTML = '';

  if (sim.nodes.length === 0) {
    el.innerHTML = '<span class="ro-empty">no chain</span>';
    return;
  }

  const sel = sim.nodes[selectedNodeIx];
  if (!sel) return;

  const headline = document.createElement('div');
  headline.className = 'ro-headline';
  const tag = selectedNodeIx === sim.tipNodeIx
    ? 'tip'
    : `beam${sel.beamIx} @${formatMm(sel.offset_mm)}`;
  headline.textContent = `${tag} δ ≈ ${formatMm(sel.delta_max_mm)}`;
  el.appendChild(headline);

  // Loads section.
  if (sel.loads.length > 0) {
    const hdr = document.createElement('div');
    hdr.className = 'ro-section';
    hdr.textContent = 'Loads:';
    el.appendChild(hdr);
    for (const f of sel.loads) {
      const row = document.createElement('div');
      row.className = 'ro-row';
      const left = document.createElement('span');
      left.textContent = `beam${f.beamIx} @${formatMm(f.offset_mm)} · ${formatLoad(f.Fmax_N)} → ${formatMm(f.delta_mm)}`;
      const right = document.createElement('span');
      right.className = 'frac';
      right.textContent = formatPct(f.fraction);
      row.append(left, right);
      el.appendChild(row);
    }
  }

  // Beams section.
  if (sel.beams.length > 0) {
    const hdr = document.createElement('div');
    hdr.className = 'ro-section';
    hdr.textContent = 'Beams:';
    el.appendChild(hdr);
    for (const b of sel.beams) {
      const row = document.createElement('div');
      row.className = 'ro-row';
      const left = document.createElement('span');
      left.append(modeChip('ax',  b.axial_fraction));
      left.append(modeChip('bx',  b.bendIx_fraction));
      left.append(modeChip('by',  b.bendIy_fraction));
      left.append(modeChip('tor', b.torsion_fraction));
      const lbl = document.createElement('span');
      lbl.textContent = `beam${b.beamIx}  ax ${formatPct(b.axial_fraction)}  bx ${formatPct(b.bendIx_fraction)}  by ${formatPct(b.bendIy_fraction)}  tor ${formatPct(b.torsion_fraction)}`;
      left.appendChild(lbl);
      const right = document.createElement('span');
      right.className = 'frac';
      right.textContent = formatPct(b.total_fraction);
      row.append(left, right);
      el.appendChild(row);
    }
  }
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
  const kgf = N / KGF_TO_N;
  if (kgf >= 1) return `${kgf.toFixed(2)} kgf`;
  return `${(N).toFixed(2)} N`;
}

function formatPct(frac: number): string {
  return `${(frac * 100).toFixed(0)}%`;
}
