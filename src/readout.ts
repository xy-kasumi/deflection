import type { SimResult } from './sim/run';
import { KGF_TO_N } from './state';

// Renders the deflection breakdown into #readout. Plain DOM, no framework.
// Keep this file impl-agnostic about how SimResult was built: it only reads.

export function renderReadout(el: HTMLElement, sim: SimResult): void {
  el.innerHTML = '';

  if (sim.nodes.length === 0) {
    el.innerHTML = '<span class="ro-empty">no chain</span>';
    return;
  }

  const tip = sim.nodes[sim.tipQueryIx];
  if (!tip) return;

  // Headline: δ tip ≈ value with display scale
  const headline = document.createElement('div');
  headline.className = 'ro-headline';
  headline.textContent = `δ tip ≈ ${formatMm(tip.delta_max_mm)}`;
  const scale = document.createElement('span');
  scale.className = 'ro-scale';
  scale.textContent = `×${sim.display_scale}`;
  headline.appendChild(scale);
  el.appendChild(headline);

  // Forces section.
  if (sim.forces.length > 0) {
    const hdr = document.createElement('div');
    hdr.className = 'ro-section';
    hdr.textContent = 'Forces:';
    el.appendChild(hdr);
    for (const f of sim.forces) {
      const row = document.createElement('div');
      row.className = 'ro-row';
      const left = document.createElement('span');
      left.textContent = `beam${f.beamIx} @${formatMm(f.offset_mm)} · ${formatForce(f.Fmax_N)} → ${formatMm(f.delta_mm)}`;
      const right = document.createElement('span');
      right.className = 'frac';
      right.textContent = formatPct(f.fraction);
      row.append(left, right);
      el.appendChild(row);
    }
  }

  // Beams section.
  if (sim.beams.length > 0) {
    const hdr = document.createElement('div');
    hdr.className = 'ro-section';
    hdr.textContent = 'Beams:';
    el.appendChild(hdr);
    for (const b of sim.beams) {
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

function formatMm(v: number): string {
  const a = Math.abs(v);
  if (a === 0) return '0';
  if (a >= 100)  return `${v.toFixed(0)} mm`;
  if (a >= 10)   return `${v.toFixed(1)} mm`;
  if (a >= 1)    return `${v.toFixed(2)} mm`;
  if (a >= 0.01) return `${v.toFixed(3)} mm`;
  if (a >= 1e-4) return `${v.toFixed(5)} mm`;
  return `${v.toExponential(2)} mm`;
}

function formatForce(N: number): string {
  const kgf = N / KGF_TO_N;
  if (kgf >= 1) return `${kgf.toFixed(2)} kgf`;
  return `${(N).toFixed(2)} N`;
}

function formatPct(frac: number): string {
  return `${(frac * 100).toFixed(0)}%`;
}
