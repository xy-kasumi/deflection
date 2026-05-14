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

/** A hovered breakdown component: a beam (mode null) or one of its modes. */
export type HoverKey = { beamIx: number; mode: Mode | null };

// The isolated Directional for a hovered component, or null if it can't be
// resolved (e.g. the beam has no entry for that mode).
export function hoverDirectional(n: DeflectionQueryResult, h: HoverKey): Directional | null {
  const bd = n.beamDeflections.find((b) => b.beamIx === h.beamIx);
  if (!bd) return null;
  if (h.mode === null) return bd.deflection;
  return bd.byMode.find((m) => m.mode === h.mode)?.deflection ?? null;
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
  onHover: (h: HoverKey | null) => void,
  pickedDir: Vec3 | null,
  onPickDir: (d: Vec3) => void,
): void {
  el.innerHTML = '';

  if (!sim || sim.queryResults.length === 0) {
    el.innerHTML = '<span class="bd-empty">no chain</span>';
    return;
  }

  const sel = sim.queryResults[selectedNodeIx];
  if (!sel) return;

  const rows = mode === 'simple'
    ? simpleRows(sel)
    : realisticRows(sim, sel, selectedNodeIx, pickedDir);

  // Realistic mode: the equirectangular picker selects the direction `rows`
  // is decomposed in (above) — click to pick.
  if (mode === 'realistic') {
    el.appendChild(buildPicker(sel, pickedDir, onPickDir));
  }

  // Header: the δ value, then where it's measured. Simple shows the
  // pessimistic bound it renders (δ ≲ …), Realistic the true worst case
  // (δ ≈ …); either way the beams/loads below sum to it.
  const headline = document.createElement('div');
  headline.className = 'bd-headline';
  headline.textContent = `δ ${rows.glyph} ${formatMm(rows.deltaMax)}`;
  el.appendChild(headline);

  const subhead = document.createElement('div');
  subhead.className = 'bd-subhead';
  subhead.textContent = selectedNodeIx === tipNodeIx
    ? `tip (beam${sel.query.beamIx})`
    : `${nodeLoc(sel.query.offset_mm)} of beam${sel.query.beamIx}`;
  el.appendChild(subhead);

  // Beams section — before loads: "which beam to stiffen" is the actionable
  // question. Realistic per-mode values can be negative (a mode that opposes
  // d*); Simple values are non-negative. The two bend modes carry an arrow
  // glyph for the two orthogonal bending planes; torsion is the plain word
  // "twist" — it has no chirality to point.
  if (rows.perBeam.length > 0) {
    el.appendChild(sectionHeader('beams'));
    const grid = document.createElement('div');
    grid.className = 'bd-beams';
    for (const text of ['', 'bend↕', 'bend↔', 'twist', 'total']) {
      const h = document.createElement('span');
      h.className = 'hdr';
      h.textContent = text;
      grid.append(h);
    }
    for (const b of rows.perBeam) {
      const name = document.createElement('span');
      name.textContent = `beam${b.beamIx}`;
      const cells = [
        name,
        fracCell(fractionOf(b.bendIx, rows.deltaMax)),
        fracCell(fractionOf(b.bendIy, rows.deltaMax)),
        fracCell(fractionOf(b.torsionJ, rows.deltaMax)),
        fracCell(fractionOf(b.total, rows.deltaMax), true),
      ];
      // Simple mode: each cell is a hover target. Name and total cells stand
      // for the whole beam (mode unset); the three frac cells for one mode.
      if (mode === 'simple') {
        const cellModes: (Mode | null)[] = [null, 'bendIx', 'bendIy', 'torsionJ', null];
        cells.forEach((cell, i) => {
          cell.classList.add('bd-hover');
          cell.dataset['beam'] = String(b.beamIx);
          const m = cellModes[i];
          if (m) cell.dataset['mode'] = m;
        });
      }
      grid.append(...cells);
    }
    if (mode === 'simple') {
      grid.addEventListener('pointermove', (e) => {
        const cell = (e.target as HTMLElement).closest('[data-beam]') as HTMLElement | null;
        if (!cell) { onHover(null); return; }
        const m = cell.dataset['mode'] as Mode | undefined;
        onHover({ beamIx: Number(cell.dataset['beam']), mode: m ?? null });
      });
      grid.addEventListener('pointerleave', () => onHover(null));
    }
    el.appendChild(grid);
  }

  // Loads section. One shared grid so columns align across rows.
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

// ── equirectangular direction picker ───────────────────────────────────────
// A δ(dir) heatmap over the full sphere: x = azimuth θ about world-up (+Y),
// y = polar angle φ from +Y — so the canvas's vertical axis is world-up.
// Clicking unprojects to a direction.
const PICKER_W = 256;
const PICKER_H = 128;

function buildPicker(
  sel: DeflectionQueryResult,
  pickedDir: Vec3 | null,
  onPickDir: (d: Vec3) => void,
): HTMLCanvasElement {
  const canvas = document.createElement('canvas');
  canvas.className = 'bd-picker';
  canvas.width = PICKER_W;
  canvas.height = PICKER_H;
  const ctx = canvas.getContext('2d')!;

  const dmax = sel.deflection.max().value || 1;
  const img = ctx.createImageData(PICKER_W, PICKER_H);
  for (let py = 0; py < PICKER_H; py++) {
    for (let px = 0; px < PICKER_W; px++) {
      const t = sel.deflection.at(pickerUnproject(px + 0.5, py + 0.5)) / dmax;
      const [r, g, b] = pickerRamp(t);
      const o = (py * PICKER_W + px) * 4;
      img.data[o] = r;
      img.data[o + 1] = g;
      img.data[o + 2] = b;
      img.data[o + 3] = 255;
    }
  }
  ctx.putImageData(img, 0, 0);

  // Marker at the current direction — the pick, or the auto d* when none.
  const [mx, my] = pickerProject(pickedDir ?? sel.deflection.max().dir);
  for (const [radius, color, width] of [[4, '#fff', 2], [4, '#000', 1]] as const) {
    ctx.lineWidth = width;
    ctx.strokeStyle = color;
    ctx.beginPath();
    ctx.arc(mx, my, radius, 0, Math.PI * 2);
    ctx.stroke();
  }

  canvas.addEventListener('click', (e) => {
    const rect = canvas.getBoundingClientRect();
    const px = ((e.clientX - rect.left) / rect.width) * PICKER_W;
    const py = ((e.clientY - rect.top) / rect.height) * PICKER_H;
    onPickDir(pickerUnproject(px, py));
  });

  return canvas;
}

// Canvas pixel → world direction. θ ∈ [0,2π) about +Y, φ ∈ [0,π] from +Y.
function pickerUnproject(px: number, py: number): Vec3 {
  const theta = (px / PICKER_W) * 2 * Math.PI;
  const phi = (py / PICKER_H) * Math.PI;
  const s = Math.sin(phi);
  return [s * Math.cos(theta), Math.cos(phi), s * Math.sin(theta)];
}

// World direction → canvas pixel (inverse of pickerUnproject).
function pickerProject(d: Vec3): [number, number] {
  const phi = Math.acos(Math.max(-1, Math.min(1, d[1])));
  let theta = Math.atan2(d[2], d[0]);
  if (theta < 0) theta += 2 * Math.PI;
  return [(theta / (2 * Math.PI)) * PICKER_W, (phi / Math.PI) * PICKER_H];
}

// δ heatmap ramp: panel-light at 0 → warm amber at the peak. Deflection's
// domain colour is warm (the scene lobes too); red stays reserved for errors.
function pickerRamp(t: number): [number, number, number] {
  const c = Math.max(0, Math.min(1, t));
  return [
    Math.round(246 + (232 - 246) * c),
    Math.round(247 + (148 - 247) * c),
    Math.round(249 + (60 - 249) * c),
  ];
}

interface BreakdownRows {
  /** Headline δ; every contribution below is a fraction of this. */
  deltaMax: number;
  /** Header glyph: ≈ for the true value, ≲ for the pessimistic bound. */
  glyph: string;
  perBeam: { beamIx: number; bendIx: number; bendIy: number; torsionJ: number; total: number }[];
  perLoad: { loadIx: number; delta_mm: number }[];
}

// Realistic: the exact deflection resolved in a direction — the user-picked
// dir, or the true worst case d* when none is picked. Apply that direction's
// worst-case forces and read the determined contributions projected onto it —
// signed, summing exactly to δ(dir).
function realisticRows(
  sim: SimResult,
  sel: DeflectionQueryResult,
  selIx: number,
  pickedDir: Vec3 | null,
): BreakdownRows {
  let dir: Vec3;
  let deltaMax: number;
  if (pickedDir) {
    dir = pickedDir;
    deltaMax = sel.deflection.at(pickedDir);
  } else {
    const m = sel.deflection.max();
    dir = m.dir;
    deltaMax = m.value;
  }
  const dd = sim.under(sel.forcesAt(dir)).queryResults[selIx];
  if (!dd) return { deltaMax, glyph: '≈', perBeam: [], perLoad: [] };
  return {
    deltaMax,
    glyph: '≈',
    perBeam: dd.perBeam.map((b) => ({
      beamIx: b.beamIx,
      bendIx: modeAlong(b.byMode, 'bendIx', dir),
      bendIy: modeAlong(b.byMode, 'bendIy', dir),
      torsionJ: modeAlong(b.byMode, 'torsionJ', dir),
      total: dot(dir, b.vector_mm),
    })),
    perLoad: dd.perLoad.map((pl) => ({ loadIx: pl.loadIx, delta_mm: dot(dir, pl.vector_mm) })),
  };
}

// Simple: the pessimistic sum decomposed at its own argmax d_simple*. Every
// part (per beam, per mode, per load) is evaluated at that one direction, so —
// the sum being pointwise-exact — the parts sum exactly to the δ ≲ headline.
// All non-negative: each isolated δ is a sum of F·|N·d|.
function simpleRows(sel: DeflectionQueryResult): BreakdownRows {
  const { dir: d, value: deltaMax } = displayDirectional(sel, 'simple').max();
  return {
    deltaMax,
    glyph: '≲',
    perBeam: sel.beamDeflections.map((b) => {
      const at = (m: Mode) => b.byMode.find((x) => x.mode === m)?.deflection.at(d) ?? 0;
      const bendIx = at('bendIx');
      const bendIy = at('bendIy');
      const torsionJ = at('torsionJ');
      return { beamIx: b.beamIx, bendIx, bendIy, torsionJ, total: bendIx + bendIy + torsionJ };
    }),
    perLoad: sel.loadDeflections.map((l) => ({ loadIx: l.loadIx, delta_mm: l.deflection.at(d) })),
  };
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
