import * as THREE from 'three';
import type { BeamNode, Vec3 } from '../walker';
import type { SimResult } from '../sim/simulate';
import type { ConvexEnvelope } from '../sim/math';
import { formatMm } from '../breakdown';
import { COLOR } from './tokens';

const LOBE_CAP_LO = 0.975;

const LOBE_INTERIOR_ALPHA = 0.05;
const LOBE_SHELL_ALPHA = 0.3;
const LOBE_CAP_ALPHA = 0.5;

const LOBE_POINT_COUNT = 10000;
// PointsMaterial.size is NOT scaled by mesh.scale; sizeAttenuation:false
// keeps point density stable across the displayScale animation.
const LOBE_POINT_SIZE_PX = 3;
const LOBE_POINT_SIZE_MIN_PX = 2;
const LOBE_CAP_POINT_BOOST = 1.5;

// δ_min/δ_max above this → render as uniform shell (no cap painting), matching
// the underflow-sphere affordance for "no meaningful direction here."
const LOBE_ISOTROPIC_RATIO = LOBE_CAP_LO;

const LOBE_CEIL_AREA_FRAC = 0.075;
// Smoothstep edges: lower bound is a fraction of the threshold, upper bound
// is the threshold itself.
const LOBE_UNDER_BLEND_LO = 0.7;
const LOBE_OVER_BLEND_LO  = 0.85;
const LOBE_KONPEITO_FRAC  = 0.5;

// Radial-falloff alpha mask sampled at gl_PointCoord; turns square GL points
// into soft round splats.
const SOFT_POINT_TEX: THREE.CanvasTexture = (() => {
  const size = 64;
  const c = document.createElement('canvas');
  c.width = c.height = size;
  const ctx = c.getContext('2d')!;
  const grad = ctx.createRadialGradient(size / 2, size / 2, 0, size / 2, size / 2, size / 2);
  grad.addColorStop(0, 'rgba(255,255,255,1)');
  grad.addColorStop(0.5, 'rgba(255,255,255,0.6)');
  grad.addColorStop(1, 'rgba(255,255,255,0)');
  ctx.fillStyle = grad;
  ctx.fillRect(0, 0, size, size);
  return new THREE.CanvasTexture(c);
})();

function fibonacciS2(n: number): Vec3[] {
  const out: Vec3[] = [];
  const golden = Math.PI * (3 - Math.sqrt(5));
  for (let i = 0; i < n; i++) {
    const z = 1 - (2 * (i + 0.5)) / n;
    const r = Math.sqrt(Math.max(0, 1 - z * z));
    const phi = golden * i;
    out.push([r * Math.cos(phi), r * Math.sin(phi), z]);
  }
  return out;
}

interface LobeAnim {
  delta_max_mm: number;
  normalBase: number;
  underBase: number;
  overBase: number;
  normal: THREE.Object3D;
  under: THREE.Mesh;
  over: THREE.Mesh;
}

export interface LobeBuildOpts {
  hitRadius: number;
  labelOffset: number;
  lobeFloor: number;
  selectedNodeIx: number;
  focused: boolean;
}

export interface LobeLabelSpec {
  text: string;
  worldPos: THREE.Vector3;
  nodeIx: number;
  classes: string[];
}

export interface LobeBuildResult {
  meshes: THREE.Object3D[];
  pickables: THREE.Mesh[];
  labels: LobeLabelSpec[];
}

export class LobeRenderer {
  private lobeAnims: LobeAnim[] = [];
  private lobeFloor = 0;

  buildFor(sim: SimResult, beams: BeamNode[], opts: LobeBuildOpts): LobeBuildResult {
    this.lobeAnims = [];
    this.lobeFloor = opts.lobeFloor;

    const meshes: THREE.Object3D[] = [];
    const pickables: THREE.Mesh[] = [];
    const labels: LobeLabelSpec[] = [];

    for (let ix = 0; ix < sim.queryResults.length; ix++) {
      const n = sim.queryResults[ix]!;
      const isSel = ix === opts.selectedNodeIx;
      const worldPos = new THREE.Vector3(...n.pos_mm);

      const lobeDir = n.deflection_mm;
      const delta_max_mm = lobeDir.furthest().distance;

      const normal = buildNormalPoints(lobeDir);
      normal.position.copy(worldPos);
      meshes.push(normal);

      const under = buildUnderflowSphere(opts.lobeFloor);
      under.position.copy(worldPos);
      meshes.push(under);

      const over = buildKonpeito();
      over.position.copy(worldPos);
      meshes.push(over);

      this.lobeAnims.push({
        delta_max_mm,
        normalBase: opts.focused ? 0.15 : isSel ? 0.75 : 0.25,
        underBase:  opts.focused ? 0.15 : isSel ? 0.75 : 0.45,
        overBase:   opts.focused ? 0.15 : isSel ? 0.75 : 0.45,
        normal,
        under,
        over,
      });

      const hit = new THREE.Mesh(
        new THREE.SphereGeometry(opts.hitRadius, 8, 6),
        new THREE.MeshBasicMaterial({ visible: false }),
      );
      hit.visible = false;
      hit.position.copy(worldPos);
      hit.userData['nodeIx'] = ix;
      meshes.push(hit);
      pickables.push(hit);

      const up = new THREE.Vector3(...beams[n.query.beamIx]!.startFrame.up);
      const labelPos = worldPos.clone().add(up.multiplyScalar(opts.labelOffset));
      const classes: string[] = [];
      if (isSel) classes.push('selected');
      labels.push({
        text: `δ ${formatMm(delta_max_mm)}`,
        worldPos: labelPos,
        nodeIx: ix,
        classes,
      });
    }

    return { meshes, pickables, labels };
  }

  reset(): void {
    this.lobeAnims = [];
  }

  apply(scale: number, targetScale: number, lobeCeilWorld: number): void {
    if (this.lobeAnims.length === 0) return;
    const floor = this.lobeFloor;

    for (const a of this.lobeAnims) {
      // Target scale decides which variant the lobe is heading toward;
      // current scale paces the fade. If the target says "no over/under",
      // force it off the whole transition (otherwise the raw smoothstep
      // would flash a fractional konpeito/underflow that vanishes at settle).
      // If the target says "yes", let the smoothstep at the current scale
      // fade it in.
      const outerMaxTarget = a.delta_max_mm * targetScale;
      const overOn  = smoothstep(LOBE_OVER_BLEND_LO * lobeCeilWorld, lobeCeilWorld, outerMaxTarget) >= 0.5;
      const underOn = (1 - smoothstep(LOBE_UNDER_BLEND_LO * floor, floor, outerMaxTarget)) >= 0.5;
      const outerMax = a.delta_max_mm * scale;
      const aOver  = overOn  ? smoothstep(LOBE_OVER_BLEND_LO * lobeCeilWorld, lobeCeilWorld, outerMax) : 0;
      const aUnder = underOn ? 1 - smoothstep(LOBE_UNDER_BLEND_LO * floor, floor, outerMax) : 0;

      // Clamp the normal lobe so it never visibly outgrows the konpeito mid-
      // crossfade. δ_max = 0 → aUnder = 1 already hides it; skip the divide.
      const normalScale = a.delta_max_mm > 0
        ? Math.min(scale, lobeCeilWorld / a.delta_max_mm)
        : scale;
      a.normal.scale.setScalar(normalScale);
      a.over.scale.setScalar(lobeCeilWorld * LOBE_KONPEITO_FRAC);

      const normalOpacity = a.normalBase * (1 - aUnder) * (1 - aOver);
      const underOpacity  = a.underBase  * aUnder;
      const overOpacity   = a.overBase   * aOver;

      setLobeOpacity(a.normal, normalOpacity);
      setLobeOpacity(a.under, underOpacity);
      setLobeOpacity(a.over, overOpacity);

      // Equalize visual density across lobes of different displayed sizes:
      // covered-pixels / lobe-area is constant when point_size scales with
      // lobe screen radius. Clamped (ceiling-sized) lobe → factor = 1.
      const sizeFactor = lobeCeilWorld > 0
        ? (a.delta_max_mm * normalScale) / lobeCeilWorld
        : 1;
      setLobePointSize(a.normal, sizeFactor);
    }
  }
}

// Recomputed each frame so resize and ortho-zoom stay correct.
export function computeLobeCeilWorld(canvas: HTMLCanvasElement, scaleHalf: number): number {
  const w = canvas.clientWidth || 1;
  const h = canvas.clientHeight || 1;
  const lobeCeilPx = Math.sqrt((w * h * LOBE_CEIL_AREA_FRAC) / Math.PI);
  const aspect = w / h;
  const worldPerPx = aspect >= 1
    ? (2 * scaleHalf) / h
    : (2 * scaleHalf) / w;
  return lobeCeilPx * worldPerPx;
}

// Three THREE.Points (interior + shell + cap) in a Group. Positions in mm;
// the Group's scale carries displayScale.
function buildNormalPoints(dir: ConvexEnvelope): THREE.Object3D {
  const dMax = dir.furthest().distance;
  const samples = fibonacciS2(LOBE_POINT_COUNT);

  let dMin = dMax;
  const N_ISO = 24;
  const stride = Math.max(1, Math.floor(LOBE_POINT_COUNT / N_ISO));
  for (let i = 0; i < LOBE_POINT_COUNT; i += stride) {
    const v = dir.support(samples[i]!);
    if (v < dMin) dMin = v;
  }
  const isIsotropic = dMax > 0 && dMin / dMax > LOBE_ISOTROPIC_RATIO;

  // Each S² direction emits two points: a boundary point at bdy(d), and an
  // interior chord sample at r·bdy(d) with r = u^(1/3) (volume-uniform for
  // rank-3 K; rank-deficient K still fills its affine hull, biased outward).
  // Boundary points near δ_max peel off as `capPos` for the peak-highlight
  // pass; interior lives in its own buffer with its own (dimmer) base alpha.
  const shellPos: number[] = [];
  const capPos: number[] = [];
  const interiorPos: number[] = [];
  const dMaxInv = dMax > 0 ? 1 / dMax : 0;
  for (const d of samples) {
    const b = dir.boundary(d);
    const t = Math.hypot(b[0], b[1], b[2]) * dMaxInv;
    const target = !isIsotropic && t >= LOBE_CAP_LO ? capPos : shellPos;
    target.push(b[0], b[1], b[2]);
    const r = Math.cbrt(Math.random());
    interiorPos.push(b[0] * r, b[1] * r, b[2] * r);
  }

  const group = new THREE.Group();

  if (interiorPos.length > 0) {
    const geom = new THREE.BufferGeometry();
    geom.setAttribute('position', new THREE.Float32BufferAttribute(interiorPos, 3));
    geom.boundingSphere = new THREE.Sphere(new THREE.Vector3(0, 0, 0), dMax || 1);
    const mat = new THREE.PointsMaterial({
      color: COLOR.deformed,
      size: LOBE_POINT_SIZE_PX,
      sizeAttenuation: false,
      map: SOFT_POINT_TEX,
      // Interior base alpha is already dim; the per-lobe fade for non-
      // selected nodes drops it further. alphaTest must stay below the
      // dimmest effective alpha, or every interior pixel gets culled.
      alphaTest: 0,
      transparent: true,
      opacity: LOBE_INTERIOR_ALPHA,
      depthWrite: false,
    });
    mat.userData['baseOpacity'] = LOBE_INTERIOR_ALPHA;
    mat.userData['baseSize'] = LOBE_POINT_SIZE_PX;
    group.add(new THREE.Points(geom, mat));
  }

  if (shellPos.length > 0) {
    const geom = new THREE.BufferGeometry();
    geom.setAttribute('position', new THREE.Float32BufferAttribute(shellPos, 3));
    geom.boundingSphere = new THREE.Sphere(new THREE.Vector3(0, 0, 0), dMax || 1);
    const mat = new THREE.PointsMaterial({
      color: COLOR.deformed,
      size: LOBE_POINT_SIZE_PX,
      sizeAttenuation: false,
      map: SOFT_POINT_TEX,
      alphaTest: 0.02,
      transparent: true,
      opacity: LOBE_SHELL_ALPHA,
      depthWrite: false,
    });
    mat.userData['baseOpacity'] = LOBE_SHELL_ALPHA;
    mat.userData['baseSize'] = LOBE_POINT_SIZE_PX;
    group.add(new THREE.Points(geom, mat));
  }

  if (capPos.length > 0) {
    const geom = new THREE.BufferGeometry();
    geom.setAttribute('position', new THREE.Float32BufferAttribute(capPos, 3));
    geom.boundingSphere = new THREE.Sphere(new THREE.Vector3(0, 0, 0), dMax || 1);
    const mat = new THREE.PointsMaterial({
      color: COLOR.deformedPeak,
      size: LOBE_POINT_SIZE_PX * LOBE_CAP_POINT_BOOST,
      sizeAttenuation: false,
      map: SOFT_POINT_TEX,
      alphaTest: 0.02,
      transparent: true,
      opacity: LOBE_CAP_ALPHA,
      depthWrite: false,
    });
    mat.userData['baseOpacity'] = LOBE_CAP_ALPHA;
    mat.userData['baseSize'] = LOBE_POINT_SIZE_PX * LOBE_CAP_POINT_BOOST;
    group.add(new THREE.Points(geom, mat));
  }

  return group;
}

function buildUnderflowSphere(floor: number): THREE.Mesh {
  const geom = new THREE.SphereGeometry(floor, 12, 8);
  const mat = new THREE.MeshBasicMaterial({
    color: COLOR.deformed,
    transparent: true,
    opacity: 1,
    depthWrite: false,
  });
  return new THREE.Mesh(geom, mat);
}

function buildKonpeito(): THREE.Mesh {
  const geom = new THREE.IcosahedronGeometry(1, 1);
  const pos = geom.attributes.position!;
  for (let i = 0; i < pos.count; i++) {
    const x = pos.getX(i);
    const y = pos.getY(i);
    const z = pos.getZ(i);
    const seed = Math.abs(Math.sin(i * 12.9898) * 43758.5453);
    const r = 1.0 + (seed - Math.floor(seed)) * 0.4;
    pos.setXYZ(i, x * r, y * r, z * r);
  }
  pos.needsUpdate = true;
  geom.computeVertexNormals();
  geom.computeBoundingSphere();
  const mat = new THREE.MeshBasicMaterial({
    color: COLOR.deformedPeak,
    transparent: true,
    opacity: 1,
    depthWrite: false,
  });
  return new THREE.Mesh(geom, mat);
}

function smoothstep(edge0: number, edge1: number, x: number): number {
  const t = Math.max(0, Math.min(1, (x - edge0) / Math.max(1e-9, edge1 - edge0)));
  return t * t * (3 - 2 * t);
}

// Multiplies each material's userData.baseOpacity (1 if unset) by `fade`.
function setLobeOpacity(obj: THREE.Object3D, fade: number): void {
  obj.traverse((child) => {
    const m = (child as THREE.Mesh | THREE.Points).material;
    if (m && !Array.isArray(m) && 'opacity' in m) {
      const base = (m.userData['baseOpacity'] ?? 1) as number;
      m.opacity = base * fade;
    }
  });
  obj.visible = fade > 1e-3;
}

// Multiplies each PointsMaterial's userData.baseSize by `factor`, floored
// at LOBE_POINT_SIZE_MIN_PX so sub-pixel points don't vanish on the
// smallest lobes.
function setLobePointSize(obj: THREE.Object3D, factor: number): void {
  obj.traverse((child) => {
    const m = (child as THREE.Points).material;
    if (m && !Array.isArray(m) && 'size' in m && m.userData['baseSize'] != null) {
      const base = m.userData['baseSize'] as number;
      m.size = Math.max(LOBE_POINT_SIZE_MIN_PX, base * factor);
    }
  });
}
