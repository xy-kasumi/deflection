import * as THREE from 'three';
import type { BeamNode, Vec3 } from '../walker';
import type { SimResult } from '../sim/simulate';
import type { ConvexEnvelope } from '../sim/math';
import { formatMm } from '../breakdown';
import { COLOR, MOTION, easeToward } from './tokens';

// Normal-lobe rendering. Sample N directions d on S² via Fibonacci spiral,
// compute boundary points bdy(d) = Σ Mᵢ · normalize(Mᵢᵀd) CPU-side, and
// render the surface as a THREE.Points cloud colored by t = |bdy|/δ_max.
// Shell points (low t) get faint orange; cap points (t≈1) get opaque red.
//
// Why points, not a mesh: the boundary map can be highly non-uniform — for an
// oblate spheroid the equator stretches tangentially while shrinking radially,
// producing needle-shaped triangles near the max-ring that slice through one
// another in 3D and cause transparency blend-order artifacts. Worse, for
// rank-deficient envelopes (a pancake from a single in-plane load) the map
// outright collapses S² onto a curve and no triangulation parameterized by d
// can recover. Uniform d ∈ S² renders dense where the lobe is fat (rims, peaks)
// and sparse where it is thin — the cap-as-ring on a pancake, cap-as-tips on
// a needle, cap-as-point on a peak all emerge for free from the sample density.
const LOBE_CAP_LO = 0.975;
const LOBE_SHELL_ALPHA = 0.22;

// Total samples per lobe. One bdy evaluation per sample per sim update; cost
// is N · n_loads · ~30 flops — negligible at sim-update cadence.
const LOBE_POINT_COUNT = 10000;
// Fixed on-screen point size in pixels. sizeAttenuation:false keeps points
// at the same pixel size regardless of camera zoom or mesh.scale, so density
// stays predictable across the displayScale animation and viewport changes.
// (PointsMaterial.size is NOT multiplied by mesh.scale when sizeAttenuation
// is true, so the world-units path requires per-frame size updates to track
// the lobe — fixed pixels is the simpler contract.)
const LOBE_POINT_SIZE_PX = 3;
// Cap points sized larger so the cap region reads as a solid colored band
// against the sparser shell, even where samples land far apart.
const LOBE_CAP_POINT_BOOST = 1.3;

// Sphere-fallback threshold — δ_min/δ_max above this and the lobe is nearly
// isotropic. All points get the shell color in that case (no cap painting),
// matching the "uniform shell" affordance used for the underflow sphere.
const LOBE_ISOTROPIC_RATIO = LOBE_CAP_LO;

// Overflow trigger: the konpeito (lobe-too-big) state begins when the lobe
// would occupy this fraction of the canvas viewport area. Picked from the
// 5–10% band; mid value keeps lobes readable without dominating the scene.
const LOBE_CEIL_AREA_FRAC = 0.075;
// Smoothstep windows around the floor/ceil transitions. Lower bounds are
// expressed relative to the threshold; the upper bound is the threshold
// itself. Past the threshold the new state is fully on.
const LOBE_UNDER_BLEND_LO = 0.7;
const LOBE_OVER_BLEND_LO  = 0.85;
// Konpeito is a compact hint, not the "real" lobe size — keep it small enough
// to not occlude nearby beams or other lobes. The trigger and the normal-lobe
// clamp both stay at lobeCeilWorld; only the rendered konpeito is halved.
const LOBE_KONPEITO_FRAC  = 0.5;

// The δ-exag button set; also the candidate set for auto-pick on node
// selection. Kept ascending so the recommend-scale loop can pick the largest
// non-overflown by iterating once.
const DISPLAY_SCALES = [1, 10, 100, 1000] as const;

// Shared radial-falloff alpha mask for all lobe Points materials. Sampled by
// gl_PointCoord in Three.js's built-in points fragment shader, it turns each
// square GL point into a soft round splat — the texture's rgb is solid white
// (multiplied by the material's color) and its alpha falls smoothly from 1 at
// the center to 0 at the rim. Without it, points render as hard squares.
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

// Fibonacci spiral on S². Deterministic, near-uniform, no clustering or
// singularities — well-suited to direction sampling for a support-function
// surface. Returns N unit Vec3s.
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

// Per-query anim context. Built once per sim update; the per-frame tick
// reads delta_max_mm and the base opacities, then drives scale and material
// opacity to crossfade between the three states.
interface LobeAnim {
  delta_max_mm: number;
  // Opacity each object fades toward when it owns the visual; matches the
  // pre-animation per-state values (normal-lobe shell points are denser via
  // SHELL_ALPHA so they sit at a lower base than the solid under/over hints).
  normalBase: number;
  underBase: number;
  overBase: number;
  // Group containing two THREE.Points (shell + cap). Object3D-typed so we
  // can scale/fade uniformly without caring about the internals.
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
  private displayScale_target = 1;
  private displayScale_anim = 1;

  getTarget(): number {
    return this.displayScale_target;
  }

  // Animated δ-exag — shared with StickRenderer so sticks scale in lockstep
  // with the lobe they decompose.
  getDisplayScale(): number {
    return this.displayScale_anim;
  }

  // Returns true if the target changed (caller may want to kick the anim
  // loop). δ-exag is animated to settle in log-space.
  setTarget(s: number): boolean {
    if (s === this.displayScale_target) return false;
    this.displayScale_target = s;
    return true;
  }

  // Largest scale from the button set that keeps the lobe at or below the
  // overflow threshold. Returns the smallest available scale when every
  // option overflows (i.e. ×1 is the best we can offer). Zero / negative
  // delta is degenerate — pick the largest so the choice is unambiguous.
  recommendDisplayScale(delta_max_mm: number, lobeCeilWorld: number): number {
    if (delta_max_mm <= 0) return DISPLAY_SCALES[DISPLAY_SCALES.length - 1]!;
    let best: number = DISPLAY_SCALES[0]!;
    for (const s of DISPLAY_SCALES) {
      if (delta_max_mm * s <= lobeCeilWorld) best = s;
    }
    return best;
  }

  // Build the three-state lobe widget at each query node: normal, underflow,
  // konpeito, plus an invisible hit-test sphere and a label spec. Caller
  // wires the meshes into the scene graph, the hits into its pickable list,
  // and the label specs into its overlay.
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

      // All three states are built up-front. The per-frame tick scales/fades
      // them according to the animated δ-exag; categorical state-switching is
      // replaced by a continuous crossfade. Base opacities preserve the prior
      // per-state look at each crossfade endpoint.
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

      // Invisible hit-test sphere — generous, fixed radius. visible:false skips
      // rendering; intersectObjects(pickables, false) still raycasts it.
      const hit = new THREE.Mesh(
        new THREE.SphereGeometry(opts.hitRadius, 8, 6),
        new THREE.MeshBasicMaterial({ visible: false }),
      );
      hit.visible = false;
      hit.position.copy(worldPos);
      hit.userData['nodeIx'] = ix;
      meshes.push(hit);
      pickables.push(hit);

      // Floating δ label, offset along the beam's walker-up so the label sits
      // off the lobe instead of behind it.
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

  // Log-space ease toward the target δ-exag. Geometric steps (×1→×10→×100)
  // feel like a uniform "zoom rate" instead of an exponential blast. Returns
  // true if the animation has settled this tick.
  tickScale(dt: number): boolean {
    const logT = Math.log(this.displayScale_target);
    const logC = Math.log(this.displayScale_anim);
    if (Math.abs(logT - logC) > MOTION.scaleSettleLog) {
      this.displayScale_anim = Math.exp(easeToward(logC, logT, dt, MOTION.scaleK));
      return false;
    }
    if (this.displayScale_anim !== this.displayScale_target) {
      this.displayScale_anim = this.displayScale_target;
    }
    return true;
  }

  // Per-frame crossfade between underflow / normal / overflow. lobeCeilWorld
  // is computed externally so this class doesn't need to know about the canvas.
  apply(lobeCeilWorld: number): void {
    const scale = this.displayScale_anim;
    if (this.lobeAnims.length === 0) return;
    const floor = this.lobeFloor;
    // User's mental model is discrete: {underflow, normal, overflow}. The
    // crossfade weights only exist to smooth the in-flight scale animation;
    // when the scale has settled, snap to the dominant state so the steady
    // image shows exactly one of the three.
    const settled = scale === this.displayScale_target;

    for (const a of this.lobeAnims) {
      const outerMax = a.delta_max_mm * scale;
      let aUnder = 1 - smoothstep(LOBE_UNDER_BLEND_LO * floor, floor, outerMax);
      let aOver  =     smoothstep(LOBE_OVER_BLEND_LO * lobeCeilWorld, lobeCeilWorld, outerMax);
      if (settled) {
        if (aOver >= 0.5)        { aOver = 1; aUnder = 0; }
        else if (aUnder >= 0.5)  { aUnder = 1; aOver = 0; }
        else                     { aOver = 0; aUnder = 0; }
      }

      // Clamp the normal lobe so it never visibly outgrows the konpeito while
      // they crossfade. delta_max_mm = 0 falls under aUnder=1, so the normal
      // mesh is invisible anyway; the clamp would divide by zero so skip it.
      const normalScale = a.delta_max_mm > 0
        ? Math.min(scale, lobeCeilWorld / a.delta_max_mm)
        : scale;
      a.normal.scale.setScalar(normalScale);
      a.over.scale.setScalar(lobeCeilWorld * LOBE_KONPEITO_FRAC);
      // under mesh built at world radius lobeFloor; no per-frame scaling.

      const normalOpacity = a.normalBase * (1 - aUnder) * (1 - aOver);
      const underOpacity  = a.underBase  * aUnder;
      const overOpacity   = a.overBase   * aOver;

      setLobeOpacity(a.normal, normalOpacity);
      setLobeOpacity(a.under, underOpacity);
      setLobeOpacity(a.over, overOpacity);
    }
  }
}

// Canvas-area-relative overflow radius. The konpeito kicks in when the lobe
// would project to ≥ LOBE_CEIL_AREA_FRAC of the viewport area. Recomputed
// each frame so window resize and ortho-zoom both stay correct.
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

// Fibonacci-sampled point cloud of ∂K. Returned as a Group containing two
// THREE.Points: shell (low t) and cap (t ≥ LOBE_CAP_LO). Two materials lets
// us keep stock PointsMaterial — base opacities differ (shell faint, cap
// solid), and the apply()-driven crossfade multiplies them uniformly through
// userData.baseOpacity.
//
// Built at native δ-exag: point positions equal bdy(d) in mm. The caller
// drives the wrapping Group's scale to apply the animated δ-exag.
//
// Isotropic case: when δ_min/δ_max > LOBE_ISOTROPIC_RATIO across a coarse
// directional sweep, no cap is emitted — the whole cloud is shell, the same
// affordance as the underflow sphere just at the lobe's natural size.
function buildNormalPoints(dir: ConvexEnvelope): THREE.Object3D {
  const dMax = dir.furthest().distance;
  const samples = fibonacciS2(LOBE_POINT_COUNT);

  // Coarse sub-sweep of the support function to decide isotropy. Skips the
  // bdy work (just |Mᵀd| sums per direction).
  let dMin = dMax;
  const N_ISO = 24;
  const stride = Math.max(1, Math.floor(LOBE_POINT_COUNT / N_ISO));
  for (let i = 0; i < LOBE_POINT_COUNT; i += stride) {
    const v = dir.support(samples[i]!);
    if (v < dMin) dMin = v;
  }
  const isIsotropic = dMax > 0 && dMin / dMax > LOBE_ISOTROPIC_RATIO;

  // Classify each sample into shell or cap by t = |bdy|/dMax. Build positions
  // in mm space; the wrapping Group's scale handles displayScale.
  const shellPos: number[] = [];
  const capPos: number[] = [];
  const dMaxInv = dMax > 0 ? 1 / dMax : 0;
  for (const d of samples) {
    const b = dir.boundary(d);
    const t = Math.hypot(b[0], b[1], b[2]) * dMaxInv;
    const target = !isIsotropic && t >= LOBE_CAP_LO ? capPos : shellPos;
    target.push(b[0], b[1], b[2]);
  }

  const group = new THREE.Group();

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
      opacity: 1,
      depthWrite: false,
    });
    mat.userData['baseOpacity'] = 1;
    group.add(new THREE.Points(geom, mat));
  }

  return group;
}

// Solid orange sphere at the floor radius — "deflection here is too tight to
// draw a meaningful lobe; see the label for the number."
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

// Konpeito-ish red spiky sphere — "deflection here would be off-scale;
// clipping for display, see the label for the real number." Cute angriness,
// not a screaming error. Built at unit radius; caller scales to the canvas-
// area-derived ceiling each frame.
function buildKonpeito(): THREE.Mesh {
  const geom = new THREE.IcosahedronGeometry(1, 1);
  const pos = geom.attributes.position!;
  for (let i = 0; i < pos.count; i++) {
    const x = pos.getX(i);
    const y = pos.getY(i);
    const z = pos.getZ(i);
    // Deterministic per-vertex scalar in [1.0, 1.4] via sin-hash.
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

// Smoothstep on [edge0, edge1], clamped to [0, 1]. Hermite-interpolated so
// the crossfade has zero derivative at the edges.
function smoothstep(edge0: number, edge1: number, x: number): number {
  const t = Math.max(0, Math.min(1, (x - edge0) / Math.max(1e-9, edge1 - edge0)));
  return t * t * (3 - 2 * t);
}

// Walk an object's materials and multiply each one's baseOpacity (1 if unset)
// by the requested fade. Hides nearly-invisible objects so the renderer can
// skip them entirely.
function setLobeOpacity(obj: THREE.Object3D, opacity: number): void {
  obj.traverse((child) => {
    const m = (child as THREE.Mesh | THREE.Points).material;
    if (m && !Array.isArray(m) && 'opacity' in m) {
      const base = (m.userData['baseOpacity'] ?? 1) as number;
      m.opacity = base * opacity;
    }
  });
  obj.visible = opacity > 1e-3;
}
