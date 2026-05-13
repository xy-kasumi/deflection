import * as THREE from 'three';
import type { BeamNode } from '../walker';
import type { SimResult } from '../sim/run';
import type { Directional } from '../sim/directional';
import { formatMm } from '../readout';
import { COLOR, MOTION, hexToVec3 } from './tokens';

// Normal-lobe rendering. Filled mesh with per-vertex t = δ/δ_max. The fragment
// shader paints a faint orange shell below t = CAP_LO and an opaque red cap
// above — the cap is a "point" on a sharp peak, a "band" on a ridge, the whole
// surface on a true sphere. fwidth-AA keeps the cap edge pixel-clean
// regardless of mesh density or lobe orientation.
const LOBE_CAP_LO = 0.975;
const LOBE_SHELL_ALPHA = 0.22;
// Sphere-detection: if more than this fraction of vertices sit inside the cap,
// the painted lobe would say nothing (whole surface red). Degrade to plain
// translucent shell instead — same look as the underflow sphere, full size.
const LOBE_SPHERE_FRACTION = 0.5;
// Icosphere subdivision. (detail+1)² sub-triangles per base face → 20·(detail+1)²
// triangles total. detail=20 → ~3.5° vertex spacing, smooth cap boundary.
const LOBE_DETAIL = 20;

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

const LOBE_VS = `
attribute float tNorm;
varying float vT;
void main() {
  vT = tNorm;
  gl_Position = projectionMatrix * modelViewMatrix * vec4(position, 1.0);
}
`;

const LOBE_FS = `
precision highp float;
uniform float alphaMul;
varying float vT;
void main() {
  float t = clamp(vT, 0.0, 1.0);
  float wT = fwidth(t);
  float capAlpha = smoothstep(${LOBE_CAP_LO} - wT, ${LOBE_CAP_LO} + wT, t);
  vec3 shellCol = ${hexToVec3(COLOR.deformed)};
  vec3 capCol   = ${hexToVec3(COLOR.deformedPeak)};
  vec3 col = mix(shellCol, capCol, capAlpha);
  float a = mix(${LOBE_SHELL_ALPHA}, 1.0, capAlpha) * alphaMul;
  gl_FragColor = vec4(col, a);
}
`;

// Per-query anim context. Built once per sim update; the per-frame tick
// reads delta_max_mm and the base opacities, then drives mesh.scale and
// material/uniform alpha to crossfade between the three states.
interface LobeAnim {
  delta_max_mm: number;
  // Opacity each mesh fades toward when it owns the visual; matches the
  // pre-animation per-state values (normal lobes are denser via shader
  // paint, so they sit at a lower base than the solid under/over hints).
  normalBase: number;
  underBase: number;
  overBase: number;
  normal: THREE.Mesh;
  under: THREE.Mesh;
  over: THREE.Mesh;
  // Normal lobe is either a ShaderMaterial (with alphaMul uniform) or the
  // isotropic-sphere fallback MeshBasicMaterial — different opacity paths.
  normalIsShader: boolean;
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

    for (let ix = 0; ix < sim.nodes.length; ix++) {
      const n = sim.nodes[ix]!;
      const isSel = ix === opts.selectedNodeIx;
      const worldPos = new THREE.Vector3(...n.worldPos_undeformed);

      // All three states are built up-front. The per-frame tick scales/fades
      // them according to the animated δ-exag; categorical state-switching is
      // replaced by a continuous crossfade. Base opacities preserve the prior
      // per-state look at each crossfade endpoint.
      const normal = buildNormalLobe(n.directional);
      normal.mesh.position.copy(worldPos);
      meshes.push(normal.mesh);

      const under = buildUnderflowSphere(opts.lobeFloor);
      under.position.copy(worldPos);
      meshes.push(under);

      const over = buildKonpeito();
      over.position.copy(worldPos);
      meshes.push(over);

      this.lobeAnims.push({
        delta_max_mm: n.delta_max_mm,
        normalBase: opts.focused ? 0.15 : isSel ? 0.75 : 0.25,
        underBase:  opts.focused ? 0.15 : isSel ? 0.75 : 0.45,
        overBase:   opts.focused ? 0.15 : isSel ? 0.75 : 0.45,
        normal: normal.mesh,
        under,
        over,
        normalIsShader: normal.isShader,
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
      const up = new THREE.Vector3(...beams[n.beamIx]!.startFrame.up);
      const labelPos = worldPos.clone().add(up.multiplyScalar(opts.labelOffset));
      const classes: string[] = [];
      if (opts.focused) classes.push('hidden');
      if (isSel) classes.push('selected');
      labels.push({
        text: `δ ${formatMm(n.delta_max_mm)}`,
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
      const alpha = 1 - Math.exp(-dt * MOTION.scaleK);
      this.displayScale_anim = Math.exp(logC + (logT - logC) * alpha);
      return false;
    }
    if (this.displayScale_anim !== this.displayScale_target) {
      this.displayScale_anim = this.displayScale_target;
    }
    return true;
  }

  // Per-frame crossfade between underflow / normal / overflow. No-op when no
  // lobes have been built. lobeCeilWorld is computed externally so this class
  // doesn't need to know about the canvas.
  apply(lobeCeilWorld: number): void {
    if (this.lobeAnims.length === 0) return;
    const floor = this.lobeFloor;
    const scale = this.displayScale_anim;

    for (const a of this.lobeAnims) {
      const outerMax = a.delta_max_mm * scale;
      const aUnder = 1 - smoothstep(LOBE_UNDER_BLEND_LO * floor, floor, outerMax);
      const aOver  =     smoothstep(LOBE_OVER_BLEND_LO * lobeCeilWorld, lobeCeilWorld, outerMax);

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

      setLobeOpacity(a.normal, a.normalIsShader, normalOpacity);
      setLobeOpacity(a.under, false, underOpacity);
      setLobeOpacity(a.over, false, overOpacity);
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

// Filled icosphere deformed radially by δ(d), painted by the lobe shader
// (faint orange shell + opaque red cap on δ/δ_max ≥ LOBE_CAP_LO). The cap
// topology emerges from the data — point on a sharp peak, band on a ridge,
// whole surface on an isotropic lobe.
//
// Built at unit δ-exag: vertex radial extent equals delta_max_mm in world
// units. The caller drives mesh.scale to apply the animated δ-exag, so the
// painted normalization stays valid under any scale.
//
// Truly isotropic lobes would paint as a uniformly red surface, which says
// nothing; we detect that case (most vertices already in the cap region) and
// degrade to a plain translucent shell — same look as the underflow sphere,
// just at the lobe's natural size. The returned isShader flag tells the
// per-frame tick which opacity path to use.
function buildNormalLobe(dir: Directional): { mesh: THREE.Mesh; isShader: boolean } {
  const geom = new THREE.IcosahedronGeometry(1, LOBE_DETAIL);
  const pos = geom.attributes.position!;
  const dvals = new Float32Array(pos.count);
  let dMax = 0;
  for (let i = 0; i < pos.count; i++) {
    const x = pos.getX(i);
    const y = pos.getY(i);
    const z = pos.getZ(i);
    const len = Math.hypot(x, y, z) || 1;
    const nx = x / len, ny = y / len, nz = z / len;
    const d = dir.at([nx, ny, nz]);
    dvals[i] = d;
    if (d > dMax) dMax = d;
    pos.setXYZ(i, nx * d, ny * d, nz * d);
  }
  pos.needsUpdate = true;
  geom.computeBoundingSphere();

  const tNorm = new Float32Array(pos.count);
  let capCount = 0;
  for (let i = 0; i < pos.count; i++) {
    const t = dMax > 0 ? dvals[i]! / dMax : 0;
    tNorm[i] = t;
    if (t >= LOBE_CAP_LO) capCount++;
  }

  if (capCount / pos.count > LOBE_SPHERE_FRACTION) {
    // Isotropic fallback. LOBE_SHELL_ALPHA encodes the same "shell tint" the
    // shader applies; the per-frame tick multiplies it through the opacity
    // crossfade by writing the material's opacity directly.
    const mat = new THREE.MeshBasicMaterial({
      color: COLOR.deformed,
      transparent: true,
      opacity: LOBE_SHELL_ALPHA,
      depthWrite: false,
      side: THREE.DoubleSide,
    });
    return { mesh: new THREE.Mesh(geom, mat), isShader: false };
  }

  geom.setAttribute('tNorm', new THREE.BufferAttribute(tNorm, 1));
  const mat = new THREE.ShaderMaterial({
    uniforms: { alphaMul: { value: 1 } },
    vertexShader: LOBE_VS,
    fragmentShader: LOBE_FS,
    transparent: true,
    side: THREE.DoubleSide,
    depthWrite: false,
  });
  return { mesh: new THREE.Mesh(geom, mat), isShader: true };
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

// Lobe meshes carry two flavors of material; route opacity to the right
// field and hide nearly-invisible meshes so the renderer can skip them.
function setLobeOpacity(mesh: THREE.Mesh, isShader: boolean, opacity: number): void {
  if (isShader) {
    (mesh.material as THREE.ShaderMaterial).uniforms['alphaMul']!.value = opacity;
  } else {
    (mesh.material as THREE.MeshBasicMaterial).opacity = opacity;
  }
  mesh.visible = opacity > 1e-3;
}
