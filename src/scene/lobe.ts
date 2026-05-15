import * as THREE from 'three';
import type { BeamNode, Vec3 } from '../walker';
import type { SimResult } from '../sim/simulate';
import { delta, type Directional, type DeltaTerm } from '../sim/directional';
import { formatMm, displayDelta, type DisplayMode } from '../breakdown';
import { COLOR, MOTION, hexToVec3 } from './tokens';

// Normal-lobe rendering. Filled mesh with per-vertex t = δ/δ_max. The fragment
// shader paints a faint orange shell below t = CAP_LO and an opaque red cap
// above — the cap is a "point" on a sharp peak, a "band" on a ridge, the whole
// surface on a true sphere. fwidth-AA keeps the cap edge pixel-clean
// regardless of mesh density or lobe orientation.
const LOBE_CAP_LO = 0.975;
const LOBE_SHELL_ALPHA = 0.22;
// Icosphere subdivision. (detail+1)² sub-triangles per base face → 20·(detail+1)²
// triangles total. detail=20 → ~3.5° vertex spacing, smooth cap boundary.
const LOBE_DETAIL = 20;

// Sum-of-ellipsoids cap for the shader. WebGL minimums easily accommodate this
// many mat3 uniforms; bumping if a future scene needs more is a one-line edit.
const MAX_LOADS = 16;

// Per-vertex CPU spot-check: K randomly chosen vertices carry a CPU-computed
// δ, the shader writes 1.0 into vDiscrep when its own δ differs by more than
// SPOTCHECK_TOL_REL relative. Fragment shader paints those vertices solid
// magenta — bright halos = shader/CPU disagree, see [[delta]] in directional.ts.
//
// K random (not strided) so the icosphere's 20-fold symmetry can't accidentally
// align with sample placement. Density target: ~14° mean spacing on S² — dense
// enough that a wrong-shape region of ~half a steradian lights multiple halos.
const K_SPOTCHECK = 384;
const SPOTCHECK_TOL_REL = 0.01;

// Sphere-fallback threshold — δ_min/δ_max above this and the lobe is nearly
// isotropic. The shader, told via uniform, skips cap-painting and renders a
// uniform shell. Computed CPU-side from `directional.sample()`.
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

// Vertex shader — GLSL TWIN of `delta(d_unit, terms)` in src/sim/directional.ts.
// Must stay in sync; the cpuDelta attribute carries CPU truth at K_SPOTCHECK
// random vertices and the fragment shader halo-paints any per-vertex divergence.
//
// Mesh is positioned at the node's world origin with identity rotation, so
// `position` (mesh-local) equals the world-frame direction d. The shader
// scales position radially by δ(d); modelViewMatrix translates the deformed
// vertex to its world location.
const LOBE_VS = `
precision highp float;
uniform mat3 Ns[${MAX_LOADS}];
uniform float Fs[${MAX_LOADS}];
uniform int nLoads;
uniform float dMaxInv;
attribute float cpuDelta;
varying float vT;
varying float vDiscrep;
void main() {
  vec3 d = normalize(position);
  float deltaSh = 0.0;
  for (int i = 0; i < ${MAX_LOADS}; i++) {
    if (i >= nLoads) break;
    vec3 v = Ns[i] * d;
    deltaSh += Fs[i] * length(v);
  }
  vT = deltaSh * dMaxInv;
  vDiscrep = 0.0;
  if (cpuDelta >= 0.0) {
    float rel = abs(deltaSh - cpuDelta) / max(deltaSh, 1e-6);
    if (rel > ${SPOTCHECK_TOL_REL}) vDiscrep = 1.0;
  }
  vec3 deformed = d * deltaSh;
  gl_Position = projectionMatrix * modelViewMatrix * vec4(deformed, 1.0);
}
`;

const LOBE_FS = `
precision highp float;
uniform float alphaMul;
uniform float isIsotropic;
varying float vT;
varying float vDiscrep;
void main() {
  if (vDiscrep > 0.5) {
    // CPU/GPU δ disagree at a spot-check vertex — emit a magenta halo (the
    // varying interpolates over the surrounding triangle).
    gl_FragColor = vec4(1.0, 0.0, 1.0, 1.0);
    return;
  }
  float t = clamp(vT, 0.0, 1.0);
  float wT = fwidth(t);
  float capAlpha = smoothstep(${LOBE_CAP_LO} - wT, ${LOBE_CAP_LO} + wT, t);
  capAlpha *= (1.0 - isIsotropic);
  vec3 shellCol = ${hexToVec3(COLOR.deformed)};
  vec3 capCol   = ${hexToVec3(COLOR.deformedPeak)};
  vec3 col = mix(shellCol, capCol, capAlpha);
  float a = mix(${LOBE_SHELL_ALPHA}, 1.0, capAlpha) * alphaMul;
  gl_FragColor = vec4(col, a);
}
`;

// Unit icosphere built once. Positions are exactly unit-length so the vertex
// shader's `normalize(position)` is structurally redundant — kept defensive
// against future floating-point drift. The BufferAttribute is shared across
// every node's per-lobe BufferGeometry: position never changes, so the GL
// buffer is uploaded once per page load.
const SHARED_POS_ATTR: THREE.BufferAttribute = (() => {
  const g = new THREE.IcosahedronGeometry(1, LOBE_DETAIL);
  const pos = g.attributes.position as THREE.BufferAttribute;
  for (let i = 0; i < pos.count; i++) {
    const x = pos.getX(i), y = pos.getY(i), z = pos.getZ(i);
    const len = Math.hypot(x, y, z) || 1;
    pos.setXYZ(i, x / len, y / len, z / len);
  }
  return pos;
})();

// K random vertex slots flagged as CPU spot-check samples. LCG-based so the
// pattern is deterministic across loads but bears no relation to the
// icosphere's 20-fold symmetry (a strided sampler could).
const SPOTCHECK_INDICES: number[] = (() => {
  const n = SHARED_POS_ATTR.count;
  const out: number[] = [];
  const seen = new Set<number>();
  let s = 1 >>> 0;
  while (out.length < K_SPOTCHECK && out.length < n) {
    s = (Math.imul(s, 1664525) + 1013904223) >>> 0;
    const idx = s % n;
    if (!seen.has(idx)) {
      seen.add(idx);
      out.push(idx);
    }
  }
  return out;
})();

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
}

export interface LobeBuildOpts {
  hitRadius: number;
  labelOffset: number;
  lobeFloor: number;
  selectedNodeIx: number;
  focused: boolean;
  mode: DisplayMode;
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
  // Per-node ShaderMaterial pool. Reusing across rebuilds (just swap uniform
  // values) avoids the ~7ms first-draw cost Three.js pays for each fresh
  // ShaderMaterial instance — uniform-location lookup, attribute binding, and
  // program acquisition for the user-defined shader.
  private normalMatPool: THREE.ShaderMaterial[] = [];

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

    for (let ix = 0; ix < sim.queryResults.length; ix++) {
      const n = sim.queryResults[ix]!;
      const isSel = ix === opts.selectedNodeIx;
      const worldPos = new THREE.Vector3(...n.pos_mm);

      // Scalar mode's δ bound is direction-independent (its "lobe" would be a
      // sphere); skip building lobe meshes entirely. Hit-sphere and label are
      // still built below so node picking and the floating δ readout work.
      let delta_max_mm: number;
      if (opts.mode === 'scalar') {
        delta_max_mm = displayDelta(n, 'scalar');
      } else {
        const lobeDir = n.deflection;
        delta_max_mm = lobeDir.max().value;

        // All three states are built up-front. The per-frame tick scales/fades
        // them according to the animated δ-exag; categorical state-switching is
        // replaced by a continuous crossfade. Base opacities preserve the prior
        // per-state look at each crossfade endpoint.
        const reuseMat = this.normalMatPool[ix];
        const normal = buildNormalLobe(lobeDir, reuseMat);
        if (!reuseMat) this.normalMatPool[ix] = normal.material as THREE.ShaderMaterial;
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
      }

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
      if (opts.focused) classes.push('hidden');
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

      setLobeOpacity(a.normal, true, normalOpacity);
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

// Filled icosphere deformed radially by δ(d) in the vertex shader. The
// fragment shader paints a faint orange shell below t = LOBE_CAP_LO and an
// opaque red cap above — cap topology emerges from the data (point on a sharp
// peak, band on a ridge), and fwidth-AA keeps its edge pixel-clean.
//
// Geometry: shares SHARED_POS_ATTR (one upload per page) plus a per-lobe
// cpuDelta attribute carrying CPU truth at K_SPOTCHECK random vertices. Any
// shader-CPU disagreement halo-paints those vertices magenta — see the LOBE_VS
// header for the contract with delta() in directional.ts.
//
// Built at unit δ-exag: the shader's radial deformation equals δ in mm. The
// caller drives mesh.scale to apply the animated δ-exag.
//
// Isotropic case: when δ_min/δ_max > LOBE_ISOTROPIC_RATIO across a coarse
// directional sample, the fragment shader's isIsotropic uniform suppresses
// cap painting and the lobe renders as a uniform shell — same affordance as
// the underflow sphere, just at the lobe's natural size.
function buildNormalLobe(dir: Directional, reuseMat?: THREE.ShaderMaterial): THREE.Mesh {
  const terms = dir.termsForGpuCompute(MAX_LOADS);
  const nLoads = Math.min(terms.length, MAX_LOADS);
  const dMax = dir.max().value;
  const dMaxInv = dMax > 0 ? 1 / dMax : 0;

  let dMin = dMax;
  for (const s of dir.sample(24)) if (s.value < dMin) dMin = s.value;
  const isIsotropic = dMax > 0 && dMin / dMax > LOBE_ISOTROPIC_RATIO;

  const geom = new THREE.BufferGeometry();
  geom.setAttribute('position', SHARED_POS_ATTR);
  const cpu = new Float32Array(SHARED_POS_ATTR.count);
  cpu.fill(-1);
  for (const idx of SPOTCHECK_INDICES) {
    const d: Vec3 = [
      SHARED_POS_ATTR.getX(idx),
      SHARED_POS_ATTR.getY(idx),
      SHARED_POS_ATTR.getZ(idx),
    ];
    cpu[idx] = delta(d, terms);
  }
  geom.setAttribute('cpuDelta', new THREE.BufferAttribute(cpu, 1));
  // Positions are unit-radius; shader scales radially by δ ≤ dMax. Use dMax
  // for frustum culling so the (shader-deformed) extent stays correct.
  geom.boundingSphere = new THREE.Sphere(new THREE.Vector3(0, 0, 0), dMax || 1);

  // Pack {N, F} into uniform arrays. THREE.Matrix3.set takes row-major args,
  // matching how Mat3 is laid out in directional.ts; the GLSL `Ns[i] * d`
  // multiply then gives N·d in the same sense as the CPU's matVec(N, d).
  let mat: THREE.ShaderMaterial;
  if (reuseMat) {
    mat = reuseMat;
    const NsArr = mat.uniforms['Ns']!.value as THREE.Matrix3[];
    const FsArr = mat.uniforms['Fs']!.value as number[];
    for (let i = 0; i < MAX_LOADS; i++) {
      if (i < nLoads) {
        const M = terms[i]!.N;
        NsArr[i]!.set(M[0], M[1], M[2], M[3], M[4], M[5], M[6], M[7], M[8]);
        FsArr[i] = terms[i]!.F;
      } else {
        NsArr[i]!.identity();
        FsArr[i] = 0;
      }
    }
    mat.uniforms['nLoads']!.value = nLoads;
    mat.uniforms['dMaxInv']!.value = dMaxInv;
    mat.uniforms['isIsotropic']!.value = isIsotropic ? 1 : 0;
  } else {
    const NsUniform: THREE.Matrix3[] = [];
    const FsUniform: number[] = [];
    for (let i = 0; i < MAX_LOADS; i++) {
      const m = new THREE.Matrix3();
      if (i < nLoads) {
        const M = terms[i]!.N;
        m.set(M[0], M[1], M[2], M[3], M[4], M[5], M[6], M[7], M[8]);
        FsUniform.push(terms[i]!.F);
      } else {
        m.identity();
        FsUniform.push(0);
      }
      NsUniform.push(m);
    }
    mat = new THREE.ShaderMaterial({
      uniforms: {
        Ns: { value: NsUniform },
        Fs: { value: FsUniform },
        nLoads: { value: nLoads },
        dMaxInv: { value: dMaxInv },
        alphaMul: { value: 1 },
        isIsotropic: { value: isIsotropic ? 1 : 0 },
      },
      vertexShader: LOBE_VS,
      fragmentShader: LOBE_FS,
      transparent: true,
      side: THREE.DoubleSide,
      depthWrite: false,
    });
  }
  return new THREE.Mesh(geom, mat);
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
