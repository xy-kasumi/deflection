import * as THREE from 'three';
import type { BeamNode, Vec3 } from './walker';
import type { SimResult } from './sim/run';
import type { Directional } from './sim/directional';
import { formatMm } from './readout';

const ISO_YAW_DEG = 45;
const ISO_PITCH_DEG = -30;

const COLOR_BG = 0xffffff;
const COLOR_JOINT = 0x444b53;
const COLOR_CLAMP = 0x444b53;
const COLOR_ATTACHMENT = 0xd97a1a;
const COLOR_WALKER_HINT_HIGHLIGHT = 0x2e7d32;
const COLOR_DEFORMED = 0xd97a1a;          // orange — deflection family (shell + underflow)
const COLOR_DEFORMED_PEAK = 0xc62828;     // red — peak cap on the normal lobe + overflow

const COLOR_BEAM = 0x9aa0a6;
const COLOR_BEAM_HIGHLIGHT = 0x2563eb;

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
  vec3 shellCol = vec3(0.85, 0.48, 0.10);
  vec3 capCol   = vec3(0.78, 0.16, 0.16);
  vec3 col = mix(shellCol, capCol, capAlpha);
  float a = mix(${LOBE_SHELL_ALPHA}, 1.0, capAlpha) * alphaMul;
  gl_FragColor = vec4(col, a);
}
`;

// Click vs drag: pointerup with movement below this threshold (squared, px) is a click.
const CLICK_MOVE_THRESH_SQ = 16;

// Drag tuning: time-constants of the input low-pass and post-release decay.
const SMOOTH_K = 18;
const DECAY_K = 16;
const STOP_VEL = 0.1;
const YAW_PER_PX = 1 / 150;

const deg = (d: number) => (d * Math.PI) / 180;

interface LabelEntry {
  el: HTMLDivElement;
  worldPos: THREE.Vector3;
}

export class Scene {
  private renderer: THREE.WebGLRenderer;
  private root: THREE.Scene;
  private camera: THREE.OrthographicCamera;
  private content: THREE.Group;
  private yaw: number;
  private pitch: number;
  private targetYaw: number;
  private yawVelocity = 0;
  private dragging = false;
  private lastPointerX = 0;
  private lastFrameTime = 0;
  private animHandle = 0;
  private center: THREE.Vector3 = new THREE.Vector3(0, 0, 0);
  private scaleHalf = 100;

  private onPick: (nodeIx: number) => void;
  private pickables: THREE.Mesh[] = [];
  private raycaster = new THREE.Raycaster();
  private labelLayer: HTMLDivElement;
  private labelEntries: LabelEntry[] = [];

  constructor(canvas: HTMLCanvasElement, onPick: (nodeIx: number) => void) {
    this.onPick = onPick;

    this.renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
    this.renderer.setPixelRatio(window.devicePixelRatio);
    this.root = new THREE.Scene();
    this.root.background = new THREE.Color(COLOR_BG);
    this.camera = new THREE.OrthographicCamera(-1, 1, 1, -1, -10000, 10000);
    this.content = new THREE.Group();
    this.root.add(this.content);
    this.yaw = deg(ISO_YAW_DEG);
    this.pitch = deg(ISO_PITCH_DEG);
    this.targetYaw = this.yaw;

    // Take focus on canvas interaction so CodeMirror's `focusChanged` event
    // fires — that's what flips the scene into inspection mode.
    canvas.tabIndex = -1;
    canvas.style.outline = 'none';

    this.labelLayer = document.createElement('div');
    this.labelLayer.className = 'scene-labels';
    canvas.parentElement!.appendChild(this.labelLayer);

    const ro = new ResizeObserver(() => this.onResize());
    ro.observe(canvas);

    this.installDrag(canvas);
  }

  update(
    beams: BeamNode[],
    sim: SimResult | undefined,
    supportKind: 'single' | 'both' | undefined,
    editor: { currentBeamIx: number | null; focused: boolean } | undefined,
    selectedNodeIx: number,
  ): void {
    disposeChildren(this.content);
    this.pickables = [];
    this.clearLabels();

    if (beams.length === 0) {
      this.center.set(0, 0, 0);
      this.scaleHalf = 100;
      this.refresh();
      return;
    }

    const focused = editor?.focused === true;

    // Visual unit `u` = rod diameter (see vocab.md). Everything visible is
    // expressed as `u × k`. Variables below are radii (Three.js geometry
    // constructors take radii), so their numerical multipliers equal half
    // the diameter ratio; each comment states the relationship in diameters,
    // the directly-visible quantity. `avgL/40` is the only place avgL touches
    // geometry; the floor keeps glyphs visible on degenerately short chains.
    const avgL = beams.reduce((s, b) => s + b.length_mm, 0) / beams.length;
    const u = Math.max(1, avgL / 40);

    const rodR    = u / 2;          // rod dia = 1u
    const jointR  = u;              // joint dia = 2u — twice as fat as the rod
    const attachR = u * 0.75;       // attach dia = 1.5u — smaller marker for loads
    const clampHalf = u * 1.1;      // clamp side = 2.2u — just wider than the joint
    const hitRadius = u * 3;        // hit dia = 6u — generous, also reaches the label

    const walkerHintOffset = u * 5; // 5u off the rod
    const labelOffset      = u * 5;

    const lobeFloor = u * 0.75;     // underflow dia = 1.5u — just edges past the rod
    const lobeCeil  = u * 7;        // overflow base dia = 14u — well under one beam length (40u)

    const jointGeom = new THREE.SphereGeometry(jointR, 12, 8);
    const clampGeom = new THREE.BoxGeometry(clampHalf * 2, clampHalf * 2, clampHalf * 2);
    const attachGeom = new THREE.SphereGeometry(attachR, 10, 6);
    const jointMat = new THREE.MeshBasicMaterial({ color: COLOR_JOINT });
    const clampMat = new THREE.MeshBasicMaterial({ color: COLOR_CLAMP });
    const attachMat = new THREE.MeshBasicMaterial({ color: COLOR_ATTACHMENT });

    const bbox = new THREE.Box3();
    bbox.makeEmpty();

    for (let i = 0; i < beams.length; i++) {
      const b = beams[i]!;
      const start = new THREE.Vector3(...b.startFrame.origin);
      const fwd = new THREE.Vector3(...b.startFrame.fwd);
      const end = start.clone().add(fwd.clone().multiplyScalar(b.length_mm));
      const isCurrent = focused && editor!.currentBeamIx === i;

      // Capsule = cylinder + hemispherical end caps in one geometry. Trim the
      // cylinder portion by 2r so the visual span (caps included) matches
      // b.length_mm exactly. Local axis is +Y; rotate to align with beam +fwd.
      const cylPart = Math.max(rodR * 0.01, b.length_mm - 2 * rodR);
      const geom = new THREE.CapsuleGeometry(rodR, cylPart, 6, 16);
      geom.translate(0, b.length_mm / 2, 0);
      const beamMat = new THREE.MeshBasicMaterial({
        color: isCurrent ? COLOR_BEAM_HIGHLIGHT : COLOR_BEAM,
        transparent: true,
        opacity: 0.32,
        depthWrite: false,
      });
      const mesh = new THREE.Mesh(geom, beamMat);
      mesh.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), fwd);
      mesh.position.copy(start);
      this.content.add(mesh);

      // Structural decorations (joints, loads, walker hints) only in editing mode.
      // Inspection mode keeps just the beam skeleton + clamps + deflection visuals.
      if (focused) {
        // Non-root beams: sphere at the parent-attachment point (this beam's start).
        if (i > 0) {
          const joint = new THREE.Mesh(jointGeom, jointMat);
          joint.position.copy(start);
          this.content.add(joint);
        }

        // Walker hint: sphere offset along walker-up at the beam's start with a
        // faint foot dropping orthogonally to the centerline. Surfaces the
        // section-frame orientation. Green on the editor's current beam.
        const hintColor = isCurrent ? COLOR_WALKER_HINT_HIGHLIGHT : COLOR_BEAM;
        const upVec = new THREE.Vector3(...b.startFrame.up);
        const fwdOffset = b.length_mm * 0.08;
        const walkerHintGeom = new THREE.SphereGeometry(attachR * 0.9, 10, 6);
        const walkerHint = new THREE.Mesh(
          walkerHintGeom,
          new THREE.MeshBasicMaterial({ color: hintColor }),
        );
        walkerHint.position
          .copy(start)
          .add(fwd.clone().multiplyScalar(fwdOffset))
          .add(upVec.clone().multiplyScalar(walkerHintOffset));
        this.content.add(walkerHint);

        const footAnchor = start.clone().add(fwd.clone().multiplyScalar(fwdOffset));
        const footGeom = new THREE.BufferGeometry().setFromPoints([
          footAnchor,
          walkerHint.position.clone(),
        ]);
        const footMat = new THREE.LineBasicMaterial({
          color: hintColor,
          transparent: true,
          opacity: 0.35,
          depthWrite: false,
        });
        this.content.add(new THREE.Line(footGeom, footMat));

        // Attachment markers along the beam axis (load locations).
        for (const att of b.attachmentOffsets) {
          const pos = start.clone().add(fwd.clone().multiplyScalar(att.local_mm));
          const dot = new THREE.Mesh(attachGeom, attachMat);
          dot.position.copy(pos);
          this.content.add(dot);
        }
      }

      bbox.expandByPoint(start);
      bbox.expandByPoint(end);
    }

    // Clamp markers on the root beam: start always; end under support(both).
    // These stay visible in both modes — they denote the world-origin reference.
    if (supportKind) {
      const root = beams[0]!;
      const rStart = new THREE.Vector3(...root.startFrame.origin);
      const rFwd = new THREE.Vector3(...root.startFrame.fwd);
      const startClamp = new THREE.Mesh(clampGeom, clampMat);
      startClamp.position.copy(rStart);
      this.content.add(startClamp);
      if (supportKind === 'both') {
        const endClamp = new THREE.Mesh(clampGeom, clampMat);
        endClamp.position.copy(rStart).add(rFwd.multiplyScalar(root.length_mm));
        this.content.add(endClamp);
      }
    }

    bbox.getCenter(this.center);
    const size = bbox.getSize(new THREE.Vector3());
    const maxDim = Math.max(size.x, size.y, size.z);
    this.scaleHalf = Math.max(50, maxDim * 0.8 + Math.max(u * 6, 10));

    if (sim && sim.nodes.length > 0) {
      this.drawDeflection(beams, sim, hitRadius, labelOffset, lobeFloor, lobeCeil, selectedNodeIx, focused);
    }

    this.refresh();
  }

  private drawDeflection(
    beams: BeamNode[],
    sim: SimResult,
    hitRadius: number,
    labelOffset: number,
    lobeFloor: number,
    lobeCeil: number,
    selectedNodeIx: number,
    focused: boolean,
  ): void {

    for (let ix = 0; ix < sim.nodes.length; ix++) {
      const n = sim.nodes[ix]!;
      const isSel = ix === selectedNodeIx;
      const worldPos = new THREE.Vector3(...n.worldPos_undeformed);

      const outerMax = n.delta_max_mm * sim.display_scale;
      let state: 'underflow' | 'normal' | 'overflow';
      if (outerMax < lobeFloor) state = 'underflow';
      else if (outerMax > lobeCeil) state = 'overflow';
      else state = 'normal';

      const opacity = focused
        ? 0.15
        : isSel
          ? 0.75
          : state === 'normal' ? 0.25 : 0.45;

      let visual: THREE.Object3D;
      if (state === 'normal') {
        visual = buildNormalLobe(n.directional, sim.display_scale, opacity);
      } else if (state === 'underflow') {
        visual = buildUnderflowSphere(lobeFloor, opacity);
      } else {
        visual = buildKonpeito(lobeCeil, opacity);
      }
      visual.position.copy(worldPos);
      this.content.add(visual);

      // Invisible hit-test sphere — generous, fixed radius. visible:false skips
      // rendering; intersectObjects(pickables, false) still raycasts it.
      const hit = new THREE.Mesh(
        new THREE.SphereGeometry(hitRadius, 8, 6),
        new THREE.MeshBasicMaterial({ visible: false }),
      );
      hit.visible = false;
      hit.position.copy(worldPos);
      hit.userData['nodeIx'] = ix;
      this.content.add(hit);
      this.pickables.push(hit);

      // Floating δ label, offset along the beam's walker-up so the label sits
      // off the lobe instead of behind it. Hidden when editor has focus.
      // Clicking the label picks the same node as clicking the lobe.
      const labelEl = document.createElement('div');
      labelEl.className = 'scene-label';
      if (focused) labelEl.classList.add('hidden');
      if (isSel) labelEl.classList.add('selected');
      labelEl.textContent = `δ ${formatMm(n.delta_max_mm)}`;
      const pickIx = ix;
      labelEl.addEventListener('click', () => this.onPick(pickIx));
      this.labelLayer.appendChild(labelEl);
      const up = new THREE.Vector3(...beams[n.beamIx]!.startFrame.up);
      const labelPos = worldPos.clone().add(up.multiplyScalar(labelOffset));
      this.labelEntries.push({ el: labelEl, worldPos: labelPos });
    }

  }

  private clearLabels() {
    for (const e of this.labelEntries) e.el.remove();
    this.labelEntries = [];
  }

  private updateLabels() {
    if (this.labelEntries.length === 0) return;
    const rect = this.renderer.domElement.getBoundingClientRect();
    const w = rect.width;
    const h = rect.height;
    const v = new THREE.Vector3();
    for (const e of this.labelEntries) {
      v.copy(e.worldPos).project(this.camera);
      const x = (v.x * 0.5 + 0.5) * w;
      const y = (-v.y * 0.5 + 0.5) * h;
      e.el.style.transform = `translate(-50%, -50%) translate(${x}px, ${y}px)`;
    }
  }

  private tryPick(clientX: number, clientY: number): void {
    if (this.pickables.length === 0) return;
    const rect = this.renderer.domElement.getBoundingClientRect();
    const ndc = new THREE.Vector2(
       ((clientX - rect.left) / rect.width)  * 2 - 1,
      -((clientY - rect.top)  / rect.height) * 2 + 1,
    );
    this.content.updateMatrixWorld(true);
    this.raycaster.setFromCamera(ndc, this.camera);
    const hits = this.raycaster.intersectObjects(this.pickables, false);
    if (hits.length > 0) {
      const nodeIx = hits[0]!.object.userData['nodeIx'] as number;
      this.onPick(nodeIx);
    }
  }

  private installDrag(canvas: HTMLCanvasElement) {
    let downX = 0;
    let downY = 0;

    canvas.addEventListener('pointerdown', (e) => {
      downX = e.clientX;
      downY = e.clientY;
      canvas.focus();
      this.dragging = true;
      this.lastPointerX = e.clientX;
      this.yawVelocity = 0;
      this.targetYaw = this.yaw;
      canvas.setPointerCapture(e.pointerId);
      canvas.classList.add('dragging');
      this.startAnim();
    });

    canvas.addEventListener('pointermove', (e) => {
      if (!this.dragging) return;
      const dx = e.clientX - this.lastPointerX;
      this.lastPointerX = e.clientX;
      this.targetYaw -= dx * YAW_PER_PX;
    });

    const end = (e: PointerEvent) => {
      if (!this.dragging) return;
      this.dragging = false;
      canvas.classList.remove('dragging');
      const dx = e.clientX - downX;
      const dy = e.clientY - downY;
      if (dx * dx + dy * dy < CLICK_MOVE_THRESH_SQ) {
        this.tryPick(e.clientX, e.clientY);
      }
    };
    canvas.addEventListener('pointerup', end);
    canvas.addEventListener('pointercancel', end);
  }

  private startAnim() {
    if (this.animHandle) return;
    this.lastFrameTime = performance.now();
    const tick = (now: number) => {
      const dt = Math.min(0.05, (now - this.lastFrameTime) / 1000);
      this.lastFrameTime = now;

      if (this.dragging) {
        const prevYaw = this.yaw;
        const alpha = 1 - Math.exp(-dt * SMOOTH_K);
        this.yaw += (this.targetYaw - this.yaw) * alpha;
        this.yawVelocity = (this.yaw - prevYaw) / Math.max(dt, 1e-3);
      } else {
        this.yaw += this.yawVelocity * dt;
        this.targetYaw = this.yaw;
        this.yawVelocity *= Math.exp(-dt * DECAY_K);
        if (Math.abs(this.yawVelocity) < STOP_VEL) this.yawVelocity = 0;
      }

      this.refresh();

      const settled = !this.dragging
        && this.yawVelocity === 0
        && Math.abs(this.targetYaw - this.yaw) < 1e-4;
      if (settled) {
        this.animHandle = 0;
        return;
      }
      this.animHandle = requestAnimationFrame(tick);
    };
    this.animHandle = requestAnimationFrame(tick);
  }

  private refresh() {
    this.updateCamera();
    this.renderer.render(this.root, this.camera);
    this.updateLabels();
  }

  private updateCamera() {
    const target = this.center;
    const elev = -this.pitch;
    const ce = Math.cos(elev);
    const se = Math.sin(elev);
    const sy = Math.sin(this.yaw);
    const cy = Math.cos(this.yaw);
    const r = 4 * this.scaleHalf;
    this.camera.position.set(
      target.x + r * ce * sy,
      target.y + r * se,
      target.z + r * ce * cy,
    );
    if (Math.abs(ce) < 0.01) {
      this.camera.up.set(sy, 0, cy);
    } else {
      this.camera.up.set(0, 1, 0);
    }
    this.camera.lookAt(target);

    const canvas = this.renderer.domElement;
    const w = canvas.clientWidth || 1;
    const h = canvas.clientHeight || 1;
    const aspect = w / h;
    const halfBase = this.scaleHalf;
    if (aspect >= 1) {
      this.camera.left = -halfBase * aspect;
      this.camera.right = halfBase * aspect;
      this.camera.top = halfBase;
      this.camera.bottom = -halfBase;
    } else {
      this.camera.left = -halfBase;
      this.camera.right = halfBase;
      this.camera.top = halfBase / aspect;
      this.camera.bottom = -halfBase / aspect;
    }
    this.camera.near = -100 * this.scaleHalf;
    this.camera.far = 100 * this.scaleHalf;
    this.camera.updateProjectionMatrix();
  }

  private onResize() {
    const canvas = this.renderer.domElement;
    const w = canvas.clientWidth;
    const h = canvas.clientHeight;
    if (w > 0 && h > 0) {
      this.renderer.setSize(w, h, false);
    }
    this.refresh();
  }
}

function disposeChildren(group: THREE.Group) {
  group.traverse((obj) => {
    const o = obj as THREE.Object3D & {
      geometry?: THREE.BufferGeometry;
      material?: THREE.Material | THREE.Material[];
    };
    if (o.geometry) o.geometry.dispose();
    if (o.material) {
      if (Array.isArray(o.material)) o.material.forEach((m) => m.dispose());
      else o.material.dispose();
    }
  });
  group.clear();
}

// Filled icosphere deformed radially by δ(d) · scale, painted by the lobe
// shader (faint orange shell + opaque red cap on δ/δ_max ≥ LOBE_CAP_LO). The
// cap topology emerges from the data — point on a sharp peak, band on a
// ridge, whole surface on an isotropic lobe.
//
// Truly isotropic lobes would paint as a uniformly red surface, which says
// nothing; we detect that case (most vertices already in the cap region) and
// degrade to a plain translucent shell — same look as the underflow sphere,
// just at the lobe's natural size.
function buildNormalLobe(dir: Directional, scale: number, opacity: number): THREE.Mesh {
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
    pos.setXYZ(i, nx * d * scale, ny * d * scale, nz * d * scale);
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
    const mat = new THREE.MeshBasicMaterial({
      color: COLOR_DEFORMED,
      transparent: true,
      opacity: LOBE_SHELL_ALPHA * opacity,
      depthWrite: false,
      side: THREE.DoubleSide,
    });
    return new THREE.Mesh(geom, mat);
  }

  geom.setAttribute('tNorm', new THREE.BufferAttribute(tNorm, 1));
  const mat = new THREE.ShaderMaterial({
    uniforms: { alphaMul: { value: opacity } },
    vertexShader: LOBE_VS,
    fragmentShader: LOBE_FS,
    transparent: true,
    side: THREE.DoubleSide,
    depthWrite: false,
  });
  return new THREE.Mesh(geom, mat);
}

// Solid orange sphere at the floor radius — "deflection here is too tight to
// draw a meaningful lobe; see the label for the number."
function buildUnderflowSphere(floor: number, opacity: number): THREE.Mesh {
  const geom = new THREE.SphereGeometry(floor, 12, 8);
  const mat = new THREE.MeshBasicMaterial({
    color: COLOR_DEFORMED,
    transparent: true,
    opacity,
    depthWrite: false,
  });
  return new THREE.Mesh(geom, mat);
}

// Konpeito-ish red spiky sphere at the ceiling radius — "deflection here would
// be off-scale; clipping for display, see the label for the real number."
// Cute angriness, not a screaming error.
function buildKonpeito(ceil: number, opacity: number): THREE.Mesh {
  const geom = new THREE.IcosahedronGeometry(ceil, 1);
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
    color: COLOR_DEFORMED_PEAK,
    transparent: true,
    opacity,
    depthWrite: false,
  });
  return new THREE.Mesh(geom, mat);
}

// Keep this export so callers using ...spread-style construction can pass a Vec3.
export type { Vec3 };
