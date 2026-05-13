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
const COLOR_DEFORMED = 0xd97a1a;          // orange — deflection family (lobe + underflow)
const COLOR_DEFORMED_OVERFLOW = 0xc62828; // red — clipped magnitude
const COLOR_HEADLINE_ARROW = 0xd62828;

const COLOR_BEAM = 0x9aa0a6;
const COLOR_BEAM_HIGHLIGHT = 0x2563eb;

// Magnitude bands for lobe rendering, relative to mean beam length `avgL`.
// Floor is roughly beamRadius * 1.5 (so the underflow sphere just edges out
// of the rod); ceiling caps the wireframe well under one beam length so the
// konpeito (vertex-bumped to ~1.4×) stays visually contained.
const LOBE_FLOOR_REL = 0.02;
const LOBE_CEIL_REL = 0.18;

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

    const avgL = beams.reduce((s, b) => s + b.length_mm, 0) / beams.length;
    const beamRadius = Math.max(0.5, avgL / 80);
    const jointRadius = Math.max(0.8, avgL / 40);
    const attachRadius = Math.max(0.6, avgL / 50);
    const clampHalf = jointRadius * 1.1;

    const jointGeom = new THREE.SphereGeometry(jointRadius, 12, 8);
    const clampGeom = new THREE.BoxGeometry(clampHalf * 2, clampHalf * 2, clampHalf * 2);
    const attachGeom = new THREE.SphereGeometry(attachRadius, 10, 6);
    const jointMat = new THREE.MeshBasicMaterial({ color: COLOR_JOINT });
    const clampMat = new THREE.MeshBasicMaterial({ color: COLOR_CLAMP });
    const attachMat = new THREE.MeshBasicMaterial({ color: COLOR_ATTACHMENT });
    const walkerHintOffset = avgL / 8;

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
      const cylPart = Math.max(beamRadius * 0.01, b.length_mm - 2 * beamRadius);
      const geom = new THREE.CapsuleGeometry(beamRadius, cylPart, 6, 16);
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
        const walkerHintGeom = new THREE.SphereGeometry(attachRadius * 0.9, 10, 6);
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
    this.scaleHalf = Math.max(50, maxDim * 0.8 + Math.max(beamRadius * 6, 10));

    const hitRadius = avgL / 25;
    const labelOffset = avgL / 8;
    if (sim && sim.nodes.length > 0) {
      this.drawDeflection(beams, sim, avgL, maxDim, hitRadius, labelOffset, selectedNodeIx, focused);
    }

    this.refresh();
  }

  private drawDeflection(
    beams: BeamNode[],
    sim: SimResult,
    avgL: number,
    maxDim: number,
    hitRadius: number,
    labelOffset: number,
    selectedNodeIx: number,
    focused: boolean,
  ): void {
    const floor = avgL * LOBE_FLOOR_REL;
    const ceil = avgL * LOBE_CEIL_REL;

    for (let ix = 0; ix < sim.nodes.length; ix++) {
      const n = sim.nodes[ix]!;
      const isSel = ix === selectedNodeIx;
      const worldPos = new THREE.Vector3(...n.worldPos_undeformed);

      const outerMax = n.delta_max_mm * sim.display_scale;
      let state: 'underflow' | 'normal' | 'overflow';
      if (outerMax < floor) state = 'underflow';
      else if (outerMax > ceil) state = 'overflow';
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
        visual = buildUnderflowSphere(floor, opacity);
      } else {
        visual = buildKonpeito(ceil, opacity);
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
      const labelEl = document.createElement('div');
      labelEl.className = 'scene-label';
      if (focused) labelEl.classList.add('hidden');
      if (isSel) labelEl.classList.add('selected');
      labelEl.textContent = `δ ${formatMm(n.delta_max_mm)}`;
      this.labelLayer.appendChild(labelEl);
      const up = new THREE.Vector3(...beams[n.beamIx]!.startFrame.up);
      const labelPos = worldPos.clone().add(up.multiplyScalar(labelOffset));
      this.labelEntries.push({ el: labelEl, worldPos: labelPos });
    }

    // Headline arrow at the selected node — only when not editing.
    const sel = sim.nodes[selectedNodeIx];
    if (!focused && sel && sel.delta_max_mm > 0) {
      const len = sel.delta_max_mm * sim.display_scale;
      if (len > 1e-6) {
        const d = new THREE.Vector3(...sel.d_star);
        const arrow = new THREE.ArrowHelper(
          d.clone().normalize(),
          new THREE.Vector3(...sel.worldPos_undeformed),
          len,
          COLOR_HEADLINE_ARROW,
          Math.min(len * 0.4, maxDim * 0.05),
          Math.min(len * 0.25, maxDim * 0.03),
        );
        this.content.add(arrow);
      }
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

// Wireframe icosphere whose vertices are deformed radially by δ(d) · scale.
// One mesh per query node visualizes how compliant the joint is in every
// direction: anisotropic chains produce elongated lobes along their weak axes.
function buildNormalLobe(dir: Directional, scale: number, opacity: number): THREE.Mesh {
  const geom = new THREE.IcosahedronGeometry(1, 3);
  const pos = geom.attributes.position!;
  for (let i = 0; i < pos.count; i++) {
    const x = pos.getX(i);
    const y = pos.getY(i);
    const z = pos.getZ(i);
    const len = Math.hypot(x, y, z) || 1;
    const nx = x / len, ny = y / len, nz = z / len;
    const r = dir.at([nx, ny, nz]) * scale;
    pos.setXYZ(i, nx * r, ny * r, nz * r);
  }
  pos.needsUpdate = true;
  geom.computeBoundingSphere();
  const mat = new THREE.MeshBasicMaterial({
    color: COLOR_DEFORMED,
    wireframe: true,
    transparent: true,
    opacity,
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
    color: COLOR_DEFORMED_OVERFLOW,
    transparent: true,
    opacity,
    depthWrite: false,
  });
  return new THREE.Mesh(geom, mat);
}

// Keep this export so callers using ...spread-style construction can pass a Vec3.
export type { Vec3 };
