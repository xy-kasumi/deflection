import * as THREE from 'three';
import type { BeamNode, Vec3 } from './walker';
import type { SimResult } from './sim/run';
import type { Directional } from './sim/directional';

const ISO_YAW_DEG = 45;
const ISO_PITCH_DEG = -30;

const COLOR_BG = 0xffffff;
const COLOR_JOINT = 0x444b53;
const COLOR_CLAMP = 0x444b53;
const COLOR_ATTACHMENT = 0xd97a1a;
const COLOR_WALKER_HINT_HIGHLIGHT = 0x2e7d32;
const COLOR_DEFORMED = 0xd97a1a;
const COLOR_HEADLINE_ARROW = 0xd62828;

const COLOR_BEAM = 0x9aa0a6;
const COLOR_BEAM_HIGHLIGHT = 0x2563eb;

// Drag tuning: time-constants of the input low-pass and post-release decay.
const SMOOTH_K = 18;
const DECAY_K = 16;
const STOP_VEL = 0.1;
const YAW_PER_PX = 1 / 150;

const deg = (d: number) => (d * Math.PI) / 180;

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
  // World-frame extent used for camera framing. Recomputed on update().
  private center: THREE.Vector3 = new THREE.Vector3(0, 0, 0);
  private scaleHalf = 100;

  constructor(canvas: HTMLCanvasElement) {
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

    const ro = new ResizeObserver(() => this.onResize());
    ro.observe(canvas);

    this.installDrag(canvas);
  }

  update(
    beams: BeamNode[],
    sim?: SimResult,
    supportKind?: 'single' | 'both',
    editor?: { currentBeamIx: number | null; focused: boolean },
  ): void {
    disposeChildren(this.content);

    if (beams.length === 0) {
      this.center.set(0, 0, 0);
      this.scaleHalf = 100;
      this.refresh();
      return;
    }

    // Determine a length scale (average beam length) for visual sizing.
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
    // Walker hint is offset along both walker-up (so the section orientation
    // is visible) and walker-fwd (so it's unambiguously associated with this
    // beam, not the parent's end joint).
    const walkerHintOffset = avgL / 8;
    const showWalkerHints = editor?.focused === true;

    const bbox = new THREE.Box3();
    bbox.makeEmpty();

    for (let i = 0; i < beams.length; i++) {
      const b = beams[i]!;
      const start = new THREE.Vector3(...b.startFrame.origin);
      const fwd = new THREE.Vector3(...b.startFrame.fwd);
      const end = start.clone().add(fwd.clone().multiplyScalar(b.length_mm));
      const isCurrent = showWalkerHints && editor!.currentBeamIx === i;

      // Capsule = cylinder + hemispherical end caps in one geometry. `length`
      // is the cylinder portion; trim it by 2r so the visual span (caps
      // included) matches b.length_mm exactly. Local axis is +Y; rotate to
      // align with beam +fwd, then translate.
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

      // Non-root beams: sphere at the parent-attachment point (this beam's start).
      if (i > 0) {
        const joint = new THREE.Mesh(jointGeom, jointMat);
        joint.position.copy(start);
        this.content.add(joint);
      }

      // Walker hint: sphere offset along walker-up at the beam's start, with
      // a faint foot dropping orthogonally to the centerline. Surfaces the
      // section-frame orientation and visually anchors to this beam (not the
      // parent joint). Suppressed entirely when the editor isn't focused;
      // green on the editor's current beam, grey otherwise.
      if (showWalkerHints) {
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
      }

      // Attachment markers along the beam axis.
      for (const att of b.attachmentOffsets) {
        const pos = start.clone().add(fwd.clone().multiplyScalar(att.local_mm));
        const dot = new THREE.Mesh(attachGeom, attachMat);
        dot.position.copy(pos);
        this.content.add(dot);
      }

      bbox.expandByPoint(start);
      bbox.expandByPoint(end);
    }

    // Clamp markers on the root beam: start always; end under support(both).
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

    if (sim && sim.nodes.length > 0) {
      this.drawDeformedOverlay(beams, sim, maxDim, attachRadius);
    }

    this.refresh();
  }

  private drawDeformedOverlay(
    _beams: BeamNode[],
    sim: SimResult,
    maxDim: number,
    _attachRadius: number,
  ): void {
    const headline = sim.nodes[sim.tipNodeIx];
    if (!headline) return;

    const k = sim.display_scale;

    for (const n of sim.nodes) {
      if (!(n.delta_max_mm > 0)) continue;
      const surface = buildDirectionalSurface(n.directional, k);
      surface.position.set(...n.worldPos_undeformed);
      this.content.add(surface);
    }

    const headlinePos = new THREE.Vector3(...headline.worldPos_undeformed);
    const d = new THREE.Vector3(...headline.d_star);
    const len = headline.delta_max_mm * k;
    if (len > 1e-6) {
      const arrow = new THREE.ArrowHelper(
        d.clone().normalize(),
        headlinePos,
        len,
        COLOR_HEADLINE_ARROW,
        Math.min(len * 0.4, maxDim * 0.05),
        Math.min(len * 0.25, maxDim * 0.03),
      );
      this.content.add(arrow);
    }
  }

  private installDrag(canvas: HTMLCanvasElement) {
    canvas.addEventListener('pointerdown', (e) => {
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

    const end = () => {
      if (!this.dragging) return;
      this.dragging = false;
      canvas.classList.remove('dragging');
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
  }

  private updateCamera() {
    const target = this.center;
    const elev = -this.pitch;
    const ce = Math.cos(elev);
    const se = Math.sin(elev);
    const sy = Math.sin(this.yaw);
    const cy = Math.cos(this.yaw);
    const r = 4 * this.scaleHalf;
    // Y-up world orbit: yaw around +Y, elev tilts toward +Y.
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
function buildDirectionalSurface(dir: Directional, scale: number): THREE.Mesh {
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
    opacity: 0.5,
  });
  return new THREE.Mesh(geom, mat);
}

// Keep this export so callers using ...spread-style construction can pass a Vec3.
export type { Vec3 };
