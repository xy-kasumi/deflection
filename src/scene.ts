import * as THREE from 'three';
import type { BeamNode, Vec3 } from './walker';

const ISO_YAW_DEG = 45;
const ISO_PITCH_DEG = -30;

const COLOR_BG = 0xffffff;
const COLOR_AXIS = 0xd0d7de;
const COLOR_JOINT = 0x444b53;
const COLOR_ATTACHMENT = 0xd97a1a;
const COLOR_UP_MARKER = 0x2e7d32;

const MATERIAL_COLORS: Record<string, number> = {
  plastic: 0xc7b56b,
  aluminum: 0x8693a3,
  steel: 0x3b6db5,
};
const COLOR_BEAM_DEFAULT = 0x9aa0a6;

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
  private axes: THREE.Group;
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
    this.axes = new THREE.Group();
    this.root.add(this.content);
    this.root.add(this.axes);
    this.yaw = deg(ISO_YAW_DEG);
    this.pitch = deg(ISO_PITCH_DEG);
    this.targetYaw = this.yaw;

    const ro = new ResizeObserver(() => this.onResize());
    ro.observe(canvas);

    this.installDrag(canvas);
  }

  update(beams: BeamNode[]): void {
    disposeChildren(this.content);
    disposeChildren(this.axes);

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

    const jointGeom = new THREE.SphereGeometry(jointRadius, 12, 8);
    const attachGeom = new THREE.SphereGeometry(attachRadius, 10, 6);
    const upMarkerGeom = new THREE.SphereGeometry(attachRadius * 0.9, 10, 6);
    const jointMat = new THREE.MeshBasicMaterial({ color: COLOR_JOINT });
    const attachMat = new THREE.MeshBasicMaterial({ color: COLOR_ATTACHMENT });
    const upMat = new THREE.MeshBasicMaterial({ color: COLOR_UP_MARKER });
    const upLineMat = new THREE.LineBasicMaterial({
      color: COLOR_UP_MARKER,
      transparent: true,
      opacity: 0.35,
      depthWrite: false,
    });
    // Up-marker is offset away from the joint along both walker-up (so the
    // section orientation is visible) and walker-fwd (so it's unambiguously
    // associated with this beam, not the parent's end joint).
    const upOffset = avgL / 8;

    const bbox = new THREE.Box3();
    bbox.makeEmpty();

    for (const b of beams) {
      const matColor = b.material && MATERIAL_COLORS[b.material]
        ? MATERIAL_COLORS[b.material]
        : COLOR_BEAM_DEFAULT;

      const start = new THREE.Vector3(...b.startFrame.origin);
      const fwd = new THREE.Vector3(...b.startFrame.fwd);
      const end = start.clone().add(fwd.clone().multiplyScalar(b.length_mm));

      const cyl = new THREE.CylinderGeometry(beamRadius, beamRadius, b.length_mm, 16, 1, true);
      // Default cylinder is along +Y; rotate to align with beam +fwd, then translate.
      cyl.translate(0, b.length_mm / 2, 0);
      const beamMat = new THREE.MeshBasicMaterial({
        color: matColor,
        transparent: true,
        opacity: 0.32,
        depthWrite: false,
      });
      const mesh = new THREE.Mesh(cyl, beamMat);
      mesh.quaternion.setFromUnitVectors(new THREE.Vector3(0, 1, 0), fwd);
      mesh.position.copy(start);
      this.content.add(mesh);

      // Joint dots at start and end.
      const startDot = new THREE.Mesh(jointGeom, jointMat);
      startDot.position.copy(start);
      this.content.add(startDot);
      const endDot = new THREE.Mesh(jointGeom, jointMat);
      endDot.position.copy(end);
      this.content.add(endDot);

      // Local-up marker: small green sphere offset along walker-up at the
      // beam's start. Makes the section-frame orientation visible and
      // assigns visual ownership to this beam (not the parent joint).
      // A faint line tethers it to the start joint to make the link explicit.
      const upVec = new THREE.Vector3(...b.startFrame.up);
      const fwdOffset = b.length_mm * 0.08;
      const upMarker = new THREE.Mesh(upMarkerGeom, upMat);
      upMarker.position
        .copy(start)
        .add(fwd.clone().multiplyScalar(fwdOffset))
        .add(upVec.clone().multiplyScalar(upOffset));
      this.content.add(upMarker);

      // Tether is orthogonal to the beam: from a point on the centerline at
      // the marker's fwd-offset, straight along walker-up to the marker.
      const tetherFoot = start.clone().add(fwd.clone().multiplyScalar(fwdOffset));
      const tetherGeom = new THREE.BufferGeometry().setFromPoints([
        tetherFoot,
        upMarker.position.clone(),
      ]);
      this.content.add(new THREE.Line(tetherGeom, upLineMat));

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

    bbox.getCenter(this.center);
    const size = bbox.getSize(new THREE.Vector3());
    const maxDim = Math.max(size.x, size.y, size.z);
    this.scaleHalf = Math.max(50, maxDim * 0.8 + Math.max(beamRadius * 6, 10));

    this.rebuildAxes(bbox);
    this.refresh();
  }

  private rebuildAxes(bbox: THREE.Box3): void {
    const mat = new THREE.LineBasicMaterial({ color: COLOR_AXIS });
    const min = bbox.min;
    const max = bbox.max;
    const pad = Math.max(10, (max.x - min.x) * 0.1);
    const positions = new Float32Array([
      min.x - pad, 0, 0,
      max.x + pad, 0, 0,
    ]);
    const geom = new THREE.BufferGeometry();
    geom.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    this.axes.add(new THREE.Line(geom, mat));
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
    this.camera.position.set(
      target.x + r * ce * sy,
      target.y - r * ce * cy,
      target.z + r * se,
    );
    if (Math.abs(ce) < 0.01) {
      this.camera.up.set(-sy, cy, 0);
    } else {
      this.camera.up.set(0, 0, 1);
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

// Keep this export so callers using ...spread-style construction can pass a Vec3.
export type { Vec3 };
