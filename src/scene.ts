import * as THREE from 'three';
import type { BeamState } from './state';

export type ViewPreset = 'front' | 'side' | 'top' | 'iso';

const PRESETS: Record<ViewPreset, { yawDeg: number; pitchDeg: number }> = {
  front: { yawDeg:  0, pitchDeg:   0 },
  side:  { yawDeg: 90, pitchDeg:   0 },
  top:   { yawDeg:  0, pitchDeg: -90 },
  iso:   { yawDeg: 45, pitchDeg: -30 },
};

const COLOR_BG = 0xffffff;
const COLOR_BEAM = 0x3b6db5;
const COLOR_DEFLECTION = 0xd97a1a;
const COLOR_FORCE = 0xc43d3d;
const COLOR_SUPPORT = 0x7a8593;
const COLOR_AXIS = 0xd0d7de;

const deg = (d: number) => (d * Math.PI) / 180;

export class Scene {
  private renderer: THREE.WebGLRenderer;
  private root: THREE.Scene;
  private camera: THREE.OrthographicCamera;
  private content: THREE.Group;
  private axes: THREE.Group;
  private currentL: number;
  private yaw: number;
  private pitch: number;

  constructor(canvas: HTMLCanvasElement, initialL: number) {
    this.renderer = new THREE.WebGLRenderer({ canvas, antialias: true });
    this.renderer.setPixelRatio(window.devicePixelRatio);
    this.root = new THREE.Scene();
    this.root.background = new THREE.Color(COLOR_BG);
    this.camera = new THREE.OrthographicCamera(-1, 1, 1, -1, -10000, 10000);
    this.content = new THREE.Group();
    this.axes = new THREE.Group();
    this.root.add(this.content);
    this.root.add(this.axes);
    this.currentL = initialL;
    this.yaw = deg(PRESETS.iso.yawDeg);
    this.pitch = deg(PRESETS.iso.pitchDeg);

    const ro = new ResizeObserver(() => this.onResize());
    ro.observe(canvas);
  }

  setView(p: ViewPreset) {
    this.yaw = deg(PRESETS[p].yawDeg);
    this.pitch = deg(PRESETS[p].pitchDeg);
    this.refresh();
  }

  nudgeYawDeg(d: number) {
    this.yaw += deg(d);
    this.refresh();
  }

  update(s: BeamState, peakMm: number, displayScale: number) {
    this.currentL = s.L_mm;
    this.rebuildContent(s, peakMm, displayScale);
    this.rebuildAxes(s.L_mm);
    this.refresh();
  }

  private rebuildContent(s: BeamState, peakMm: number, displayScale: number) {
    disposeChildren(this.content);
    const L = s.L_mm;
    const r = L / 80;

    // Undeformed beam: thin translucent cylinder along +X from x=0 to x=L.
    const beamGeom = new THREE.CylinderGeometry(r, r, L, 24);
    beamGeom.rotateZ(Math.PI / 2);
    beamGeom.translate(L / 2, 0, 0);
    const beamMat = new THREE.MeshBasicMaterial({
      color: COLOR_BEAM,
      transparent: true,
      opacity: 0.32,
      depthWrite: false,
    });
    this.content.add(new THREE.Mesh(beamGeom, beamMat));

    // Supports.
    const supportMat = new THREE.MeshBasicMaterial({ color: COLOR_SUPPORT });
    if (s.beamType === 'cantilever') {
      const wall = new THREE.Mesh(
        new THREE.BoxGeometry(L / 40, L / 3, L / 3),
        supportMat,
      );
      wall.position.set(-L / 80, 0, 0);
      this.content.add(wall);
    } else {
      const pinH = L / 15;
      const pinR = L / 28;
      const pinGeom = new THREE.ConeGeometry(pinR, pinH, 4);
      pinGeom.rotateX(Math.PI / 2);       // apex from +Y to +Z
      pinGeom.translate(0, 0, -pinH / 2); // apex at z=0, base at z=-pinH
      const pin1 = new THREE.Mesh(pinGeom, supportMat);
      const pin2 = new THREE.Mesh(pinGeom, supportMat);
      pin2.position.x = L;
      this.content.add(pin1, pin2);
    }

    // Load point and exaggerated deflection.
    const loadX = s.beamType === 'cantilever' ? L : L / 2;
    const peakAbs = Math.max(0, peakMm);
    const displayDelta = peakAbs * displayScale; // mm in scene
    const renderDeflection = peakAbs > 0 && displayDelta > L * 0.003;
    const renderForce = Math.abs(s.force_kgf) > 0;

    if (renderDeflection) {
      const headLen = Math.min(displayDelta * 0.3, L / 18);
      const headWid = headLen * 0.5;
      const arrow = new THREE.ArrowHelper(
        new THREE.Vector3(0, 0, -1),
        new THREE.Vector3(loadX, 0, 0),
        displayDelta,
        COLOR_DEFLECTION,
        headLen,
        headWid,
      );
      this.content.add(arrow);
    }

    if (renderForce) {
      // Force arrow lives above the *undeformed* beam at the load point and
      // points down to it. The deflection arrow continues downward from there.
      // Stacking the two avoids the overlap that you get when both terminate
      // at the deflected tip.
      const forceLen = L / 4;
      const tail = new THREE.Vector3(loadX, 0, forceLen);
      const arrow = new THREE.ArrowHelper(
        new THREE.Vector3(0, 0, -1),
        tail,
        forceLen,
        COLOR_FORCE,
        forceLen * 0.22,
        forceLen * 0.11,
      );
      this.content.add(arrow);
    }
  }

  private rebuildAxes(L: number) {
    disposeChildren(this.axes);
    // Simple ground grid line + origin tick to give the view a reference frame.
    const mat = new THREE.LineBasicMaterial({ color: COLOR_AXIS });
    const positions = new Float32Array([
      -L * 0.1, 0, 0,
       L * 1.1, 0, 0,
    ]);
    const geom = new THREE.BufferGeometry();
    geom.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    this.axes.add(new THREE.Line(geom, mat));
  }

  private refresh() {
    this.updateCamera();
    this.renderer.render(this.root, this.camera);
  }

  private updateCamera() {
    const L = this.currentL;
    const target = new THREE.Vector3(L / 2, 0, 0);
    const elev = -this.pitch; // pitch=-π/2  =>  elev=π/2 (top)
    const ce = Math.cos(elev);
    const se = Math.sin(elev);
    const sy = Math.sin(this.yaw);
    const cy = Math.cos(this.yaw);
    const r = 2 * L;
    this.camera.position.set(
      target.x + r * ce * sy,
      target.y - r * ce * cy,
      target.z + r * se,
    );
    if (Math.abs(ce) < 0.01) {
      // gimbal-lock fallback for top / bottom view
      this.camera.up.set(-sy, cy, 0);
    } else {
      this.camera.up.set(0, 0, 1);
    }
    this.camera.lookAt(target);

    const canvas = this.renderer.domElement;
    const w = canvas.clientWidth || 1;
    const h = canvas.clientHeight || 1;
    const aspect = w / h;
    const halfBase = L * 0.6;
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
    this.camera.near = -10 * L;
    this.camera.far = 10 * L;
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
