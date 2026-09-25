/**
 * Renderer, camera, lights and the render loop. Knows nothing about engines.
 *
 * Scene units are metres, matching the physics exactly, so a 42 mm pipe is 0.042
 * units across. Keeping one coordinate system means the piston you see is at the
 * position the gas law is using, with no scale factor to get wrong.
 */

import * as THREE from 'three';
import { OrbitControls } from 'three/examples/jsm/controls/OrbitControls.js';

import { RoomEnvironment } from 'three/examples/jsm/environments/RoomEnvironment.js';

export type FrameHook = (dt: number, elapsed: number) => void;

export class Viewer {
  readonly scene = new THREE.Scene();
  readonly camera: THREE.PerspectiveCamera;
  readonly renderer: THREE.WebGLRenderer;
  readonly controls: OrbitControls;

  /**
   * Section plane for the cutaway. Materials that opt in get clipped so the
   * mechanism inside the block is visible; the pipe and moving parts do not.
   */
  readonly clipPlane = new THREE.Plane(new THREE.Vector3(0, 0, -1), 0.001);

  private readonly hooks: FrameHook[] = [];
  private running = false;
  private disposed = false;
  private lastFrameMs = 0;
  private elapsedS = 0;

  constructor(private readonly container: HTMLElement) {
    this.renderer = new THREE.WebGLRenderer({ antialias: true, alpha: false });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio, 2));
    this.renderer.shadowMap.enabled = true;
    this.renderer.shadowMap.type = THREE.PCFShadowMap;
    this.renderer.localClippingEnabled = true;
    this.renderer.toneMapping = THREE.ACESFilmicToneMapping;
    this.renderer.toneMappingExposure = 1.05;
    container.appendChild(this.renderer.domElement);

    this.scene.background = new THREE.Color(0x14171c);
    this.scene.fog = new THREE.Fog(0x14171c, 4, 12);

    this.camera = new THREE.PerspectiveCamera(42, 1, 0.01, 60);
    this.camera.position.set(0.85, 0.55, 1.15);

    this.controls = new OrbitControls(this.camera, this.renderer.domElement);
    this.controls.enableDamping = true;
    this.controls.dampingFactor = 0.08;
    this.controls.target.set(0.45, 0.2, 0);
    this.controls.minDistance = 0.25;
    this.controls.maxDistance = 6;
    // Keep the camera from flipping underneath, where there is nothing to see but the sump.
    this.controls.maxPolarAngle = Math.PI * 0.495;

    this.addEnvironment();
    this.addLights();

    this.resize();
    window.addEventListener('resize', this.resize);
  }

  /**
   * A prefiltered environment map. This is not a nicety: a `metalness: 0.9` surface
   * reflects almost nothing *but* the environment, so without one every polished
   * part — the pipe, the piston, the valves — renders nearly black no matter how
   * many lights are added.
   */
  private addEnvironment(): void {
    const pmrem = new THREE.PMREMGenerator(this.renderer);
    const env = pmrem.fromScene(new RoomEnvironment(), 0.04);
    this.scene.environment = env.texture;
    this.scene.environmentIntensity = 0.85;
    pmrem.dispose();
  }

  private addLights(): void {
    this.scene.add(new THREE.HemisphereLight(0x9fb4d6, 0x2a2118, 0.85));

    const key = new THREE.DirectionalLight(0xfff2e0, 2.1);
    key.position.set(1.2, 2.0, 1.4);
    key.castShadow = true;
    key.shadow.mapSize.set(2048, 2048);
    key.shadow.camera.near = 0.5;
    key.shadow.camera.far = 8;
    const d = 1.6;
    key.shadow.camera.left = -d;
    key.shadow.camera.right = d;
    key.shadow.camera.top = d;
    key.shadow.camera.bottom = -d;
    key.shadow.bias = -0.0008;
    this.scene.add(key);

    // Cool rim light from behind so the pipe reads against the dark background.
    const rim = new THREE.DirectionalLight(0x8fb8ff, 0.75);
    rim.position.set(-1.4, 0.7, -1.1);
    this.scene.add(rim);
  }

  /** Register a per-frame callback. */
  onFrame(hook: FrameHook): void {
    this.hooks.push(hook);
  }

  setCutaway(enabled: boolean): void {
    // Pushing the plane far away is cheaper than walking every material to toggle
    // its clippingPlanes array, and avoids shader recompiles.
    this.clipPlane.constant = enabled ? 0.001 : 100;
  }

  start(): void {
    if (this.running) return;
    this.running = true;
    this.lastFrameMs = performance.now();
    this.renderer.setAnimationLoop(() => this.frame());
  }

  private frame(): void {
    const now = performance.now();
    // Clamped so a backgrounded tab does not resume by jumping the crank a full
    // second forward.
    const dt = Math.min((now - this.lastFrameMs) / 1000, 0.1);
    this.lastFrameMs = now;
    this.elapsedS += dt;
    for (const hook of this.hooks) hook(dt, this.elapsedS);
    this.controls.update();
    this.renderer.render(this.scene, this.camera);
  }

  private resize = (): void => {
    if (this.disposed) return;
    const w = this.container.clientWidth || 1;
    const h = this.container.clientHeight || 1;
    this.camera.aspect = w / h;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(w, h, false);
  };

  /** Frames the camera on a bounding box, used after loading a long preset pipe. */
  frameBounds(box: THREE.Box3, padding = 1.1): void {
    if (box.isEmpty()) return;
    const centre = box.getCenter(new THREE.Vector3());
    const size = box.getSize(new THREE.Vector3());
    const radius = Math.max(size.length() / 2, 0.2) * padding;
    const fov = (this.camera.fov * Math.PI) / 180;
    const dist = radius / Math.sin(fov / 2);

    const dir = new THREE.Vector3(0.62, 0.42, 0.66).normalize();
    this.camera.position.copy(centre).addScaledVector(dir, dist);
    this.controls.target.copy(centre);
    this.controls.update();
  }

  dispose(): void {
    this.disposed = true;
    this.running = false;
    this.renderer.setAnimationLoop(null);
    window.removeEventListener('resize', this.resize);
    this.renderer.dispose();
    this.container.removeChild(this.renderer.domElement);
  }
}
