import * as THREE from 'three';
import { OrbitControls } from '../vendor/OrbitControls.js';

function geometryWithCapacity(capacity) {
  const geometry = new THREE.BufferGeometry();
  geometry.setAttribute('position', new THREE.BufferAttribute(new Float32Array(capacity * 3), 3).setUsage(THREE.DynamicDrawUsage));
  geometry.setAttribute('color', new THREE.BufferAttribute(new Float32Array(capacity * 3), 3).setUsage(THREE.DynamicDrawUsage));
  geometry.setDrawRange(0, 0);
  return geometry;
}

const srgbToLinear = (v) => v <= 0.04045 ? v / 12.92 : ((v + 0.055) / 1.055) ** 2.4;
const rainbow = Array.from({ length: 256 }, (_, i) => {
  // RViz-style rainbow: low intensity blue, then cyan/green/yellow, high red.
  const h = (1 - i / 255) * 4;
  const channels = [Math.max(0, Math.min(1, 2 - h)), Math.max(0, Math.min(1, 2 - Math.abs(h - 2))), Math.max(0, Math.min(1, h - 2))];
  return channels.map(srgbToLinear);
});

function fillIntensityColors(intensities, count, colors, rgb = null, hasRgb = null) {
  const sorted = hasRgb ? intensities.slice(0, count).filter((_, i) => !hasRgb[i]).sort() : intensities.slice(0, count).sort();
  const n = sorted.length;
  const min = n ? sorted[0] : 0;
  const max = n ? sorted[n - 1] : 0;
  const quantile = (q) => {
    if (!n) return 0;
    const at = (n - 1) * q;
    const low = Math.floor(at), high = Math.ceil(at);
    return sorted[low] + (sorted[high] - sorted[low]) * (at - low);
  };
  const p5 = quantile(0.05), p95 = quantile(0.95);
  const span = p95 - p5;
  const constant = span <= Math.max(1e-6, Math.abs(p95) * 1e-6);
  for (let i = 0; i < count; i++) {
    const level = constant ? 0.5 : Math.min(1, Math.max(0, (intensities[i] - p5) / span));
    const color = rainbow[Math.round(level * 255)];
    for (let channel = 0; channel < 3; channel++) {
      colors[i * 3 + channel] = rgb && (!hasRgb || hasRgb[i])
        ? srgbToLinear(rgb[i * 3 + channel] / 255) : color[channel];
    }
  }
  return { min, max, p5, p95 };
}

export class PointCloudRenderer {
  constructor(canvas) {
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x0b1119);
    this.camera = new THREE.PerspectiveCamera(60, 1, 0.05, 5000);
    this.camera.position.set(8, 6, 8);
    this.renderer = new THREE.WebGLRenderer({ canvas, antialias: false, powerPreference: 'high-performance' });
    this.renderer.setPixelRatio(Math.min(window.devicePixelRatio || 1, 1.5));
    this.controls = new OrbitControls(this.camera, canvas);
    this.controls.enableDamping = true;
    this.controls.target.set(0, 0, 0);

    // ROS camera_init has Z up. Both point layers and the ROI box share this one transform.
    this.rosRoot = new THREE.Group();
    this.rosRoot.rotation.x = -Math.PI / 2;
    this.scene.add(this.rosRoot);
    this.axes = new THREE.AxesHelper(3);
    this.rosRoot.add(this.axes);
    this.grid = new THREE.GridHelper(20, 20, 0x516071, 0x273444);
    this.grid.rotation.x = Math.PI / 2;
    this.rosRoot.add(this.grid);

    this.roiUniforms = {
      uRoiEnabled: { value: false },
      uRoiMin: { value: new THREE.Vector3() },
      uRoiMax: { value: new THREE.Vector3() },
    };
    const withRoi = (material) => {
      material.onBeforeCompile = (shader) => {
        Object.assign(shader.uniforms, this.roiUniforms);
        shader.vertexShader = shader.vertexShader
          .replace('#include <common>', '#include <common>\nvarying vec3 vRosPoint;')
          .replace('#include <begin_vertex>', '#include <begin_vertex>\nvRosPoint = position;');
        shader.fragmentShader = shader.fragmentShader
          .replace('#include <common>', '#include <common>\nvarying vec3 vRosPoint;\nuniform bool uRoiEnabled;\nuniform vec3 uRoiMin;\nuniform vec3 uRoiMax;')
          .replace('#include <clipping_planes_fragment>', '#include <clipping_planes_fragment>\nif (uRoiEnabled && (any(lessThan(vRosPoint, uRoiMin)) || any(greaterThan(vRosPoint, uRoiMax)))) discard;');
      };
      return material;
    };
    this.colorMaterial = withRoi(new THREE.PointsMaterial({ size: 2, sizeAttenuation: false, vertexColors: true }));
    this.whiteMaterial = withRoi(new THREE.PointsMaterial({ size: 2, sizeAttenuation: false, color: 0xffffff }));
    this.historyMaterial = withRoi(new THREE.PointsMaterial({ size: 2, sizeAttenuation: false, vertexColors: true }));
    this.geometry = geometryWithCapacity(0);
    this.cloud = new THREE.Points(this.geometry, this.colorMaterial);
    this.cloud.frustumCulled = false;
    this.rosRoot.add(this.cloud);
    this.historyGeometry = geometryWithCapacity(0);
    this.historyCloud = new THREE.Points(this.historyGeometry, this.historyMaterial);
    this.historyCloud.frustumCulled = false;
    this.historyCloud.visible = false;
    this.rosRoot.add(this.historyCloud);
    this.roiBox = new THREE.Box3Helper(new THREE.Box3(new THREE.Vector3(), new THREE.Vector3()), 0x59d6ff);
    this.roiBox.visible = false;
    this.rosRoot.add(this.roiBox);
    this.capacity = this.historyCapacity = this.historyCount = 0;
    this.mode = 'auto';
    this.showCurrent = true;
    this.showHistory = false;
    this.lastBounds = null;
    this.hasFramed = false;
    this.resize();
  }

  resize() {
    const canvas = this.renderer.domElement;
    const width = Math.max(1, canvas.clientWidth);
    const height = Math.max(1, canvas.clientHeight);
    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(width, height, false);
  }

  setPointSize(value) {
    const size = Math.min(10, Math.max(1, Number(value) || 2));
    this.colorMaterial.size = this.whiteMaterial.size = this.historyMaterial.size = size;
  }

  updateCurrentMaterial() {
    this.cloud.material = this.mode === 'white' ? this.whiteMaterial : this.colorMaterial;
  }
  setMode(mode) { this.mode = mode === 'white' ? 'white' : 'auto'; this.updateCurrentMaterial(); }
  setShowCurrent(visible) { this.showCurrent = visible; this.cloud.visible = visible; }
  setShowHistory(visible) {
    this.showHistory = visible;
    this.historyCloud.visible = visible && this.historyCount > 0;
    this.updateCurrentMaterial();
  }
  setAxesVisible(visible) { this.axes.visible = visible; }
  setGridVisible(visible) { this.grid.visible = visible; }

  setRoi(enabled, bounds) {
    const active = Boolean(enabled && bounds);
    this.roiUniforms.uRoiEnabled.value = active;
    this.roiBox.visible = active;
    if (!active) return;
    this.roiUniforms.uRoiMin.value.fromArray(bounds.low);
    this.roiUniforms.uRoiMax.value.fromArray(bounds.high);
    this.roiBox.box.min.fromArray(bounds.low);
    this.roiBox.box.max.fromArray(bounds.high);
    this.roiBox.updateMatrixWorld(true);
  }

  ensureCapacity(count) {
    if (count <= this.capacity) return;
    const capacity = Math.max(count, this.capacity * 2, 1024);
    const next = geometryWithCapacity(capacity);
    this.cloud.geometry = next;
    this.geometry.dispose();
    this.geometry = next;
    this.capacity = capacity;
  }

  setFrame(frame) {
    this.ensureCapacity(frame.count);
    const positions = this.geometry.getAttribute('position');
    const colors = this.geometry.getAttribute('color');
    positions.array.set(frame.positions);
    positions.needsUpdate = true;
    const intensity = frame.rgb ? null : fillIntensityColors(frame.intensities, frame.count, colors.array);
    if (frame.rgb) for (let i = 0; i < frame.rgb.length; i++) colors.array[i] = srgbToLinear(frame.rgb[i] / 255);
    colors.needsUpdate = true;
    this.geometry.setDrawRange(0, frame.count);
    this.lastBounds = frame.bounds;
    if (!this.hasFramed && frame.count) {
      this.resetView();
      this.hasFramed = true;
    }
    return intensity;
  }

  ensureHistoryCapacity(count) {
    if (count <= this.historyCapacity) return;
    const capacity = Math.max(count, this.historyCapacity * 2, 4096);
    const next = geometryWithCapacity(capacity);
    this.historyCloud.geometry = next;
    this.historyGeometry.dispose();
    this.historyGeometry = next;
    this.historyCapacity = capacity;
  }

  setHistory(history) {
    this.ensureHistoryCapacity(history.count);
    const positions = this.historyGeometry.getAttribute('position');
    const colors = this.historyGeometry.getAttribute('color');
    positions.array.set(history.positions);
    positions.needsUpdate = true;
    fillIntensityColors(history.intensities, history.count, colors.array, history.rgb, history.hasRgb);
    colors.needsUpdate = true;
    this.historyGeometry.setDrawRange(0, history.count);
    this.historyCount = history.count;
    this.historyCloud.visible = this.showHistory && history.count > 0;
  }

  clearHistory() {
    const next = geometryWithCapacity(0);
    this.historyCloud.geometry = next;
    this.historyGeometry.dispose();
    this.historyGeometry = next;
    this.historyCapacity = this.historyCount = 0;
    this.historyCloud.visible = false;
  }

  resetView(bounds = this.lastBounds) {
    if (!bounds) {
      this.camera.position.set(8, 6, 8);
      this.controls.target.set(0, 0, 0);
    } else {
      const { low, high } = bounds;
      const center = new THREE.Vector3((low[0] + high[0]) / 2,
                                       (low[1] + high[1]) / 2,
                                       (low[2] + high[2]) / 2);
      this.rosRoot.updateMatrixWorld(true);
      this.rosRoot.localToWorld(center);
      const radius = Math.max(2, Math.hypot(high[0] - low[0], high[1] - low[1], high[2] - low[2]) / 2);
      this.controls.target.copy(center);
      this.camera.position.copy(center).add(new THREE.Vector3(1.6 * radius, 1.2 * radius, 1.6 * radius));
      this.camera.far = Math.max(5000, radius * 100);
      this.camera.updateProjectionMatrix();
    }
    this.controls.update();
  }

  draw() {
    this.controls.update();
    this.renderer.render(this.scene, this.camera);
  }

  get geometryCount() { return this.renderer.info.memory.geometries; }
  get bufferBytes() { return (this.capacity + this.historyCapacity) * 3 * 4 * 2; }

  dispose() {
    this.controls.dispose();
    this.geometry.dispose();
    this.historyGeometry.dispose();
    this.colorMaterial.dispose();
    this.whiteMaterial.dispose();
    this.historyMaterial.dispose();
    this.roiBox.geometry.dispose();
    this.roiBox.material.dispose();
    this.grid.geometry.dispose();
    this.grid.material.dispose();
    this.axes.geometry.dispose();
    this.axes.material.dispose();
    this.renderer.dispose();
  }
}
