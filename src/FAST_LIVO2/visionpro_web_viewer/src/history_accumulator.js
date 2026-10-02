export class HistoryAccumulator {
  constructor(voxelSize = 0.01, maxPoints = 1000000) {
    if (!Number.isSafeInteger(maxPoints) || maxPoints < 1) throw new Error('Invalid history limit');
    this.maxPoints = maxPoints;
    this.setVoxelSize(voxelSize);
  }

  setVoxelSize(size) {
    if (!Number.isFinite(size) || size <= 0) throw new Error('Voxel size must be finite and positive');
    this.voxelSize = size;
    this.clear();
  }

  clear() {
    // ponytail: string keys keep voxel identity simple; the point cap bounds their cost. A packed integer key is the upgrade path for larger maps.
    this.voxels = new Map();
    this.positions = new Float32Array(0);
    this.intensities = new Float32Array(0);
    this.rgb = new Uint8Array(0);
    this.hasRgb = new Uint8Array(0);
    this.count = this.capacity = this.keyChars = 0;
    this.bounds = null;
    this.limitReached = false;
  }

  ensureCapacity(count) {
    if (count <= this.capacity) return;
    const capacity = Math.min(this.maxPoints, Math.max(count, this.capacity * 2, 4096));
    const positions = new Float32Array(capacity * 3);
    const intensities = new Float32Array(capacity);
    const rgb = new Uint8Array(capacity * 3);
    const hasRgb = new Uint8Array(capacity);
    positions.set(this.positions);
    intensities.set(this.intensities);
    rgb.set(this.rgb);
    hasRgb.set(this.hasRgb);
    this.positions = positions;
    this.intensities = intensities;
    this.rgb = rgb;
    this.hasRgb = hasRgb;
    this.capacity = capacity;
  }

  insertChunk(frame, start = 0, maxPoints = 4096, maxMs = 4) {
    const end = Math.min(frame.count, start + maxPoints);
    const begun = performance.now();
    let added = 0;
    let i = start;
    for (; i < end; i++) {
      if (i > start && (i - start) % 128 === 0 && performance.now() - begun >= maxMs) break;
      const p = i * 3;
      const x = frame.positions[p], y = frame.positions[p + 1], z = frame.positions[p + 2];
      const intensity = frame.intensities ? frame.intensities[i] : 0;
      if (!Number.isFinite(x) || !Number.isFinite(y) || !Number.isFinite(z) || !Number.isFinite(intensity)) continue;
      const ix = Math.floor(x / this.voxelSize);
      const iy = Math.floor(y / this.voxelSize);
      const iz = Math.floor(z / this.voxelSize);
      if (!Number.isSafeInteger(ix) || !Number.isSafeInteger(iy) || !Number.isSafeInteger(iz)) continue;
      const key = `${ix},${iy},${iz}`;
      if (this.voxels.has(key)) continue; // First valid point represents each voxel.
      if (this.count >= this.maxPoints) { this.limitReached = true; continue; }
      this.ensureCapacity(this.count + 1);
      this.voxels.set(key, this.count);
      this.keyChars += key.length;
      const target = this.count * 3;
      this.positions[target] = x;
      this.positions[target + 1] = y;
      this.positions[target + 2] = z;
      this.intensities[this.count] = intensity;
      if (frame.rgb) {
        this.rgb.set(frame.rgb.subarray(p, p + 3), target);
        this.hasRgb[this.count] = 1;
      }
      if (!this.bounds) this.bounds = { low: [x, y, z], high: [x, y, z] };
      else for (let axis = 0; axis < 3; axis++) {
        const value = frame.positions[p + axis];
        this.bounds.low[axis] = Math.min(this.bounds.low[axis], value);
        this.bounds.high[axis] = Math.max(this.bounds.high[axis], value);
      }
      this.count++;
      added++;
    }
    return { next: i, added };
  }

  snapshot() {
    return { count: this.count,
             positions: this.positions.subarray(0, this.count * 3),
             intensities: this.intensities.subarray(0, this.count),
             rgb: this.rgb.subarray(0, this.count * 3),
             hasRgb: this.hasRgb.subarray(0, this.count),
             bounds: this.bounds };
  }

  countSelected(bounds) {
    if (!bounds) return this.count;
    let count = 0;
    for (let i = 0; i < this.count; i++) {
      const p = i * 3;
      if (this.positions[p] >= bounds.low[0] && this.positions[p] <= bounds.high[0] &&
          this.positions[p + 1] >= bounds.low[1] && this.positions[p + 1] <= bounds.high[1] &&
          this.positions[p + 2] >= bounds.low[2] && this.positions[p + 2] <= bounds.high[2]) count++;
    }
    return count;
  }

  get memoryEstimateBytes() {
    // Approximate JS storage: allocated float arrays plus 64 B per Map entry and UTF-16 key bytes; excludes engine and GPU overhead.
    return this.positions.byteLength + this.intensities.byteLength + this.rgb.byteLength +
      this.hasRgb.byteLength + this.count * 64 + this.keyChars * 2;
  }
}
