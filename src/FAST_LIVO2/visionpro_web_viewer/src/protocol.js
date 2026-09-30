// VPPC v1, as emitted by src/visionpro_pointcloud_gateway.cpp.
export const HEADER_BYTES = 36;
export const POINT_STRIDE = 16;
const MAX_PAYLOAD = 64 * 1024 * 1024;
const utf8 = new TextDecoder('utf-8', { fatal: true });

export function decodeFrame(buffer) {
  if (!(buffer instanceof ArrayBuffer) || buffer.byteLength < HEADER_BYTES) {
    throw new Error('VPPC header is missing or truncated');
  }
  const view = new DataView(buffer);
  if (view.getUint8(0) !== 86 || view.getUint8(1) !== 80 ||
      view.getUint8(2) !== 80 || view.getUint8(3) !== 67) {
    throw new Error('VPPC magic mismatch');
  }
  const version = view.getUint16(4, true);
  const type = view.getUint16(6, true);
  if (version !== 1 || type !== 1) throw new Error(`Unsupported VPPC version/type ${version}/${type}`);
  const sequence = view.getBigUint64(8, true);
  const timestampNs = view.getBigUint64(16, true);
  const count = view.getUint32(24, true);
  const stride = view.getUint16(28, true);
  const frameIdBytes = view.getUint16(30, true);
  const payloadBytes = view.getUint32(32, true);
  if (stride !== POINT_STRIDE) throw new Error(`Unexpected point stride ${stride}`);
  if (count > Math.floor(MAX_PAYLOAD / stride) || payloadBytes !== count * stride) {
    throw new Error('Point count and payload length disagree or exceed limit');
  }
  const payloadOffset = HEADER_BYTES + frameIdBytes;
  if (payloadOffset > buffer.byteLength || payloadOffset + payloadBytes !== buffer.byteLength) {
    throw new Error('VPPC frame ID or payload is truncated');
  }
  const frameId = utf8.decode(new Uint8Array(buffer, HEADER_BYTES, frameIdBytes));
  const positions = new Float32Array(count * 3);
  const intensities = new Float32Array(count);
  const low = [Infinity, Infinity, Infinity];
  const high = [-Infinity, -Infinity, -Infinity];
  for (let i = 0; i < count; i++) {
    const offset = payloadOffset + i * stride;
    for (let axis = 0; axis < 3; axis++) {
      const value = view.getFloat32(offset + axis * 4, true);
      if (!Number.isFinite(value)) throw new Error(`Nonfinite XYZ at point ${i}`);
      positions[i * 3 + axis] = value;
      low[axis] = Math.min(low[axis], value);
      high[axis] = Math.max(high[axis], value);
    }
    const intensity = view.getFloat32(offset + 12, true);
    if (!Number.isFinite(intensity)) throw new Error(`Nonfinite intensity at point ${i}`);
    intensities[i] = intensity;
  }
  return { sequence, timestampNs, frameId, count, payloadBytes, frameBytes: buffer.byteLength,
           positions, intensities, bounds: count ? { low, high } : null };
}
