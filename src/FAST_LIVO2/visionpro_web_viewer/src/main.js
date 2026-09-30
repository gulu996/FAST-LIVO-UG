import { decodeFrame } from './protocol.js';
import { Metrics } from './metrics.js';
import { PointCloudRenderer } from './pointcloud_renderer.js';
import { HistoryAccumulator } from './history_accumulator.js';
import { ROIController } from './roi_controller.js';

const byId = (id) => document.getElementById(id);
const urlInput = byId('ws-url');
urlInput.value = `ws://${location.hostname || 'localhost'}:8765/`;
const metrics = new Metrics();
let viewer;
try {
  viewer = new PointCloudRenderer(byId('viewport'));
} catch (error) {
  byId('connection').textContent = 'WebGL unavailable';
  byId('error').textContent = String(error);
  throw error;
}
const history = new HistoryAccumulator();
let socket = null;
let pending = null;
let pendingHistory = null;
let historyDirty = false;
let historySkipped = 0;
let frameNumber = 0;
let lastHistoryUpload = 0;
let current = null;
let currentIntensity = null;
let selectedPoints = 0;
let selectedTimer = null;
let paused = false;
let lastUi = 0;
const roi = new ROIController(byId('roi-controls'), () => {
  viewer.setRoi(roi.enabled, roi.bounds);
  scheduleSelectedCount();
});

function countFrameSelected(frame, bounds) {
  if (!frame) return 0;
  if (!bounds) return frame.count;
  let count = 0;
  for (let i = 0; i < frame.count; i++) {
    const p = i * 3;
    if (frame.positions[p] >= bounds.low[0] && frame.positions[p] <= bounds.high[0] &&
        frame.positions[p + 1] >= bounds.low[1] && frame.positions[p + 1] <= bounds.high[1] &&
        frame.positions[p + 2] >= bounds.low[2] && frame.positions[p + 2] <= bounds.high[2]) count++;
  }
  return count;
}

function scheduleSelectedCount() {
  if (selectedTimer !== null) return;
  selectedTimer = setTimeout(() => {
    selectedTimer = null;
    const bounds = roi.enabled ? roi.bounds : null;
    selectedPoints = (byId('show-current').checked ? countFrameSelected(current, bounds) : 0) +
      (byId('show-history').checked ? history.countSelected(bounds) : 0);
    byId('selected-points').textContent = selectedPoints.toLocaleString();
  }, 100);
}

function clearHistory() {
  pendingHistory = null;
  historyDirty = false;
  history.clear();
  viewer.clearHistory();
  if (current?.bounds) roi.setDataBounds(current.bounds, true);
  else roi.clearBounds();
  byId('mapping-message').textContent = 'History cleared; current frame continues.';
  scheduleSelectedCount();
}

function processHistory(now) {
  if (pendingHistory && byId('accumulate').checked) {
    const { frame, offset } = pendingHistory;
    const result = history.insertChunk(frame, offset);
    pendingHistory.offset = result.next;
    if (result.added) historyDirty = true;
    if (result.next === frame.count) pendingHistory = null;
    if (history.limitReached) byId('mapping-message').textContent = 'History limit reached; no new voxels will be added.';
  }
  if (historyDirty && now - lastHistoryUpload >= 200) {
    viewer.setHistory(history.snapshot());
    historyDirty = false;
    lastHistoryUpload = now;
    roi.setDataBounds(history.bounds);
    scheduleSelectedCount();
  }
}

function connect() {
  let url;
  try {
    url = new URL(urlInput.value.trim());
    if (url.protocol !== 'ws:' && url.protocol !== 'wss:') throw new Error('Use ws:// or wss://');
  } catch (error) {
    byId('error').textContent = `Invalid WebSocket URL: ${error.message}`;
    return;
  }
  if (socket) socket.close();
  pending = null;
  let ws;
  try {
    ws = new WebSocket(url.href);
  } catch (error) {
    byId('error').textContent = `Cannot open WebSocket: ${error.message}`;
    byId('connection').textContent = 'Disconnected';
    return;
  }
  socket = ws;
  ws.binaryType = 'arraybuffer';
  byId('connection').textContent = 'Connecting';
  byId('server').textContent = url.host;
  byId('error').textContent = '';
  ws.onopen = () => {
    if (socket === ws) byId('connection').textContent = 'Connected';
  };
  ws.onmessage = (event) => {
    if (socket !== ws) return;
    if (!(event.data instanceof ArrayBuffer)) {
      metrics.malformed++;
      console.error('VPPC malformed frame: expected binary WebSocket message');
      return;
    }
    metrics.onReceive(event.data.byteLength);
    // ponytail: the browser event queue is consumed cheaply; only one unparsed frame survives until the next render tick.
    if (pending) metrics.viewerDrops++;
    pending = event.data;
  };
  ws.onerror = () => {
    if (socket === ws) byId('error').textContent = 'WebSocket connection error; check Gateway address and port.';
  };
  ws.onclose = () => {
    if (socket === ws) {
      socket = null;
      pending = null;
      byId('connection').textContent = 'Disconnected';
    }
  };
}

byId('connect').addEventListener('click', connect);
byId('disconnect').addEventListener('click', () => {
  if (socket) socket.close();
  pending = null;
});
byId('reset').addEventListener('click', () => viewer.resetView());
byId('point-size').addEventListener('input', (event) => {
  viewer.setPointSize(event.target.value);
  byId('point-size-value').textContent = event.target.value;
});
byId('color-mode').addEventListener('change', (event) => viewer.setMode(event.target.value));
byId('show-axis').addEventListener('change', (event) => viewer.setAxesVisible(event.target.checked));
byId('show-grid').addEventListener('change', (event) => viewer.setGridVisible(event.target.checked));
byId('show-panels').addEventListener('change', (event) => {
  byId('status-panel').hidden = byId('controls-panel').hidden = !event.target.checked;
  document.body.classList.toggle('panels-hidden', !event.target.checked);
  viewer.resize();
});
byId('show-current').addEventListener('change', (event) => { viewer.setShowCurrent(event.target.checked); scheduleSelectedCount(); });
byId('show-history').addEventListener('change', (event) => { viewer.setShowHistory(event.target.checked); scheduleSelectedCount(); });
byId('accumulate').addEventListener('change', (event) => {
  if (!event.target.checked) pendingHistory = null;
  byId('mapping-message').textContent = event.target.checked ? 'Accumulating valid voxels.' : 'Accumulation paused; current frame continues.';
});
byId('voxel-size').addEventListener('change', (event) => {
  const size = Number(event.target.value);
  if (!Number.isFinite(size) || size < 0.001) {
    event.target.value = history.voxelSize.toFixed(3);
    byId('mapping-message').textContent = 'Voxel size must be at least 0.001 m and finite.';
    return;
  }
  if (size !== history.voxelSize) {
    clearHistory();
    history.setVoxelSize(size);
    byId('mapping-message').textContent = `Voxel size changed to ${size} m; history cleared.`;
  }
});
byId('history-stride').addEventListener('change', (event) => {
  const value = Number(event.target.value);
  if (!Number.isSafeInteger(value) || value < 1 || value > 20) {
    event.target.value = '1';
    byId('mapping-message').textContent = 'History update interval must be an integer from 1 to 20.';
  }
});
byId('clear-history').addEventListener('click', clearHistory);
byId('enable-roi').addEventListener('change', (event) => roi.setEnabled(event.target.checked));
byId('reset-roi').addEventListener('click', () => roi.fit());
byId('fit-roi').addEventListener('click', () => {
  roi.fit();
  viewer.resetView(history.bounds || current?.bounds);
});
byId('pause').addEventListener('click', () => {
  paused = !paused;
  byId('pause').textContent = paused ? 'Resume Rendering' : 'Pause Rendering';
});
window.addEventListener('resize', () => viewer.resize());
window.addEventListener('pagehide', () => {
  if (socket) socket.close();
  if (selectedTimer !== null) clearTimeout(selectedTimer);
  viewer.dispose();
});

function refreshStatus() {
  byId('sequence').textContent = current ? current.sequence.toString() : '—';
  byId('frame-id').textContent = current?.frameId || '—';
  byId('timestamp').textContent = current ? current.timestampNs.toString() : '—';
  byId('points').textContent = current ? current.count.toLocaleString() : '0';
  byId('history-voxels').textContent = history.count.toLocaleString();
  byId('history-points').textContent = history.count.toLocaleString();
  byId('history-voxel-size').textContent = `${history.voxelSize.toFixed(3)} m`;
  byId('history-memory').textContent = `${(history.memoryEstimateBytes / 1048576).toFixed(1)} MB`;
  byId('accumulation-status').textContent = !byId('accumulate').checked ? 'OFF' : (history.limitReached ? 'LIMIT REACHED' : 'ON');
  const bounds = history.bounds || current?.bounds;
  byId('map-size').textContent = bounds ? ['X', 'Y', 'Z'].map((axis, i) => `${axis} ${(bounds.high[i] - bounds.low[i]).toFixed(2)} m`).join(' · ') : '—';
  byId('receive-hz').textContent = `${metrics.receiveHz.toFixed(1)} Hz`;
  byId('render-fps').textContent = `${metrics.renderFps.toFixed(0)} FPS`;
  byId('frame-size').textContent = current ? `${(current.frameBytes / 1024).toFixed(1)} KiB` : '0 KiB';
  byId('bandwidth').textContent = `${(metrics.bandwidth / 1e6).toFixed(2)} MB/s`;
  byId('viewer-drops').textContent = String(metrics.viewerDrops);
  byId('malformed').textContent = String(metrics.malformed);
  byId('geometry-count').textContent = String(viewer.geometryCount);
  byId('buffer-size').textContent = `${(viewer.bufferBytes / 1048576).toFixed(1)} MB`;
  byId('history-skipped').textContent = String(historySkipped);
  byId('intensity-range').textContent = currentIntensity
    ? `${currentIntensity.min.toFixed(1)}–${currentIntensity.max.toFixed(1)} (P5 ${currentIntensity.p5.toFixed(1)}, P95 ${currentIntensity.p95.toFixed(1)})`
    : '—';
}

function tick(now) {
  if (!paused) {
    if (pending) {
      const data = pending;
      pending = null;
      try {
        const frame = decodeFrame(data);
        currentIntensity = viewer.setFrame(frame);
        current = frame;
        frameNumber++;
        if (!history.count) roi.setDataBounds(frame.bounds);
        if (byId('accumulate').checked && frameNumber % (Number(byId('history-stride').value) || 1) === 0) {
          if (pendingHistory) historySkipped++;
          pendingHistory = { frame, offset: 0 };
        }
        scheduleSelectedCount();
        byId('error').textContent = '';
      } catch (error) {
        metrics.malformed++;
        console.error('VPPC malformed frame:', error);
        byId('error').textContent = `Dropped malformed frame: ${error.message}`;
      }
    }
    viewer.draw();
    metrics.onRender();
    // ponytail: ingest at most a short chunk after drawing current; a newer frame replaces unfinished history work.
    processHistory(now);
  }
  metrics.sample(now);
  if (now - lastUi >= 250) {
    refreshStatus();
    lastUi = now;
  }
  requestAnimationFrame(tick);
}
requestAnimationFrame(tick);
