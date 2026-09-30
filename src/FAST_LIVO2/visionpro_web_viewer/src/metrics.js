export class Metrics {
  constructor() {
    this.received = 0;
    this.rendered = 0;
    this.receivedBytes = 0;
    this.viewerDrops = 0;
    this.malformed = 0;
    this.receiveHz = 0;
    this.renderFps = 0;
    this.bandwidth = 0;
    this.lastSample = performance.now();
  }

  onReceive(bytes) {
    this.received++;
    this.receivedBytes += bytes;
  }

  onRender() { this.rendered++; }

  sample(now = performance.now()) {
    const elapsed = (now - this.lastSample) / 1000;
    if (elapsed < 1) return;
    this.receiveHz = this.received / elapsed;
    this.renderFps = this.rendered / elapsed;
    this.bandwidth = this.receivedBytes / elapsed;
    this.received = this.rendered = this.receivedBytes = 0;
    this.lastSample = now;
  }
}
