export class ROIController {
  constructor(root, onChange) {
    this.onChange = onChange;
    this.enabled = false;
    this.customized = false;
    this.dataBounds = null;
    this.domain = null;
    this.selected = null;
    this.axes = ['x', 'y', 'z'].map((name, axis) => {
      const min = root.querySelector(`#roi-${name}-min`);
      const max = root.querySelector(`#roi-${name}-max`);
      const output = root.querySelector(`#roi-${name}-value`);
      const pair = root.querySelector(`#roi-${name}-pair`);
      min.addEventListener('input', () => this.change(axis, 0, Number(min.value)));
      max.addEventListener('input', () => this.change(axis, 1, Number(max.value)));
      pair.addEventListener('pointerdown', (event) => {
        if (!this.selected) return;
        event.preventDefault();
        const value = this.valueAt(axis, event.clientX);
        const lo = this.selected.low[axis], hi = this.selected.high[axis];
        // ponytail: one active pointer keeps the two-thumb control small; add per-pointer state if multitouch editing is needed.
        const end = lo === hi ? (value >= lo ? 1 : 0) :
          (Math.abs(value - lo) <= Math.abs(value - hi) ? 0 : 1);
        this.active = { axis, end, pointerId: event.pointerId };
        pair.setPointerCapture(event.pointerId);
        this.change(axis, end, value);
      });
      pair.addEventListener('pointermove', (event) => {
        if (this.active?.axis === axis && this.active.pointerId === event.pointerId)
          this.change(axis, this.active.end, this.valueAt(axis, event.clientX));
      });
      const stop = (event) => { if (this.active?.pointerId === event.pointerId) this.active = null; };
      pair.addEventListener('pointerup', stop);
      pair.addEventListener('pointercancel', stop);
      return { min, max, output, pair };
    });
  }

  setDataBounds(bounds, reset = false) {
    if (!bounds) return;
    if (bounds.low.some((v) => !Number.isFinite(v)) ||
        bounds.high.some((v, i) => !Number.isFinite(v) || v < bounds.low[i])) return;
    if (!reset && this.dataBounds && bounds.low.every((v, i) => v === this.dataBounds.low[i]) &&
        bounds.high.every((v, i) => v === this.dataBounds.high[i])) return;
    this.dataBounds = { low: [...bounds.low], high: [...bounds.high] };
    const low = [], high = [];
    for (let i = 0; i < 3; i++) {
      const margin = Math.max(0.05, (bounds.high[i] - bounds.low[i]) * 0.05);
      low[i] = bounds.low[i] - margin;
      high[i] = bounds.high[i] + margin;
      if (this.customized && !reset && this.domain) {
        low[i] = Math.min(low[i], this.domain.low[i]);
        high[i] = Math.max(high[i], this.domain.high[i]);
      }
    }
    this.domain = { low, high };
    if (!this.customized || reset) {
      this.selected = { low: [...low], high: [...high] };
      this.customized = false;
    }
    this.render();
    this.onChange();
  }

  valueAt(axis, clientX) {
    const rect = this.axes[axis].pair.getBoundingClientRect();
    const fraction = Math.min(1, Math.max(0, (clientX - rect.left - 9) / Math.max(1, rect.width - 18)));
    return this.domain.low[axis] + fraction * (this.domain.high[axis] - this.domain.low[axis]);
  }

  clearBounds() {
    this.dataBounds = this.domain = this.selected = null;
    this.customized = false;
    for (const { min, max, output, pair } of this.axes) {
      min.disabled = max.disabled = true;
      output.textContent = '— m';
      pair.style.setProperty('--lo', '0%');
      pair.style.setProperty('--hi', '100%');
    }
    this.onChange();
  }

  change(axis, end, value) {
    if (!this.selected || !Number.isFinite(value)) return;
    if (end === 0) this.selected.low[axis] = Math.min(value, this.selected.high[axis]);
    else this.selected.high[axis] = Math.max(value, this.selected.low[axis]);
    this.customized = true;
    this.render();
    this.onChange();
  }

  fit() { if (this.dataBounds) this.setDataBounds(this.dataBounds, true); }
  setEnabled(enabled) { this.enabled = enabled; this.onChange(); }
  get bounds() { return this.selected; }

  render() {
    if (!this.domain || !this.selected) return;
    for (let i = 0; i < 3; i++) {
      const { min, max, output, pair } = this.axes[i];
      const from = this.domain.low[i], to = this.domain.high[i];
      for (const input of [min, max]) {
        input.min = String(from);
        input.max = String(to);
        input.step = 'any';
        input.disabled = false;
      }
      min.value = String(this.selected.low[i]);
      max.value = String(this.selected.high[i]);
      output.textContent = `${this.selected.low[i].toFixed(3)} to ${this.selected.high[i].toFixed(3)} m`;
      pair.style.setProperty('--lo', `${100 * (this.selected.low[i] - from) / (to - from)}%`);
      pair.style.setProperty('--hi', `${100 * (this.selected.high[i] - from) / (to - from)}%`);
    }
  }
}
