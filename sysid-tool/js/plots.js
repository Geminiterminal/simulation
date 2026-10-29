/**
 * Lightweight interactive 2D canvas plots (scatter + lines) with legends,
 * tick axes, wheel zoom, drag pan, and double-click-to-fit.
 */

const NICE_STEPS = [1, 2, 2.5, 5, 10];

function niceTickStep(span, targetTicks) {
  const raw = span / Math.max(1, targetTicks);
  const mag = 10 ** Math.floor(Math.log10(raw));
  for (const step of NICE_STEPS) {
    if (step * mag >= raw) return step * mag;
  }
  return 10 * mag;
}

function formatTick(x) {
  if (x === 0) return "0";
  const ax = Math.abs(x);
  if (ax >= 1e5 || ax < 1e-3) return x.toExponential(1);
  if (ax >= 10) return x.toFixed(ax % 1 === 0 ? 0 : 1);
  if (ax >= 1) return x.toFixed(ax % 1 === 0 ? 0 : 2).replace(/\.?0+$/, "");
  return x.toFixed(3).replace(/0+$/, "").replace(/\.$/, "");
}

export class Plot {
  /**
   * @param {HTMLCanvasElement} canvas
   * @param {{title: string, xLabel: string, yLabel: string, yLock?: [number, number]}} opts
   */
  constructor(canvas, opts) {
    this.canvas = canvas;
    this.opts = opts;
    this.series = [];
    this.limits = null; // {xMin, xMax, yMin, yMax}
    this.drag = null;

    canvas.addEventListener("wheel", (e) => this.onWheel(e), { passive: false });
    canvas.addEventListener("mousedown", (e) => {
      this.drag = { x: e.clientX, y: e.clientY };
    });
    window.addEventListener("mouseup", () => (this.drag = null));
    canvas.addEventListener("mousemove", (e) => this.onMouseMove(e));
    canvas.addEventListener("dblclick", () => {
      this.limits = null;
      this.draw();
    });
    new ResizeObserver(() => this.draw()).observe(canvas);
  }

  themeColors() {
    const style = getComputedStyle(document.documentElement);
    const get = (name, fallback) => style.getPropertyValue(name).trim() || fallback;
    return {
      bg: get("--plot-bg", "#101018"),
      grid: get("--plot-grid", "#26263a"),
      axis: get("--plot-axis", "#5a5a72"),
      text: get("--plot-text", "#c5c5d8"),
      title: get("--plot-title", "#e8e8f4"),
      legendBg: get("--plot-legend-bg", "rgba(16,16,24,0.85)"),
    };
  }

  /** Sets series: [{name, points: [[x,y]...], color, kind: 'scatter'|'line', size}] */
  setSeries(series) {
    this.series = series;
    this.limits = null;
    this.draw();
  }

  fit() {
    let xMin = Infinity, xMax = -Infinity, yMin = Infinity, yMax = -Infinity;
    for (const s of this.series) {
      for (const [x, y] of s.points) {
        if (!Number.isFinite(x) || !Number.isFinite(y)) continue;
        xMin = Math.min(xMin, x);
        xMax = Math.max(xMax, x);
        yMin = Math.min(yMin, y);
        yMax = Math.max(yMax, y);
      }
    }
    if (!Number.isFinite(xMin)) {
      xMin = 0; xMax = 1; yMin = 0; yMax = 1;
    }
    if (xMin === xMax) { xMin -= 1; xMax += 1; }
    if (yMin === yMax) { yMin -= 1; yMax += 1; }
    const xPad = (xMax - xMin) * 0.05;
    const yPad = (yMax - yMin) * 0.08;
    if (this.opts.yLock) {
      yMin = this.opts.yLock[0];
      yMax = this.opts.yLock[1];
    }
    this.limits = {
      xMin: xMin - xPad,
      xMax: xMax + xPad,
      yMin: yMin - (this.opts.yLock ? 0 : yPad),
      yMax: yMax + (this.opts.yLock ? 0 : yPad),
    };
  }

  onWheel(e) {
    e.preventDefault();
    if (!this.limits) this.fit();
    const rect = this.canvas.getBoundingClientRect();
    const frac = (e.clientX - rect.left - this.marginLeft()) / this.plotWidth();
    const zoom = e.deltaY > 0 ? 1.15 : 1 / 1.15;
    const { xMin, xMax } = this.limits;
    const xAt = xMin + frac * (xMax - xMin);
    this.limits.xMin = xAt - (xAt - xMin) * zoom;
    this.limits.xMax = xAt + (xMax - xAt) * zoom;

    const fracY = 1 - (e.clientY - rect.top - this.marginTop()) / this.plotHeight();
    const { yMin, yMax } = this.limits;
    const yAt = yMin + fracY * (yMax - yMin);
    this.limits.yMin = this.opts.yLock ? yMin : yAt - (yAt - yMin) * zoom;
    this.limits.yMax = this.opts.yLock ? yMax : yAt + (yMax - yAt) * zoom;
    this.draw();
  }

  onMouseMove(e) {
    if (!this.drag || !this.limits) return;
    const dx = e.clientX - this.drag.x;
    const dy = e.clientY - this.drag.y;
    this.drag = { x: e.clientX, y: e.clientY };
    const xShift = (-dx / this.plotWidth()) * (this.limits.xMax - this.limits.xMin);
    const yShift = (dy / this.plotHeight()) * (this.limits.yMax - this.limits.yMin);
    this.limits.xMin += xShift;
    this.limits.xMax += xShift;
    if (!this.opts.yLock) {
      this.limits.yMin += yShift;
      this.limits.yMax += yShift;
    }
    this.draw();
  }

  marginLeft() {
    return 52;
  }
  marginTop() {
    return 26;
  }
  marginRight() {
    return 10;
  }
  marginBottom() {
    return 34;
  }
  plotWidth() {
    return this.canvas.clientWidth - this.marginLeft() - this.marginRight();
  }
  plotHeight() {
    return this.canvas.clientHeight - this.marginTop() - this.marginBottom();
  }

  draw() {
    const canvas = this.canvas;
    const dpr = window.devicePixelRatio || 1;
    const w = canvas.clientWidth;
    const h = canvas.clientHeight;
    if (w === 0 || h === 0) return;
    if (canvas.width !== w * dpr || canvas.height !== h * dpr) {
      canvas.width = w * dpr;
      canvas.height = h * dpr;
    }
    const ctx = canvas.getContext("2d");
    ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    const colors = this.themeColors();

    ctx.fillStyle = colors.bg;
    ctx.fillRect(0, 0, w, h);

    if (!this.limits) this.fit();
    const { xMin, xMax, yMin, yMax } = this.limits;
    const ml = this.marginLeft();
    const mt = this.marginTop();
    const pw = this.plotWidth();
    const ph = this.plotHeight();

    const toX = (x) => ml + ((x - xMin) / (xMax - xMin)) * pw;
    const toY = (y) => mt + ph - ((y - yMin) / (yMax - yMin)) * ph;

    // Grid + ticks
    const xStep = niceTickStep(xMax - xMin, Math.max(3, Math.floor(pw / 70)));
    const yStep = niceTickStep(yMax - yMin, Math.max(3, Math.floor(ph / 45)));
    ctx.font = "10px system-ui, sans-serif";
    ctx.strokeStyle = colors.grid;
    ctx.fillStyle = colors.text;
    ctx.lineWidth = 1;

    for (let x = Math.ceil(xMin / xStep) * xStep; x <= xMax; x += xStep) {
      const px = toX(x);
      ctx.beginPath();
      ctx.moveTo(px, mt);
      ctx.lineTo(px, mt + ph);
      ctx.stroke();
      ctx.textAlign = "center";
      ctx.fillText(formatTick(x), px, mt + ph + 13);
    }
    for (let y = Math.ceil(yMin / yStep) * yStep; y <= yMax; y += yStep) {
      const py = toY(y);
      ctx.beginPath();
      ctx.moveTo(ml, py);
      ctx.lineTo(ml + pw, py);
      ctx.stroke();
      ctx.textAlign = "right";
      ctx.fillText(formatTick(y), ml - 5, py + 3);
    }

    // Axes labels + title
    ctx.fillStyle = colors.text;
    ctx.textAlign = "center";
    ctx.fillText(this.opts.xLabel, ml + pw / 2, h - 6);
    ctx.save();
    ctx.translate(11, mt + ph / 2);
    ctx.rotate(-Math.PI / 2);
    ctx.fillText(this.opts.yLabel, 0, 0);
    ctx.restore();

    ctx.fillStyle = colors.title;
    ctx.font = "bold 11px system-ui, sans-serif";
    ctx.textAlign = "center";
    ctx.fillText(this.opts.title, ml + pw / 2, 13);
    ctx.font = "10px system-ui, sans-serif";

    // Clip plot region and draw series.
    ctx.save();
    ctx.beginPath();
    ctx.rect(ml, mt, pw, ph);
    ctx.clip();

    for (const s of this.series) {
      if (s.kind === "line") {
        ctx.strokeStyle = s.color;
        ctx.lineWidth = s.width ?? 1.5;
        ctx.beginPath();
        let started = false;
        for (const [x, y] of s.points) {
          if (!Number.isFinite(x) || !Number.isFinite(y)) continue;
          const px = toX(x);
          const py = toY(y);
          if (!started) {
            ctx.moveTo(px, py);
            started = true;
          } else ctx.lineTo(px, py);
        }
        ctx.stroke();
      } else {
        ctx.fillStyle = s.color;
        const size = s.size ?? 1.25;
        for (const [x, y] of s.points) {
          if (!Number.isFinite(x) || !Number.isFinite(y)) continue;
          ctx.fillRect(toX(x) - size / 2, toY(y) - size / 2, size, size);
        }
      }
    }
    ctx.restore();

    // Frame
    ctx.strokeStyle = colors.axis;
    ctx.strokeRect(ml, mt, pw, ph);

    // Legend (top-right)
    const visible = this.series.filter((s) => s.points.length > 0);
    if (visible.length > 0) {
      ctx.font = "10px system-ui, sans-serif";
      const pad = 6;
      const rowH = 14;
      const boxW = 96;
      const boxH = visible.length * rowH + pad * 2 - 4;
      const bx = ml + pw - boxW - 6;
      const by = mt + 6;
      ctx.fillStyle = colors.legendBg;
      ctx.fillRect(bx, by, boxW, boxH);
      ctx.strokeStyle = colors.axis;
      ctx.strokeRect(bx, by, boxW, boxH);
      visible.forEach((s, i) => {
        const yy = by + pad + i * rowH + 4;
        ctx.fillStyle = s.color;
        ctx.fillRect(bx + pad, yy - 4, 10, 8);
        ctx.fillStyle = colors.text;
        ctx.textAlign = "left";
        ctx.fillText(s.name, bx + pad + 16, yy + 1);
      });
    }
  }
}
