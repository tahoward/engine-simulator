/**
 * The dyno sheet: crank power and torque against rpm, drawn live as a dyno run goes, one trace per
 * gear.
 *
 * Power and torque are two plots stacked on one rpm axis rather than two scales on one plot, whose
 * alignment would be arbitrary. Each gear keeps its own colour, in a fixed order, and the legend names
 * them. The traces are the same engine, so they lie on top of one another where the gears overlap; what
 * differs between them is how fast the rpm rose, which the car's gearing sets.
 *
 * Each point is one engine cycle's average, as a real dyno reports it. The line is smoothed over a
 * few cycles either side, because cycle-to-cycle combustion scatter is a few percent; the tooltip and
 * the peaks read the smoothed values too.
 */

import type { DynoConfig, DynoSnapshot } from '../model/spec.js';

/** One colour per gear, first to sixth: the reference categorical palette's dark steps. */
const GEAR_COLOURS = ['#3987e5', '#d95926', '#199e70', '#c98500', '#d55181', '#008300'];
const SURFACE = '#1a1e25';
const INK = '#e6eaf0';
const INK_DIM = '#8b95a5';
const GRID = 'rgba(255,255,255,0.06)';
const AXIS = 'rgba(255,255,255,0.14)';

/** Cycles either side of a point averaged into the line. */
const SMOOTH = 3;

const KW_TO_HP = 1.341;

interface Point {
  rpm: number;
  torque: number;
  kmh: number;
  gear: number;
}

interface Smoothed extends Point {
  kw: number;
}

const PHASE_TEXT: Record<DynoSnapshot['phase'], string> = {
  pull: 'pulling',
  shiftOut: 'shifting',
  shiftIn: 'shifting',
  cooldown: 'winding down',
};

export class DynoSheet {
  private readonly card: HTMLElement;
  private readonly status: HTMLElement;
  private readonly peaks: HTMLElement;
  private readonly canvas: HTMLCanvasElement;
  private readonly ctx: CanvasRenderingContext2D;
  private readonly tooltip: HTMLElement;
  private readonly table: HTMLTableSectionElement;

  private points: Point[] = [];
  private smoothed: Smoothed[] = [];
  private config: DynoConfig | null = null;
  private minRpm = 0;
  private dirty = true;
  private hoverX: number | null = null;
  private running = false;
  /**
   * Whether a snapshot has shown this run going. Until one has, a snapshot without a run is one sent
   * before the audio thread took the start, not the run ending.
   */
  private seen = false;

  private width = 0;
  private height = 0;

  constructor(parent: HTMLElement) {
    this.card = el('div', 'dyno-card hidden', parent);
    const head = el('div', 'dyno-head', this.card);
    el('strong', '', head).textContent = 'Dyno run · 6-speed';
    const close = el('button', 'dyno-close', head) as HTMLButtonElement;
    close.textContent = '×';
    close.title = 'Hide the dyno sheet';
    close.addEventListener('click', () => this.card.classList.add('hidden'));

    this.status = el('div', 'dyno-status', this.card);
    this.peaks = el('div', 'dyno-peaks', this.card);

    const plot = el('div', 'dyno-plot', this.card);
    this.canvas = el('canvas', '', plot) as HTMLCanvasElement;
    this.canvas.setAttribute('role', 'img');
    this.canvas.setAttribute('aria-label', 'Crank power and torque against engine speed, one trace per gear');
    this.tooltip = el('div', 'dyno-tooltip hidden', plot);
    const ctx = this.canvas.getContext('2d');
    if (!ctx) throw new Error('DynoSheet: 2D canvas context unavailable');
    this.ctx = ctx;

    const legend = el('div', 'dyno-legend', this.card);
    GEAR_COLOURS.forEach((c, i) => {
      const item = el('span', '', legend);
      const swatch = el('i', '', item);
      swatch.style.background = c;
      item.append(`${ordinal(i + 1)}`);
    });

    const details = el('details', 'dyno-table', this.card);
    el('summary', '', details).textContent = 'Per gear';
    const tbl = el('table', '', details) as HTMLTableElement;
    const thead = el('thead', '', tbl);
    const hr = el('tr', '', thead);
    for (const h of ['Gear', 'km/h', 'Peak kW', 'Peak N·m']) el('th', '', hr).textContent = h;
    this.table = el('tbody', '', tbl) as HTMLTableSectionElement;

    this.canvas.addEventListener('pointermove', (e) => {
      const r = this.canvas.getBoundingClientRect();
      this.hoverX = e.clientX - r.left;
      this.dirty = true;
    });
    this.canvas.addEventListener('pointerleave', () => {
      this.hoverX = null;
      this.dirty = true;
    });
    new ResizeObserver(() => this.resize()).observe(this.canvas);
  }

  /** Clear the sheet and show it, for a run about to start through `config` from `rpm`. */
  begin(config: DynoConfig, rpm: number): void {
    this.config = config;
    this.minRpm = Math.max(Math.floor(rpm / 1000) * 1000, 0);
    this.points = [];
    this.smoothed = [];
    this.running = true;
    this.seen = false;
    this.card.classList.remove('hidden');
    this.status.textContent = 'Starting in 1st…';
    this.peaks.textContent = '';
    this.table.replaceChildren();
    this.resize();
    this.dirty = true;
  }

  /** Take a snapshot's dyno state: `null` once the run is over. */
  onSnapshot(dyno: DynoSnapshot | null): void {
    if (!this.running) return;
    if (!dyno) {
      if (!this.seen) return;
      this.running = false;
      this.status.textContent = this.points.length > 0 ? 'Run complete' : 'Run stopped';
      return;
    }
    this.seen = true;
    const p = dyno.points;
    for (let i = 0; i + 3 < p.length; i += 4) {
      this.points.push({ rpm: p[i]!, torque: p[i + 1]!, kmh: p[i + 2]!, gear: p[i + 3]! });
    }
    if (p.length > 0) {
      this.smoothed = smooth(this.points);
      this.summarise();
      this.dirty = true;
    }
    const state = dyno.finished ? 'winding down' : PHASE_TEXT[dyno.phase];
    this.status.textContent =
      `${ordinal(dyno.gear)} gear · ${Math.round(dyno.speedKmh)} km/h · ` +
      `${dyno.elapsed.toFixed(1)} s · ${state}`;
  }

  /** Redraw if anything changed. Call once per animation frame. */
  draw(): void {
    if (!this.dirty || this.card.classList.contains('hidden')) return;
    this.dirty = false;
    const ctx = this.ctx;
    const w = this.width;
    const h = this.height;
    ctx.clearRect(0, 0, w, h);
    if (!this.config || w === 0) return;

    const left = 46;
    const right = 12;
    const gap = 14;
    const axisBand = 22;
    const plotH = (h - axisBand - gap - 8) / 2;
    const tops = [6, 6 + plotH + gap];
    const x0 = this.minRpm;
    const x1 = Math.ceil((this.config.shiftRpm + 1) / 1000) * 1000;
    const xs = (rpm: number) => left + ((rpm - x0) / (x1 - x0)) * (w - left - right);

    const pts = this.smoothed;
    const maxKw = Math.max(10, ...pts.map((q) => q.kw));
    const maxNm = Math.max(10, ...pts.map((q) => q.torque));
    const panes = [
      { title: 'Power, kW', max: niceMax(maxKw * 1.08), value: (q: Smoothed) => q.kw },
      { title: 'Torque, N·m', max: niceMax(maxNm * 1.08), value: (q: Smoothed) => q.torque },
    ];

    ctx.font = '11px system-ui, sans-serif';
    panes.forEach((pane, k) => {
      const top = tops[k]!;
      const ys = (v: number) => top + plotH - (Math.max(v, 0) / pane.max) * plotH;
      // Grid and y labels.
      ctx.textAlign = 'right';
      ctx.textBaseline = 'middle';
      for (let i = 0; i <= 4; i++) {
        const v = (pane.max * i) / 4;
        const y = Math.round(ys(v)) + 0.5;
        ctx.strokeStyle = i === 0 ? AXIS : GRID;
        ctx.lineWidth = 1;
        ctx.beginPath();
        ctx.moveTo(left, y);
        ctx.lineTo(w - right, y);
        ctx.stroke();
        ctx.fillStyle = INK_DIM;
        ctx.fillText(formatNumber(v), left - 6, y);
      }
      ctx.textAlign = 'left';
      ctx.textBaseline = 'top';
      ctx.fillStyle = INK;
      ctx.fillText(pane.title, left + 4, top + 2);

      // One line per gear, broken at every shift.
      ctx.lineWidth = 2;
      ctx.lineJoin = 'round';
      ctx.lineCap = 'round';
      let prevGear = -1;
      for (const q of pts) {
        if (q.gear !== prevGear) {
          if (prevGear >= 0) ctx.stroke();
          ctx.strokeStyle = GEAR_COLOURS[(q.gear - 1) % GEAR_COLOURS.length]!;
          ctx.beginPath();
          ctx.moveTo(xs(q.rpm), ys(pane.value(q)));
          prevGear = q.gear;
        } else {
          ctx.lineTo(xs(q.rpm), ys(pane.value(q)));
        }
      }
      if (prevGear >= 0) ctx.stroke();

      // The peak, marked and labelled.
      const peak = pts.reduce<Smoothed | null>((b, q) => (!b || pane.value(q) > pane.value(b) ? q : b), null);
      if (peak) {
        const px = xs(peak.rpm);
        const py = ys(pane.value(peak));
        ctx.fillStyle = SURFACE;
        ctx.beginPath();
        ctx.arc(px, py, 6, 0, Math.PI * 2);
        ctx.fill();
        ctx.fillStyle = GEAR_COLOURS[(peak.gear - 1) % GEAR_COLOURS.length]!;
        ctx.beginPath();
        ctx.arc(px, py, 4, 0, Math.PI * 2);
        ctx.fill();
        const label = `${Math.round(pane.value(peak))} @ ${formatNumber(Math.round(peak.rpm))}`;
        ctx.fillStyle = INK;
        ctx.textBaseline = 'bottom';
        const tw = ctx.measureText(label).width;
        ctx.textAlign = px + 8 + tw > w - right ? 'right' : 'left';
        ctx.fillText(label, ctx.textAlign === 'right' ? px - 8 : px + 8, py - 4);
      }
    });

    // Shared rpm axis.
    const axisY = tops[1]! + plotH;
    ctx.fillStyle = INK_DIM;
    ctx.textAlign = 'center';
    ctx.textBaseline = 'top';
    const step = niceStep((x1 - x0) / 6);
    for (let r = Math.ceil(x0 / step) * step; r <= x1; r += step) {
      const x = Math.round(xs(r)) + 0.5;
      ctx.strokeStyle = GRID;
      ctx.lineWidth = 1;
      for (const top of tops) {
        ctx.beginPath();
        ctx.moveTo(x, top);
        ctx.lineTo(x, top + plotH);
        ctx.stroke();
      }
      ctx.fillText(formatNumber(r), x, axisY + 5);
    }
    ctx.textAlign = 'right';
    ctx.fillText('rpm', w - right, axisY + 5);

    this.drawHover(xs, x0, x1, left, right, tops, plotH);
  }

  /** Crosshair across both plots, and a tooltip with every gear's reading at that speed. */
  private drawHover(
    xs: (rpm: number) => number,
    x0: number,
    x1: number,
    left: number,
    right: number,
    tops: number[],
    plotH: number,
  ): void {
    const hx = this.hoverX;
    if (hx === null || hx < left || hx > this.width - right || this.smoothed.length === 0) {
      this.tooltip.classList.add('hidden');
      return;
    }
    const rpm = x0 + ((hx - left) / (this.width - left - right)) * (x1 - x0);
    const rows: Smoothed[] = [];
    for (let g = 1; g <= GEAR_COLOURS.length; g++) {
      const at = interpolate(this.smoothed, g, rpm);
      if (at) rows.push(at);
    }
    if (rows.length === 0) {
      this.tooltip.classList.add('hidden');
      return;
    }
    const ctx = this.ctx;
    ctx.strokeStyle = 'rgba(230,234,240,0.35)';
    ctx.lineWidth = 1;
    const x = Math.round(xs(rpm)) + 0.5;
    ctx.beginPath();
    ctx.moveTo(x, tops[0]!);
    ctx.lineTo(x, tops[1]! + plotH);
    ctx.stroke();

    this.tooltip.replaceChildren();
    el('div', 'dyno-tip-head', this.tooltip).textContent = `${formatNumber(Math.round(rpm))} rpm`;
    for (const q of rows) {
      const row = el('div', 'dyno-tip-row', this.tooltip);
      const swatch = el('i', '', row);
      swatch.style.background = GEAR_COLOURS[q.gear - 1]!;
      row.append(
        `${ordinal(q.gear)}  ${q.kw.toFixed(1)} kW (${Math.round(q.kw * KW_TO_HP)} hp) · ` +
          `${Math.round(q.torque)} N·m · ${Math.round(q.kmh)} km/h`,
      );
    }
    this.tooltip.classList.remove('hidden');
    const tw = this.tooltip.offsetWidth;
    const flip = hx + 14 + tw > this.width;
    this.tooltip.style.left = `${flip ? hx - 14 - tw : hx + 14}px`;
    this.tooltip.style.top = `${tops[0]! + 4}px`;
  }

  /** Peak power and torque in the header, and the per-gear table. */
  private summarise(): void {
    const pts = this.smoothed;
    const pk = pts.reduce((b, q) => (q.kw > b.kw ? q : b));
    const tk = pts.reduce((b, q) => (q.torque > b.torque ? q : b));
    this.peaks.textContent =
      `Peak ${pk.kw.toFixed(1)} kW (${Math.round(pk.kw * KW_TO_HP)} hp) @ ${formatNumber(Math.round(pk.rpm))} rpm · ` +
      `${Math.round(tk.torque)} N·m @ ${formatNumber(Math.round(tk.rpm))} rpm`;

    this.table.replaceChildren();
    for (let g = 1; g <= GEAR_COLOURS.length; g++) {
      const inGear = pts.filter((q) => q.gear === g);
      if (inGear.length === 0) continue;
      const tr = el('tr', '', this.table);
      el('td', '', tr).textContent = ordinal(g);
      el('td', '', tr).textContent =
        `${Math.round(inGear[0]!.kmh)}–${Math.round(inGear[inGear.length - 1]!.kmh)}`;
      el('td', '', tr).textContent = Math.max(...inGear.map((q) => q.kw)).toFixed(1);
      el('td', '', tr).textContent = String(Math.round(Math.max(...inGear.map((q) => q.torque))));
    }
  }

  private resize(): void {
    const dpr = Math.min(window.devicePixelRatio, 2);
    const w = this.canvas.clientWidth;
    const h = this.canvas.clientHeight;
    if (w === 0 || h === 0) return;
    this.canvas.width = Math.round(w * dpr);
    this.canvas.height = Math.round(h * dpr);
    this.ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    this.width = w;
    this.height = h;
    this.dirty = true;
  }
}

/** Each point averaged with up to `SMOOTH` cycles either side of it in the same gear. */
function smooth(points: Point[]): Smoothed[] {
  return points.map((p, i) => {
    let torque = 0;
    let rpm = 0;
    let n = 0;
    for (let j = Math.max(i - SMOOTH, 0); j <= Math.min(i + SMOOTH, points.length - 1); j++) {
      if (points[j]!.gear !== p.gear) continue;
      torque += points[j]!.torque;
      rpm += points[j]!.rpm;
      n++;
    }
    torque /= n;
    rpm /= n;
    return { rpm, torque, kmh: p.kmh, gear: p.gear, kw: (torque * rpm * 2 * Math.PI) / 60 / 1000 };
  });
}

/** Gear `gear`'s reading at `rpm`, between its two nearest points, or `null` outside its pull. */
function interpolate(points: Smoothed[], gear: number, rpm: number): Smoothed | null {
  let prev: Smoothed | null = null;
  for (const q of points) {
    if (q.gear !== gear) continue;
    if (prev && prev.rpm <= rpm && q.rpm >= rpm) {
      const t = q.rpm > prev.rpm ? (rpm - prev.rpm) / (q.rpm - prev.rpm) : 0;
      const mix = (a: number, b: number) => a + (b - a) * t;
      return { rpm, gear, torque: mix(prev.torque, q.torque), kw: mix(prev.kw, q.kw), kmh: mix(prev.kmh, q.kmh) };
    }
    prev = q;
  }
  return null;
}

function niceStep(raw: number): number {
  const mag = Math.pow(10, Math.floor(Math.log10(raw)));
  const f = raw / mag;
  return (f <= 1 ? 1 : f <= 2 ? 2 : f <= 5 ? 5 : 10) * mag;
}

/** A top for the axis that four gridlines divide into round numbers. */
function niceMax(v: number): number {
  return niceStep(v / 4) * 4;
}

function formatNumber(v: number): string {
  return v.toLocaleString('en-US', { maximumFractionDigits: 0 });
}

function ordinal(n: number): string {
  return `${n}${n === 1 ? 'st' : n === 2 ? 'nd' : n === 3 ? 'rd' : 'th'}`;
}

function el(tag: string, cls: string, parent: HTMLElement): HTMLElement {
  const e = document.createElement(tag);
  if (cls) e.className = cls;
  parent.appendChild(e);
  return e;
}
