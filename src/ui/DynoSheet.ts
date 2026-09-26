/**
 * The dyno sheet: crank power and torque against rpm, drawn live as a dyno run goes. Three charts on one
 * rpm axis: horsepower and pound-feet together, then kilowatts, newton-metres, and volumetric efficiency,
 * the fresh charge each cylinder traps as a share of its swept volume at ambient density.
 *
 * Horsepower and pound-feet share one chart, and not by convention alone. Horsepower is pound-feet times
 * rpm over 5252, so in these two units the curves share a scale and cross at 5252 rpm whatever the engine,
 * which is what lets them sit on one axis honestly: where each lies against the other means something. The
 * crossing is marked. Kilowatts and newton-metres have no such relation, so each has a chart of its own
 * rather than a second scale on one.
 *
 * On the top chart colour says which measure a line is; on the three below it, which gear, in the
 * palette's fixed order. The gears are the same engine, so their traces lie on top of one another where
 * they overlap; each is its own stretch of line, broken at every shift.
 *
 * Each point is one engine cycle's average, as a real dyno reports it. The line is smoothed over a
 * few cycles either side, because cycle-to-cycle combustion scatter is a few percent; the tooltip and
 * the peaks read the smoothed values too.
 */

import type { DynoConfig, DynoSnapshot } from '../model/spec.js';

/** Power and torque: the reference categorical palette's first two dark steps. */
const POWER = '#3987e5';
const TORQUE = '#d95926';
/**
 * One colour per gear, first to sixth, for the kilowatt and newton-metre charts: the same palette's
 * first six dark steps, in its fixed order.
 */
const GEAR_COLOURS = ['#3987e5', '#d95926', '#199e70', '#c98500', '#d55181', '#008300'];
const SURFACE = '#1a1e25';
const INK = '#e6eaf0';
const INK_DIM = '#8b95a5';
const GRID = 'rgba(255,255,255,0.06)';
const AXIS = 'rgba(255,255,255,0.14)';

/** Cycles either side of a point averaged into the line. */
const SMOOTH = 3;

/** Watts per mechanical horsepower, newton-metres per pound-foot, and km/h per mph. */
const W_PER_HP = 745.7;
const NM_PER_LBFT = 1.3558;
const KMH_PER_MPH = 1.609344;

/** The speed at which horsepower and pound-feet are equal, rpm: `5252 = 33000 / (2 pi)`. */
const CROSSOVER_RPM = 33000 / (2 * Math.PI);

interface Point {
  rpm: number;
  /** Crank torque, N*m, as the run measures it. */
  torque: number;
  kmh: number;
  gear: number;
  /** Volumetric efficiency, a fraction. */
  ve: number;
}

interface Smoothed {
  rpm: number;
  hp: number;
  lbft: number;
  kw: number;
  nm: number;
  /** Volumetric efficiency, %. */
  ve: number;
  mph: number;
  gear: number;
}

/** One of the stacked charts: the lines it draws, each a measure of a smoothed point. */
interface Pane {
  title: string;
  series: { colour: string; value: (q: Smoothed) => number; unit: string }[];
  /** Whether to mark where its two lines cross. */
  crossing: boolean;
  /** Whether each gear's stretch of line takes that gear's colour, rather than its series'. */
  byGear: boolean;
}

/** The colour a point of `series` is drawn in, on `pane`. */
function colourOf(pane: Pane, series: Pane['series'][number], q: Smoothed): string {
  return pane.byGear ? GEAR_COLOURS[(q.gear - 1) % GEAR_COLOURS.length]! : series.colour;
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
    this.canvas.setAttribute(
      'aria-label',
      'Crank horsepower and pound-feet of torque against engine speed on one axis, with kilowatts, newton-metres and volumetric efficiency below',
    );
    this.tooltip = el('div', 'dyno-tooltip hidden', plot);
    const ctx = this.canvas.getContext('2d');
    if (!ctx) throw new Error('DynoSheet: 2D canvas context unavailable');
    this.ctx = ctx;

    // Two legends: the measures for the top chart, the gears for the two below it.
    const legend = el('div', 'dyno-legend', this.card);
    for (const [colour, label] of [
      [POWER, 'Power, hp'],
      [TORQUE, 'Torque, lb·ft'],
    ] as const) {
      const item = el('span', '', legend);
      const swatch = el('i', '', item);
      swatch.style.background = colour;
      item.append(label);
    }
    const gears = el('div', 'dyno-legend', this.card);
    el('span', 'dyno-legend-title', gears).textContent = 'kW, N·m and VE by gear:';
    GEAR_COLOURS.forEach((colour, i) => {
      const item = el('span', '', gears);
      const swatch = el('i', '', item);
      swatch.style.background = colour;
      item.append(ordinal(i + 1));
    });

    const details = el('details', 'dyno-table', this.card);
    el('summary', '', details).textContent = 'Per gear';
    const tbl = el('table', '', details) as HTMLTableElement;
    const thead = el('thead', '', tbl);
    const hr = el('tr', '', thead);
    for (const h of ['Gear', 'mph', 'Peak hp', 'Peak lb·ft', 'Peak kW', 'Peak N·m', 'Peak VE']) {
      el('th', '', hr).textContent = h;
    }
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
    for (let i = 0; i + 4 < p.length; i += 5) {
      this.points.push({ rpm: p[i]!, torque: p[i + 1]!, kmh: p[i + 2]!, gear: p[i + 3]!, ve: p[i + 4]! });
    }
    if (p.length > 0) {
      this.smoothed = smooth(this.points);
      this.summarise();
      this.dirty = true;
    }
    const state = dyno.finished ? 'winding down' : PHASE_TEXT[dyno.phase];
    this.status.textContent =
      `${ordinal(dyno.gear)} gear · ${Math.round(dyno.speedKmh / KMH_PER_MPH)} mph · ` +
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
    const top = 8;
    const gap = 16;
    const axisBand = 22;
    // The combined chart gets the most room: it is the one whose crossing is the point.
    const shares = [0.34, 0.22, 0.22, 0.22];
    const usable = h - top - axisBand - (shares.length - 1) * gap;
    const heights = shares.map((f) => usable * f);
    const tops: number[] = [];
    let y = top;
    for (const hk of heights) {
      tops.push(y);
      y += hk + gap;
    }
    const x0 = this.minRpm;
    const x1 = Math.ceil((this.config.shiftRpm + 1) / 1000) * 1000;
    const xs = (rpm: number) => left + ((rpm - x0) / (x1 - x0)) * (w - left - right);
    const pts = this.smoothed;

    const panes: Pane[] = [
      {
        title: 'hp / lb·ft',
        series: [
          { colour: POWER, value: (q) => q.hp, unit: 'hp' },
          { colour: TORQUE, value: (q) => q.lbft, unit: 'lb·ft' },
        ],
        crossing: true,
        byGear: false,
      },
      {
        title: 'Power, kW',
        series: [{ colour: POWER, value: (q) => q.kw, unit: 'kW' }],
        crossing: false,
        byGear: true,
      },
      {
        title: 'Torque, N·m',
        series: [{ colour: TORQUE, value: (q) => q.nm, unit: 'N·m' }],
        crossing: false,
        byGear: true,
      },
      {
        title: 'Volumetric efficiency, %',
        series: [{ colour: POWER, value: (q) => q.ve, unit: '%' }],
        crossing: false,
        byGear: true,
      },
    ];
    const maxes = panes.map((pane) =>
      niceMax(Math.max(10, ...pts.flatMap((q) => pane.series.map((m) => m.value(q)))) * 1.08),
    );
    const scales = panes.map((_, k) => {
      const max = maxes[k]!;
      const paneTop = tops[k]!;
      const paneH = heights[k]!;
      return (v: number) => paneTop + paneH - (Math.max(v, 0) / max) * paneH;
    });

    ctx.font = '11px system-ui, sans-serif';
    panes.forEach((pane, k) =>
      this.drawPane(pane, xs, scales[k]!, maxes[k]!, tops[k]!, heights[k]!, left, right, pts),
    );

    // Shared rpm axis, and its gridlines through every pane.
    const last = panes.length - 1;
    const axisY = tops[last]! + heights[last]!;
    ctx.fillStyle = INK_DIM;
    ctx.textAlign = 'center';
    ctx.textBaseline = 'top';
    const step = niceStep((x1 - x0) / 6);
    for (let r = Math.ceil(x0 / step) * step; r <= x1; r += step) {
      const x = Math.round(xs(r)) + 0.5;
      ctx.strokeStyle = GRID;
      ctx.lineWidth = 1;
      panes.forEach((_, k) => {
        ctx.beginPath();
        ctx.moveTo(x, tops[k]!);
        ctx.lineTo(x, tops[k]! + heights[k]!);
        ctx.stroke();
      });
      ctx.fillText(formatNumber(r), x, axisY + 5);
    }
    ctx.textAlign = 'right';
    ctx.fillText('rpm', w - right, axisY + 5);

    this.drawHover(xs, panes, scales, x0, x1, left, right, tops[0]!, axisY);
  }

  /** One chart: its grid and scale, its lines broken at every shift, its peaks, and the crossing. */
  private drawPane(
    pane: Pane,
    xs: (rpm: number) => number,
    ys: (v: number) => number,
    max: number,
    top: number,
    height: number,
    left: number,
    right: number,
    pts: Smoothed[],
  ): void {
    const ctx = this.ctx;
    const w = this.width;
    const lines = height > 120 ? 5 : 3;
    ctx.textAlign = 'right';
    ctx.textBaseline = 'middle';
    for (let i = 0; i <= lines; i++) {
      const v = (max * i) / lines;
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

    ctx.lineWidth = 2;
    ctx.lineJoin = 'round';
    ctx.lineCap = 'round';
    // Broken at every shift; each stretch is stroked on its own, so it can take its gear's colour.
    for (const m of pane.series) {
      let prevGear = -1;
      for (const q of pts) {
        if (q.gear !== prevGear) {
          if (prevGear >= 0) ctx.stroke();
          ctx.strokeStyle = colourOf(pane, m, q);
          ctx.beginPath();
          ctx.moveTo(xs(q.rpm), ys(m.value(q)));
          prevGear = q.gear;
        } else {
          ctx.lineTo(xs(q.rpm), ys(m.value(q)));
        }
      }
      if (prevGear >= 0) ctx.stroke();
    }

    // Where they cross, which in horsepower and pound-feet is 5252 rpm.
    if (pane.crossing) {
      const cross = crossing(pts);
      if (cross) {
        this.marker(xs(cross.rpm), ys(cross.value), INK_DIM);
        ctx.fillStyle = INK_DIM;
        ctx.textAlign = 'center';
        ctx.textBaseline = 'top';
        ctx.fillText(`equal at ${formatNumber(Math.round(cross.rpm))}`, xs(cross.rpm), ys(cross.value) + 8);
      }
    }

    // Each peak, marked and labelled with its unit, which names its line.
    for (const m of pane.series) {
      const peak = pts.reduce<Smoothed | null>((b, q) => (!b || m.value(q) > m.value(b) ? q : b), null);
      if (!peak) continue;
      const px = xs(peak.rpm);
      const py = ys(m.value(peak));
      this.marker(px, py, colourOf(pane, m, peak));
      const value = Math.round(m.value(peak));
      const label = `${value}${m.unit === '%' ? '%' : ` ${m.unit}`} @ ${formatNumber(Math.round(peak.rpm))}`;
      ctx.fillStyle = INK;
      ctx.textBaseline = 'bottom';
      const tw = ctx.measureText(label).width;
      ctx.textAlign = px + 8 + tw > w - right ? 'right' : 'left';
      ctx.fillText(label, ctx.textAlign === 'right' ? px - 8 : px + 8, py - 4);
    }
  }

  /** A mark with a ring of the surface round it, so it stands off the line it sits on. */
  private marker(x: number, y: number, colour: string): void {
    const ctx = this.ctx;
    ctx.fillStyle = SURFACE;
    ctx.beginPath();
    ctx.arc(x, y, 6, 0, Math.PI * 2);
    ctx.fill();
    ctx.fillStyle = colour;
    ctx.beginPath();
    ctx.arc(x, y, 4, 0, Math.PI * 2);
    ctx.fill();
  }

  /** Crosshair through every chart, and a tooltip with every gear's reading at that speed. */
  private drawHover(
    xs: (rpm: number) => number,
    panes: Pane[],
    scales: ((v: number) => number)[],
    x0: number,
    x1: number,
    left: number,
    right: number,
    top: number,
    bottom: number,
  ): void {
    const hx = this.hoverX;
    if (hx === null || hx < left || hx > this.width - right || this.smoothed.length === 0) {
      this.tooltip.classList.add('hidden');
      return;
    }
    const rpm = x0 + ((hx - left) / (this.width - left - right)) * (x1 - x0);
    const rows: Smoothed[] = [];
    for (let g = 1; g <= 6; g++) {
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
    ctx.moveTo(x, top);
    ctx.lineTo(x, bottom);
    ctx.stroke();
    for (const q of rows) {
      panes.forEach((pane, k) => {
        for (const m of pane.series) this.marker(x, scales[k]!(m.value(q)), colourOf(pane, m, q));
      });
    }

    this.tooltip.replaceChildren();
    el('div', 'dyno-tip-head', this.tooltip).textContent = `${formatNumber(Math.round(rpm))} rpm`;
    for (const q of rows) {
      el('div', 'dyno-tip-row', this.tooltip).textContent =
        `${ordinal(q.gear)}  ${Math.round(q.hp)} hp · ${Math.round(q.lbft)} lb·ft · ` +
        `${Math.round(q.kw)} kW · ${Math.round(q.nm)} N·m · VE ${Math.round(q.ve)}% · ${Math.round(q.mph)} mph`;
    }
    this.tooltip.classList.remove('hidden');
    const tw = this.tooltip.offsetWidth;
    const flip = hx + 14 + tw > this.width;
    this.tooltip.style.left = `${flip ? hx - 14 - tw : hx + 14}px`;
    this.tooltip.style.top = `${top + 4}px`;
  }

  /** Peak power and torque in the header, and the per-gear table. */
  private summarise(): void {
    const pts = this.smoothed;
    const pk = pts.reduce((b, q) => (q.hp > b.hp ? q : b));
    const tk = pts.reduce((b, q) => (q.lbft > b.lbft ? q : b));
    this.peaks.textContent =
      `Peak ${Math.round(pk.hp)} hp (${Math.round(pk.kw)} kW) @ ${formatNumber(Math.round(pk.rpm))} rpm · ` +
      `${Math.round(tk.lbft)} lb·ft (${Math.round(tk.nm)} N·m) @ ${formatNumber(Math.round(tk.rpm))} rpm`;

    this.table.replaceChildren();
    for (let g = 1; g <= 6; g++) {
      const inGear = pts.filter((q) => q.gear === g);
      if (inGear.length === 0) continue;
      const tr = el('tr', '', this.table);
      el('td', '', tr).textContent = ordinal(g);
      el('td', '', tr).textContent =
        `${Math.round(inGear[0]!.mph)}–${Math.round(inGear[inGear.length - 1]!.mph)}`;
      el('td', '', tr).textContent = String(Math.round(Math.max(...inGear.map((q) => q.hp))));
      el('td', '', tr).textContent = String(Math.round(Math.max(...inGear.map((q) => q.lbft))));
      el('td', '', tr).textContent = String(Math.round(Math.max(...inGear.map((q) => q.kw))));
      el('td', '', tr).textContent = String(Math.round(Math.max(...inGear.map((q) => q.nm))));
      el('td', '', tr).textContent = `${Math.round(Math.max(...inGear.map((q) => q.ve)))}%`;
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

/**
 * Each point averaged with up to `SMOOTH` cycles either side of it in the same gear, and put in
 * horsepower, pound-feet and mph.
 */
function smooth(points: Point[]): Smoothed[] {
  return points.map((p, i) => {
    let torque = 0;
    let rpm = 0;
    let ve = 0;
    let n = 0;
    for (let j = Math.max(i - SMOOTH, 0); j <= Math.min(i + SMOOTH, points.length - 1); j++) {
      if (points[j]!.gear !== p.gear) continue;
      torque += points[j]!.torque;
      rpm += points[j]!.rpm;
      ve += points[j]!.ve;
      n++;
    }
    torque /= n;
    rpm /= n;
    ve /= n;
    const watts = (torque * rpm * 2 * Math.PI) / 60;
    return {
      rpm,
      hp: watts / W_PER_HP,
      lbft: torque / NM_PER_LBFT,
      kw: watts / 1000,
      nm: torque,
      ve: ve * 100,
      mph: p.kmh / KMH_PER_MPH,
      gear: p.gear,
    };
  });
}

/**
 * Where power and torque cross, if the run passed 5252 rpm in any gear: the first place, going up the
 * run, where one overtakes the other, found between the two points either side of it.
 */
function crossing(points: Smoothed[]): { rpm: number; value: number } | null {
  if (!points.some((q) => q.rpm >= CROSSOVER_RPM)) return null;
  for (let i = 1; i < points.length; i++) {
    const a = points[i - 1]!;
    const b = points[i]!;
    if (a.gear !== b.gear) continue;
    const da = a.hp - a.lbft;
    const db = b.hp - b.lbft;
    if (da <= 0 && db >= 0 && db !== da) {
      const t = -da / (db - da);
      return { rpm: a.rpm + (b.rpm - a.rpm) * t, value: a.hp + (b.hp - a.hp) * t };
    }
  }
  return null;
}

/** Gear `gear`'s reading at `rpm`, between its two nearest points, or `null` outside its pull. */
function interpolate(points: Smoothed[], gear: number, rpm: number): Smoothed | null {
  let prev: Smoothed | null = null;
  for (const q of points) {
    if (q.gear !== gear) continue;
    if (prev && prev.rpm <= rpm && q.rpm >= rpm) {
      const t = q.rpm > prev.rpm ? (rpm - prev.rpm) / (q.rpm - prev.rpm) : 0;
      const mix = (a: number, b: number) => a + (b - a) * t;
      return {
        rpm,
        gear,
        hp: mix(prev.hp, q.hp),
        lbft: mix(prev.lbft, q.lbft),
        kw: mix(prev.kw, q.kw),
        nm: mix(prev.nm, q.nm),
        ve: mix(prev.ve, q.ve),
        mph: mix(prev.mph, q.mph),
      };
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

/** A top for an axis that five gridlines, or three, divide into round numbers. */
function niceMax(v: number): number {
  return niceStep(v / 15) * 15;
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
