/**
 * Waveform, spectrum and pressure-along-pipe display.
 *
 * Three stacked panes on one canvas. The spectrum is drawn on a log frequency axis
 * because that is how the exhaust resonances are spaced and how hearing works; a
 * linear axis crams every interesting harmonic into the leftmost eighth.
 */

import type { AudioEngine } from '../audio/AudioEngine.js';
import type { EngineSnapshot } from '../model/spec.js';

const INK = '#8b95a5';
const GRID = 'rgba(255,255,255,0.06)';
const WAVE = '#4fd1ff';
const SPECTRUM = '#ffb347';
const PRESSURE_POS = '#ff8a4c';
const PRESSURE_NEG = '#4fd1ff';

const MIN_HZ = 20;
const MAX_HZ = 12000;

export class Scope {
  private readonly ctx: CanvasRenderingContext2D;
  private waveBuf = new Float32Array(2048);
  private specBuf = new Float32Array(1024);
  private pressure: Float32Array | null = null;
  /** Auto-ranging pressure scale, Pa, matching the pipe colouring. */
  private pressureScale = 8000;
  private width = 0;
  private height = 0;

  constructor(
    private readonly canvas: HTMLCanvasElement,
    private readonly audio: AudioEngine,
  ) {
    const ctx = canvas.getContext('2d');
    if (!ctx) throw new Error('Scope: 2D canvas context unavailable');
    this.ctx = ctx;
    this.resize();
    window.addEventListener('resize', this.resize);
  }

  private resize = (): void => {
    const dpr = Math.min(window.devicePixelRatio, 2);
    const w = this.canvas.clientWidth || 1;
    const h = this.canvas.clientHeight || 1;
    this.canvas.width = Math.round(w * dpr);
    this.canvas.height = Math.round(h * dpr);
    this.ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    this.width = w;
    this.height = h;
  };

  onSnapshot(s: EngineSnapshot): void {
    this.pressure = s.pipePressure;
    let peak = 0;
    for (const v of s.pipePressure) peak = Math.max(peak, Math.abs(v));
    this.pressureScale =
      peak > this.pressureScale
        ? peak
        : Math.max(1500, this.pressureScale * 0.985 + peak * 0.015);
  }

  draw(): void {
    const { ctx, width: w, height: h } = this;
    ctx.clearRect(0, 0, w, h);

    if (this.specBuf.length !== this.audio.spectrumSize) {
      this.specBuf = new Float32Array(this.audio.spectrumSize);
    }

    // The spectrum gets the most room: it is where the pipe's resonances are legible,
    // and it needs vertical range to show a 100 dB span.
    const wave = h * 0.26;
    const spectrum = h * 0.44;
    this.drawWave(0, wave);
    this.drawSpectrum(wave, spectrum);
    this.drawPressure(wave + spectrum, h - wave - spectrum);
  }

  private label(text: string, x: number, y: number): void {
    const { ctx } = this;
    ctx.fillStyle = INK;
    ctx.font = '10px ui-monospace, SFMono-Regular, Menlo, monospace';
    ctx.fillText(text, x, y);
  }

  private drawWave(top: number, h: number): void {
    const { ctx, width: w } = this;
    const mid = top + h / 2;

    ctx.strokeStyle = GRID;
    ctx.lineWidth = 1;
    ctx.beginPath();
    ctx.moveTo(0, mid);
    ctx.lineTo(w, mid);
    ctx.stroke();

    if (this.audio.readWaveform(this.waveBuf)) {
      ctx.strokeStyle = WAVE;
      ctx.lineWidth = 1.25;
      ctx.beginPath();
      const n = this.waveBuf.length;
      for (let i = 0; i < n; i++) {
        const x = (i / (n - 1)) * w;
        const y = mid - this.waveBuf[i]! * (h / 2) * 0.92;
        if (i === 0) ctx.moveTo(x, y);
        else ctx.lineTo(x, y);
      }
      ctx.stroke();
    }
    this.label('pressure at the listener', 6, top + 12);
  }

  private drawSpectrum(top: number, h: number): void {
    const { ctx, width: w } = this;
    const bottom = top + h - 2;

    // Decade grid.
    ctx.strokeStyle = GRID;
    ctx.lineWidth = 1;
    for (const hz of [50, 100, 200, 500, 1000, 2000, 5000, 10000]) {
      const x = this.hzToX(hz, w);
      ctx.beginPath();
      ctx.moveTo(x, top);
      ctx.lineTo(x, bottom);
      ctx.stroke();
    }

    if (this.audio.readSpectrum(this.specBuf)) {
      const nyquist = this.audio.sampleRate / 2;
      const bins = this.specBuf.length;
      ctx.strokeStyle = SPECTRUM;
      ctx.lineWidth = 1.25;
      ctx.beginPath();
      let started = false;
      for (let i = 1; i < bins; i++) {
        const hz = (i / bins) * nyquist;
        if (hz < MIN_HZ || hz > MAX_HZ) continue;
        const x = this.hzToX(hz, w);
        // getFloatFrequencyData is already dBFS; map -100..0 dB onto the pane.
        const db = Math.max(-100, Math.min(0, this.specBuf[i]!));
        const y = bottom - ((db + 100) / 100) * (h - 14);
        if (!started) {
          ctx.moveTo(x, y);
          started = true;
        } else ctx.lineTo(x, y);
      }
      ctx.stroke();
    }
    this.label('spectrum  20 Hz – 12 kHz (log)', 6, top + 12);
  }

  private hzToX(hz: number, w: number): number {
    const t =
      (Math.log(hz) - Math.log(MIN_HZ)) / (Math.log(MAX_HZ) - Math.log(MIN_HZ));
    return t * w;
  }

  private drawPressure(top: number, h: number): void {
    const { ctx, width: w } = this;
    const mid = top + h / 2;

    ctx.strokeStyle = GRID;
    ctx.beginPath();
    ctx.moveTo(0, mid);
    ctx.lineTo(w, mid);
    ctx.stroke();

    const p = this.pressure;
    if (p && p.length > 1) {
      const scale = (h / 2) * 0.86;
      // Filled area, split by sign: compression above the line, rarefaction below.
      for (const positive of [true, false]) {
        ctx.beginPath();
        ctx.moveTo(0, mid);
        for (let i = 0; i < p.length; i++) {
          const x = (i / (p.length - 1)) * w;
          const v = p[i]! / this.pressureScale;
          const clipped = positive ? Math.max(v, 0) : Math.min(v, 0);
          ctx.lineTo(x, mid - clipped * scale);
        }
        ctx.lineTo(w, mid);
        ctx.closePath();
        ctx.fillStyle = positive ? PRESSURE_POS : PRESSURE_NEG;
        ctx.globalAlpha = 0.5;
        ctx.fill();
        ctx.globalAlpha = 1;
      }
    }
    this.label(
      `pressure along the pipe · port → mouth · ±${(this.pressureScale / 1000).toFixed(1)} kPa`,
      6,
      top + 12,
    );
  }

  dispose(): void {
    window.removeEventListener('resize', this.resize);
  }
}
