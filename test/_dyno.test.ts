import { describe, it } from 'vitest';
import { ENGINE_PRESETS, defaultConfig, defaultCollector, fitDyno, type EngineSpec } from '../src/model/spec.js';
import { EngineSim } from '../src/audio/worklet/engineSim.js';
declare const process: { stdout: { write(s: string): void } };
const FS = 48000;
describe('dyno', () => { it('run', () => {
  for (const name of ['Inline four', 'Boxer four', 'Single, megaphone', 'V8, crossplane, manifold per bank']) {
    const p = ENGINE_PRESETS.find((x) => x.name === name)!;
    const cfg = defaultConfig();
    cfg.engine = { ...cfg.engine, ...p.engine } as EngineSpec;
    cfg.pipe = p.pipe(); cfg.collector = p.collector ? p.collector() : defaultCollector();
    const sim = new EngineSim(FS, cfg); sim.render(FS);
    const dc = fitDyno(cfg.engine);
    sim.startDyno(dc);
    const pts: number[][] = []; let t = 0; let end = 0; let lastPhase = ''; const shifts: string[] = [];
    while (t < 200) {
      sim.render(FS / 60); t += 1 / 60;
      const s = sim.snapshot();
      if (!s.dyno) { end = t; break; }
      for (let i = 0; i < s.dyno.points.length; i += 4) pts.push([...s.dyno.points.slice(i, i + 4)]);
      if (s.dyno.phase !== lastPhase && s.dyno.phase === 'shiftOut') shifts.push(`${s.dyno.gear}@${s.rpm.toFixed(0)}/${s.dyno.speedKmh.toFixed(0)}kmh/${s.dyno.elapsed.toFixed(1)}s`);
      lastPhase = s.dyno.phase;
    }
    let pk = [0, 0], tk = [0, 0];
    for (const [rpm, tq] of pts) { const kw = tq * rpm * 2 * Math.PI / 60 / 1000; if (kw > pk[0]!) pk = [kw, rpm]; if (tq > tk[0]!) tk = [tq, rpm]; }
    process.stdout.write(`\nDYNO ${name}: mass ${dc.mass.toFixed(0)} fd ${dc.finalDrive.toFixed(2)} shift ${dc.shiftRpm}  points ${pts.length}  end ${end.toFixed(1)}s  rpm after ${sim.rpm.toFixed(0)}  peak ${pk[0]!.toFixed(0)}kW@${pk[1]!.toFixed(0)} ${tk[0]!.toFixed(0)}Nm@${tk[1]!.toFixed(0)}\n   shifts ${shifts.join(' ')}\n   first ${pts[0]?.map((v) => v.toFixed(0))} last ${pts[pts.length - 1]?.map((v) => v.toFixed(0))}`);
  }
}); });
