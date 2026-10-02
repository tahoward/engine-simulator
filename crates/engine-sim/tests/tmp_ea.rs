mod common;
use common::FS;
use engine_sim::EngineSim;
use serde_json::json;
#[test]
fn ea() {
    for (bov, free) in [("none", false), ("none", true)] {
        let mut cfg = common::engine_preset(&std::env::var("PRESET").unwrap()).config.clone();
        cfg.engine = common::with(&cfg.engine, json!({ "blowOff": bov, "combustionVariability": 0 }));
        let mut sim;
        if free {
            cfg.engine = common::with(&cfg.engine, json!({ "throttle": 0, "rpm": 900, "freeRunning": true }));
            sim = EngineSim::new(FS, &cfg); sim.render(2 * FS as usize); sim.set_controls(1.0, 0.0);
            sim.render((1.0 * FS) as usize);
        } else {
            cfg.engine = common::with(&cfg.engine, json!({ "throttle": 1, "rpm": 4500, "freeRunning": false }));
            sim = EngineSim::new(FS, &cfg); sim.render(3 * FS as usize);
        }
        let rpm = sim.snapshot().rpm; let t = sim.turbo().unwrap();
        println!("{bov} free={free}: before lift rpm {:.0} boost {:.2} shaft {:.0}", rpm, t.boost()/1e5, t.shaft_rpm());
        sim.set_controls(0.0, 0.0);
        let mut prev = 1.0; let mut rev = Vec::new(); let mut lo: f64 = 0.0;
        for i in 0..(2.5 * FS) as usize {
            sim.render(1);
            let rpm = sim.snapshot().rpm; let t = sim.turbo().unwrap(); let f = t.compressor_flow();
            if prev >= 0.0 && f < 0.0 { rev.push((i as f64 / FS * 1000.0) as i64); }
            prev = f; lo = lo.min(f);
            if std::env::var("QUIET").is_err() && i % 4800 == 0 { println!("  t {:.1} rpm {:.0} shaft {:.0} boost {:+.3} flow {:+.3} stall {:.2} hump-ish s", i as f64 / FS, rpm, t.shaft_rpm(), t.boost()/1e5, f, t.stall()); }
        }
        println!("  {bov} free={free} min flow {lo:.3} reversals at ms: {:?}", rev);
    }
}
