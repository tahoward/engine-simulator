mod common;
use common::FS;
use engine_sim::EngineSim;
use serde_json::json;
use std::io::Write;
#[test]
fn render_lift() {
    let mut cfg = common::engine_preset("Inline four, Toyota 3S-GTE").config.clone();
    cfg.engine = common::with(&cfg.engine, json!({ "throttle": 1, "rpm": 4000, "blowOff": "none", "freeRunning": false, "combustionVariability": 0 }));
    let mut sim = EngineSim::new(FS, &cfg);
    sim.render(3 * FS as usize);
    let mut all: Vec<f32> = Vec::new();
    let mut turbo: Vec<f32> = Vec::new();
    let mut lifted = false;
    for i in 0..(3.0 * FS) as usize {
        if !lifted && i as f64 >= 1.0 * FS {
            sim.set_controls(0.0, 0.0); lifted = true;
            println!("lift at {:.2} s, rpm {:.0} boost {:.2}", i as f64 / FS, sim.snapshot().rpm, sim.turbo().unwrap().boost() / 1e5);
        }
        all.extend(sim.render(1));
        turbo.push(sim.turbo().unwrap().last_sound() as f32);
    }
    for (name, v) in [("/tmp/sim_all.f32", &all), ("/tmp/sim_turbo.f32", &turbo)] {
        let mut f = std::fs::File::create(name).unwrap();
        for s in v.iter() { f.write_all(&s.to_le_bytes()).unwrap(); }
    }
}
