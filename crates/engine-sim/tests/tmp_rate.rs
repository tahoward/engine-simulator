mod common;
use common::FS;
use engine_sim::EngineSim;
use serde_json::json;
#[test]
fn rate() {
    let mut cfg = common::engine_preset(&std::env::var("PRESET").unwrap()).config.clone();
    cfg.engine = common::with(&cfg.engine, json!({ "throttle": 1, "rpm": std::env::var("RPM").unwrap().parse::<f64>().unwrap(), "blowOff": "none", "freeRunning": false, "combustionVariability": 0 }));
    let mut sim = EngineSim::new(FS, &cfg);
    sim.render(3 * FS as usize);
    sim.set_controls(0.0, 0.0);
    let mut prev = 1.0; let mut times = Vec::new(); let mut lo: f64 = 0.0; let mut f0 = None; let mut fell = None;
    for i in 0..FS as usize {
        sim.render(1);
        let f = sim.turbo().unwrap().compressor_flow();
        if prev >= 0.0 && f < 0.0 { times.push(i as f64 / FS); }
        prev = f; lo = lo.min(f); let b = *f0.get_or_insert(f); if fell.is_none() && f < 0.5 * b { fell = Some(i as f64 / FS * 1000.0); }
    }
    let first: Vec<i64> = times.iter().take(10).map(|t| (t * 1000.0) as i64).collect();
    let rate = if times.len() > 3 { (times.len().min(8) - 1) as f64 / (times[times.len().min(8) - 1] - times[0]) } else { 0.0 };
    println!("FELL {:?} ms RATE", fell); println!("RATE {rate:.1} Hz, {} reversals in 1 s, min flow {lo:.3}, first at {:?}", times.len(), first);
}
