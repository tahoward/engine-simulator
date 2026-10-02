mod common;
use common::FS;
use engine_sim::EngineSim;
fn run(name: &str) {
    let preset = common::engine_preset(name);
    let mut cfg = preset.config.clone();
    if let Ok(v) = std::env::var("TN") { cfg.engine = common::with(&cfg.engine, serde_json::json!({ "turboNoise": v.parse::<f64>().unwrap() })); }
    if let Ok(v) = std::env::var("BOV") { cfg.engine = common::with(&cfg.engine, serde_json::json!({ "blowOff": v })); }
    let mut sim = EngineSim::new(FS, &cfg);
    sim.render(FS as usize);
    let before: Vec<f32> = sim.render(FS as usize / 4);
    sim.start_launch(preset.launch.clone());
    let out: Vec<f32> = sim.render((0.1 * FS) as usize);
    let peak = |v: &[f32]| v.iter().fold(0.0f32, |m, x| m.max(x.abs()));
    let (i, p) = out.iter().enumerate().fold((0, 0.0f32), |a, (i, x)| if x.abs() > a.1 { (i, x.abs()) } else { a });
    let ms: Vec<i32> = out.chunks(96).map(|c| (20.0 * (peak(c) / peak(&before)).log10()) as i32).collect();
    println!("{name}: peak {:.4} ({:+.0} dB re idle) at sample {i}; per 2 ms dB: {:?}", p, 20.0 * (p / peak(&before)).log10(), ms);
    let mut f = std::fs::File::create(format!("/tmp/pop_{}.f32", name.split(',').next().unwrap().replace(' ', "_"))).unwrap();
    use std::io::Write; for x in before.iter().chain(out.iter()) { f.write_all(&x.to_le_bytes()).unwrap(); }
}
#[test]
fn pop() { for n in ["Inline six, Nissan RB26DETT", "Inline four, Toyota 3S-GTE", "Inline four, Honda F20C"] { run(n); } }
