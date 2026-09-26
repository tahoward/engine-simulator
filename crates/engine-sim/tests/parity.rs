//! The simulation's reference renders, replayed bit for bit: every 10 ms block of audio, every
//! snapshot, and every compiled graph.
//!
//! `tests/fixtures/scenarios.json` holds a few dozen scenarios, each a config and a list of steps
//! (render, change the controls, edit the engine, swap the exhaust, run the dyno, take a snapshot),
//! with the audio as a hash per block, its first samples raw, and every snapshot. Any change to what
//! the simulation computes, down to the last bit of one sample, fails here, so a change that is meant
//! to be a pure refactor is proven to be one. The web app's `test/wasm.test.ts` replays the same file
//! through the Wasm build, which is what keeps the desktop and web builds sounding the same.
//!
//! When a change is meant to alter the sound, run with `PARITY_BLESS=1` to write the new results into
//! the fixture, and say why in the change.

use engine_sim::euler_pipe::EulerPipeOptions;
use engine_sim::exhaust_graph::{ExhaustGraph, compile_exhaust};
use engine_sim::{DynoConfig, EngineConfig, EngineSim};
use serde::Deserialize;
use serde_json::Value;

#[derive(Deserialize)]
struct Fixture {
    #[serde(rename = "blockSize")]
    block_size: usize,
    scenarios: Vec<Scenario>,
    compiled: Vec<Compiled>,
}

#[derive(Deserialize)]
#[serde(rename_all = "camelCase")]
struct Scenario {
    name: String,
    sample_rate: f64,
    config: EngineConfig,
    steps: Vec<Value>,
    result: Expected,
}

#[derive(Deserialize)]
struct Expected {
    blocks: Vec<String>,
    head: Vec<f32>,
    snapshots: Vec<Value>,
}

#[derive(Deserialize)]
struct Compiled {
    name: String,
    config: EngineConfig,
    graph: Value,
}

fn fixture() -> Fixture {
    let text = std::fs::read_to_string(concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/scenarios.json"))
        .expect("fixtures: run `npm run capture` in apps/web");
    serde_json::from_str(&text).expect("fixture parses")
}

fn fnv(samples: &[f32]) -> String {
    let mut h: u32 = 0x811c9dc5;
    for s in samples {
        for b in s.to_le_bytes() {
            h ^= b as u32;
            h = h.wrapping_mul(0x01000193);
        }
    }
    format!("{h:08x}")
}

/// The first place two JSON values differ, numbers compared as exact doubles.
fn first_difference(path: &str, a: &Value, b: &Value) -> Option<String> {
    match (a, b) {
        (Value::Number(x), Value::Number(y)) => {
            let (x, y) = (x.as_f64().unwrap(), y.as_f64().unwrap());
            if x.to_bits() == y.to_bits() || x == y { None } else { Some(format!("{path}: {x} vs {y}")) }
        }
        (Value::Array(x), Value::Array(y)) => {
            if x.len() != y.len() {
                return Some(format!("{path}: length {} vs {}", x.len(), y.len()));
            }
            x.iter().zip(y).enumerate().find_map(|(i, (p, q))| first_difference(&format!("{path}[{i}]"), p, q))
        }
        (Value::Object(x), Value::Object(y)) => {
            for (k, v) in x {
                match y.get(k) {
                    Some(w) => {
                        if let Some(d) = first_difference(&format!("{path}.{k}"), v, w) {
                            return Some(d);
                        }
                    }
                    None => return Some(format!("{path}.{k}: missing")),
                }
            }
            for k in y.keys() {
                if !x.contains_key(k) {
                    return Some(format!("{path}.{k}: unexpected"));
                }
            }
            None
        }
        _ => {
            if a == b {
                None
            } else {
                Some(format!("{path}: {a} vs {b}"))
            }
        }
    }
}

/// What a scenario renders: its audio and its snapshots.
fn render(s: &Scenario) -> (Vec<f32>, Vec<Value>) {
    let mut sim = EngineSim::with_options(s.sample_rate, &s.config, EulerPipeOptions::default(), None);
    let mut audio: Vec<f32> = Vec::new();
    let mut snapshots: Vec<Value> = Vec::new();
    for step in &s.steps {
        let obj = step.as_object().unwrap();
        if let Some(n) = obj.get("render") {
            audio.extend(sim.render(n.as_u64().unwrap() as usize));
        } else if let Some(c) = obj.get("controls") {
            sim.set_controls(c[0].as_f64().unwrap(), c[1].as_f64().unwrap());
        } else if let Some(e) = obj.get("engine") {
            sim.set_engine_json(e).unwrap();
        } else if let Some(g) = obj.get("graph") {
            let graph: Option<ExhaustGraph> = serde_json::from_value(g.clone()).unwrap();
            sim.set_graph(graph);
        } else if let Some(d) = obj.get("dyno") {
            let config: Option<DynoConfig> = serde_json::from_value(d.clone()).unwrap();
            match config {
                Some(c) => sim.start_dyno(c),
                None => sim.stop_dyno(),
            }
        } else if obj.contains_key("snapshot") {
            snapshots.push(serde_json::to_value(sim.snapshot()).unwrap());
        }
    }
    (audio, snapshots)
}

fn run(s: &Scenario, block: usize) -> Result<(), String> {
    let (audio, snapshots) = render(s);
    let blocks: Vec<String> = audio.chunks(block).map(fnv).collect();
    if blocks.len() != s.result.blocks.len() {
        return Err(format!("{} blocks, expected {}", blocks.len(), s.result.blocks.len()));
    }
    if let Some(k) = (0..blocks.len()).find(|&k| blocks[k] != s.result.blocks[k]) {
        let head = s.result.head.len().min(audio.len());
        let first = (0..head).find(|&i| audio[i].to_bits() != s.result.head[i].to_bits());
        let detail = match first {
            Some(i) => {
                format!("; first differing sample {i}: {} vs {} (of the first {head})", audio[i], s.result.head[i])
            }
            None => format!("; the first {head} samples match"),
        };
        return Err(format!("block {k} of {} differs{detail}", blocks.len()));
    }
    for (i, (got, want)) in snapshots.iter().zip(&s.result.snapshots).enumerate() {
        if let Some(d) = first_difference("", got, want) {
            return Err(format!("snapshot {i} differs at {d}"));
        }
    }
    Ok(())
}

#[test]
fn every_scenario_renders_bit_for_bit() {
    if std::env::var("PARITY_BLESS").is_ok() {
        bless();
        return;
    }
    let f = fixture();
    let only = std::env::var("PARITY_ONLY").ok();
    let mut failures = Vec::new();
    for s in &f.scenarios {
        if only.as_deref().is_some_and(|o| !s.name.contains(o)) {
            continue;
        }
        match run(s, f.block_size) {
            Ok(()) => println!("ok    {}", s.name),
            Err(e) => {
                println!("FAIL  {}: {e}", s.name);
                failures.push(s.name.clone());
            }
        }
    }
    assert!(failures.is_empty(), "{} scenarios differ: {failures:?}", failures.len());
}

/// Rewrite every scenario's expected results, and every compiled graph, from what the simulation
/// computes now.
fn bless() {
    let path = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures/scenarios.json");
    let mut raw: Value = serde_json::from_str(&std::fs::read_to_string(path).unwrap()).unwrap();
    let f = fixture();
    let block = f.block_size;
    for (i, s) in f.scenarios.iter().enumerate() {
        let (audio, snapshots) = render(s);
        let head: Vec<f32> = audio.iter().copied().take(s.result.head.len()).collect();
        raw["scenarios"][i]["result"] = serde_json::json!({
            "blocks": audio.chunks(block).map(fnv).collect::<Vec<_>>(),
            "head": head,
            "snapshots": snapshots,
        });
    }
    for (i, c) in f.compiled.iter().enumerate() {
        let graph = compile_exhaust(&c.config.engine, &c.config.pipe, &c.config.collector);
        raw["compiled"][i]["graph"] = serde_json::to_value(&graph).unwrap();
    }
    std::fs::write(path, serde_json::to_string(&raw).unwrap()).unwrap();
    println!("blessed {} scenarios and {} graphs", f.scenarios.len(), f.compiled.len());
}

/// Segment ids come from a counter on each side, so they are left out of the comparison.
fn without_segment_ids(mut v: Value) -> Value {
    if let Some(ducts) = v.get_mut("ducts").and_then(|d| d.as_array_mut()) {
        for d in ducts {
            if let Some(segs) = d.get_mut("segments").and_then(|s| s.as_array_mut()) {
                for s in segs {
                    s.as_object_mut().unwrap().remove("id");
                }
            }
        }
    }
    v
}

#[test]
fn every_preset_compiles_to_the_same_graph() {
    let f = fixture();
    let mut failures = Vec::new();
    for c in &f.compiled {
        let graph = compile_exhaust(&c.config.engine, &c.config.pipe, &c.config.collector);
        let got = without_segment_ids(serde_json::to_value(&graph).unwrap());
        let want = without_segment_ids(c.graph.clone());
        if let Some(d) = first_difference("", &got, &want) {
            println!("FAIL  {}: {d}", c.name);
            failures.push(c.name.clone());
        }
    }
    assert!(failures.is_empty(), "{failures:?}");
}
