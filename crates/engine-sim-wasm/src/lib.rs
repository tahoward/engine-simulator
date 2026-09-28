//! The engine simulation for the web app's AudioWorklet, behind a plain C ABI.
//!
//! No wasm-bindgen: `AudioWorkletGlobalScope` has no `TextEncoder`, `TextDecoder` or `fetch`, which
//! its glue relies on. Everything crosses as numbers and as bytes in linear memory:
//!
//! - Configuration goes in as UTF-8 JSON in the web app's shapes. The caller reserves room with
//!   `alloc`, writes the bytes and passes the pointer and length; the call takes ownership and frees
//!   them.
//! - Audio comes out as `f32` samples in a buffer the simulation owns, valid until the next call.
//! - A snapshot comes out as UTF-8 JSON in a buffer the simulation owns, likewise.
//!
//! Any call can grow the memory, which detaches every view the caller holds on it, so views are
//! taken afresh after each call. A call that fails returns nonzero and leaves the reason in
//! `sim_error`.

use engine_sim::exhaust_graph::ExhaustGraph;
use engine_sim::{DynoConfig, EngineConfig, EngineSim};

pub struct Handle {
    sim: EngineSim,
    audio: Vec<f32>,
    snapshot: Vec<u8>,
    error: Vec<u8>,
}

/// Reserve `len` bytes for the caller to write a message into.
#[unsafe(no_mangle)]
pub extern "C" fn alloc(len: usize) -> *mut u8 {
    let mut v = Vec::<u8>::with_capacity(len.max(1));
    let p = v.as_mut_ptr();
    std::mem::forget(v);
    p
}

/// Free what `alloc` reserved, for a message that was never passed on.
///
/// # Safety
/// `ptr` and `len` must be exactly what `alloc` returned and was given.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn dealloc(ptr: *mut u8, len: usize) {
    drop(unsafe { Vec::from_raw_parts(ptr, 0, len.max(1)) });
}

/// Take back the bytes of a message the caller wrote with `alloc`.
unsafe fn take(ptr: *mut u8, len: usize) -> Vec<u8> {
    unsafe { Vec::from_raw_parts(ptr, len, len.max(1)) }
}

fn parse<T: serde::de::DeserializeOwned>(bytes: &[u8]) -> Result<T, String> {
    serde_json::from_slice(bytes).map_err(|e| e.to_string())
}

impl Handle {
    fn result(&mut self, r: Result<(), String>) -> i32 {
        match r {
            Ok(()) => 0,
            Err(e) => {
                self.error = e.into_bytes();
                1
            }
        }
    }
}

/// A simulation at `rate` Hz from an `EngineConfig`, or null if the config does not parse.
///
/// # Safety
/// `ptr` and `len` must come from `alloc`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn sim_new(rate: f64, ptr: *mut u8, len: usize) -> *mut Handle {
    let bytes = unsafe { take(ptr, len) };
    let Ok(config) = parse::<EngineConfig>(&bytes) else { return std::ptr::null_mut() };
    let sim = EngineSim::new(rate, &config);
    Box::into_raw(Box::new(Handle { sim, audio: Vec::new(), snapshot: Vec::new(), error: Vec::new() }))
}

/// # Safety
/// `h` must come from `sim_new` and not be used again.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn sim_free(h: *mut Handle) {
    if !h.is_null() {
        drop(unsafe { Box::from_raw(h) });
    }
}

/// Change the engine: a JSON object of the `EngineSpec` fields to change.
///
/// # Safety
/// `h` from `sim_new`; `ptr` and `len` from `alloc`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn sim_set_engine(h: *mut Handle, ptr: *mut u8, len: usize) -> i32 {
    let h = unsafe { &mut *h };
    let bytes = unsafe { take(ptr, len) };
    let r = parse::<serde_json::Value>(&bytes).and_then(|v| h.sim.set_engine_json(&v).map_err(|e| e.to_string()));
    h.result(r)
}

/// Replace the exhaust graph: an `ExhaustGraph`, or `null` to compile one from the layout.
///
/// # Safety
/// `h` from `sim_new`; `ptr` and `len` from `alloc`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn sim_set_graph(h: *mut Handle, ptr: *mut u8, len: usize) -> i32 {
    let h = unsafe { &mut *h };
    let bytes = unsafe { take(ptr, len) };
    let r = parse::<Option<ExhaustGraph>>(&bytes).map(|g| h.sim.set_graph(g));
    h.result(r)
}

/// Start a dyno run through a `DynoConfig`'s gearbox.
///
/// # Safety
/// `h` from `sim_new`; `ptr` and `len` from `alloc`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn sim_start_dyno(h: *mut Handle, ptr: *mut u8, len: usize) -> i32 {
    let h = unsafe { &mut *h };
    let bytes = unsafe { take(ptr, len) };
    let r = parse::<DynoConfig>(&bytes).map(|c| h.sim.start_dyno(c));
    h.result(r)
}

/// End the dyno run.
///
/// # Safety
/// `h` from `sim_new`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn sim_stop_dyno(h: *mut Handle) {
    unsafe { &mut *h }.sim.stop_dyno();
}

/// The operating point: throttle and load, 0..1.
///
/// # Safety
/// `h` from `sim_new`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn sim_set_controls(h: *mut Handle, throttle: f64, load: f64) {
    unsafe { &mut *h }.sim.set_controls(throttle, load);
}

/// Run at `scale` of real time: 1 is real time, less is slow motion.
///
/// # Safety
/// `h` from `sim_new`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn sim_set_time_scale(h: *mut Handle, scale: f64) {
    unsafe { &mut *h }.sim.set_time_scale(scale);
}

/// Render `n` samples; returns where they are.
///
/// # Safety
/// `h` from `sim_new`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn sim_render(h: *mut Handle, n: usize) -> *const f32 {
    let h = unsafe { &mut *h };
    if h.audio.len() < n {
        h.audio.resize(n, 0.0);
    }
    h.sim.render_into(&mut h.audio[..n]);
    h.audio.as_ptr()
}

/// Take a snapshot, as JSON; returns where it is, with its length from `sim_snapshot_len`.
///
/// # Safety
/// `h` from `sim_new`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn sim_snapshot(h: *mut Handle) -> *const u8 {
    let h = unsafe { &mut *h };
    let snap = h.sim.snapshot();
    h.snapshot.clear();
    serde_json::to_writer(&mut h.snapshot, &snap).expect("a snapshot serialises");
    h.snapshot.as_ptr()
}

/// # Safety
/// `h` from `sim_new`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn sim_snapshot_len(h: *mut Handle) -> usize {
    unsafe { &*h }.snapshot.len()
}

/// Why the last call that failed did, as UTF-8; its length from `sim_error_len`.
///
/// # Safety
/// `h` from `sim_new`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn sim_error(h: *mut Handle) -> *const u8 {
    unsafe { &*h }.error.as_ptr()
}

/// # Safety
/// `h` from `sim_new`.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn sim_error_len(h: *mut Handle) -> usize {
    unsafe { &*h }.error.len()
}
