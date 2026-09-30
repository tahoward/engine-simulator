//! The engine simulation: cylinders, valves, intake and exhaust gas dynamics and radiation, stepped
//! one audio sample at a time. No audio I/O and no platform dependencies, so the same code runs
//! natively in the desktop app and as Wasm in the web app's AudioWorklet.

// Kept as they are: index loops that read like the formulas they compute, `!(x > y)` where it is
// how a NaN is let through or kept out, and `math::clamp`, which passes NaN where `f64::clamp` would
// not and never panics.
#![allow(clippy::needless_range_loop, clippy::neg_cmp_op_on_partial_ord, clippy::manual_clamp, clippy::collapsible_if)]

pub mod afterfire;
pub mod cross_modes;
pub mod cylinder;
pub mod drivetrain;
pub mod dsp;
pub mod engine_sim;
pub mod euler_pipe;
pub mod exhaust_graph;
pub mod exhaust_system;
pub mod geometry;
pub mod intake;
pub mod listener;
pub mod math;
mod math_tables;
pub mod plenum;
pub mod pow;
pub mod radiation;
pub mod shell;
pub mod simd;
pub mod spec;
pub mod turbo;
pub mod valve;

pub use engine_sim::EngineSim;
pub use spec::{EngineConfig, EngineSnapshot, EngineSpec, LaunchConfig, PipeSegment};
