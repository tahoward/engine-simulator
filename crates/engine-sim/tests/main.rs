//! The physics tests, as one test binary so that every test shares the machine's cores and the crate
//! is linked once. Each module is one area of the model; `common` holds what they share.
//!
//! The sweeps over the engine presets run on the LT6, the 3S-GTE and the 6CT: `common::SWEPT`.

mod common;

mod afterfire;
mod combustion;
mod controls;
mod cross_modes;
mod cylinder;
mod diesel;
mod dsp;
mod dyno;
mod engine;
mod engine8;
mod euler_pipe;
mod exhaust_graph;
mod idle;
mod ignition;
mod inlet;
mod intake;
mod jet;
mod junction;
mod launch;
mod parity;
mod realism;
mod room;
mod shell;
mod steepening;
mod stereo;
mod surfaces;
mod timbre;
mod turbo;
mod twin;
mod vtec;
