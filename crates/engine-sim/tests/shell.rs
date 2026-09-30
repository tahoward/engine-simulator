//! The walls of a muffler can: where they ring, and that they radiate only what changes.

mod common;

use common::FS;
use engine_sim::euler_pipe::{EulerPipe, EulerPipeOptions};
use engine_sim::shell::ChamberShell;
use engine_sim::spec::{ChamberSection, PipeSegment, SegmentKind, SegmentPartial, gas, make_segment};

fn pipe(length: f64, d: f64) -> PipeSegment {
    make_segment(SegmentPartial { length: Some(length), d_in: Some(d), ..Default::default() })
}

/// A can of `section`, `width` by `height`, between two pipes.
fn can(section: ChamberSection, width: f64, height: f64) -> Vec<PipeSegment> {
    vec![
        pipe(0.4, 0.042),
        make_segment(SegmentPartial {
            kind: Some(SegmentKind::Chamber),
            length: Some(0.4),
            d_in: Some(0.042),
            d_out: Some(width),
            section: Some(section),
            height: Some(height),
            ..Default::default()
        }),
        pipe(0.4, 0.042),
    ]
}

fn duct(segments: &[PipeSegment], wall: f64) -> EulerPipe {
    let opts = EulerPipeOptions { cell_size: Some(0.035), wall_thickness: Some(wall), ..Default::default() };
    EulerPipe::new(segments, FS, 900.0, &opts)
}

fn shell_of(d: &EulerPipe) -> ChamberShell {
    ChamberShell::new(0, &d.chambers[0], d.dx, d.n, d.wall_thickness(), FS)
}

/// A round can's lowest mode is its end plates' fundamental: a clamped disc's, stiffened a little by
/// the gas behind it.
#[test]
fn a_round_can_rings_first_at_its_end_plates() {
    let d = duct(&can(ChamberSection::Round, 0.16, 0.16), 0.0012);
    let lowest = shell_of(&d).frequencies()[0];
    // `10.2158 / (2 pi a^2) * h * sqrt(E / (12 rho (1 - nu^2)))`
    let bending = (200e9 / (12.0 * 7800.0 * (1.0 - 0.09f64))).sqrt();
    let disc = 10.2158 / (2.0 * std::f64::consts::PI * 0.08 * 0.08) * 0.0012 * bending;
    assert!(lowest >= disc && lowest < 1.2 * disc, "lowest {lowest} Hz, disc {disc} Hz");
}

/// Thicker walls ring higher.
#[test]
fn thicker_walls_ring_higher() {
    let segs = can(ChamberSection::Round, 0.16, 0.16);
    let thin = shell_of(&duct(&segs, 0.0008)).frequencies()[0];
    let thick = shell_of(&duct(&segs, 0.002)).frequencies()[0];
    assert!(thick > 1.5 * thin, "thin {thin} Hz, thick {thick} Hz");
}

/// An oval can's broad faces ring lower than a round can's stiff shell and small end plates.
#[test]
fn an_oval_can_rings_lower_than_a_round_one() {
    let round = shell_of(&duct(&can(ChamberSection::Round, 0.16, 0.16), 0.0012)).frequencies()[0];
    let oval = shell_of(&duct(&can(ChamberSection::Oval, 0.26, 0.12), 0.0012)).frequencies()[0];
    assert!(oval < round, "oval {oval} Hz, round {round} Hz");
}

/// A steady pressure inside only holds the walls out; once they settle they radiate nothing.
#[test]
fn a_steady_pressure_radiates_nothing_once_settled() {
    let mut d = duct(&can(ChamberSection::Oval, 0.26, 0.12), 0.0012);
    let mut shell = shell_of(&d);
    let p = 1.2 * gas::P_AMB;
    for i in 0..d.n {
        d.set_primitive(i, p / (gas::R * 900.0), 0.0, p);
    }
    let first = shell.process(&d).abs();
    let mut last = 0.0f64;
    for _ in 0..(FS as usize / 2) {
        last = shell.process(&d).abs();
    }
    assert!(first > 0.0, "the step should ring the walls");
    assert!(last < 1e-6 * first.max(1e-12), "still radiating {last} Pa after {first} Pa");
}
