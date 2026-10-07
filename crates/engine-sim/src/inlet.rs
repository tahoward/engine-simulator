//! The air's way in, on an engine without a turbo: from the snorkel's open mouth, through the airbox,
//! to the throttle, solved with the same gas dynamics as the exhaust. Dual plenums' two throttle bodies
//! have a tract each, mirrored either side of the engine, each with its own airbox and snorkel.
//!
//! The throttle draws from the tract's closed end: air is drawn through it as the plenum's pressure
//! falls below the pressure there, chokes at a small opening, and is pushed back out when the plenum is
//! above it. The runners empty and refill the plenum at the firing frequency, so the throttle's flow
//! pulses, and those pulses travel up the tract, ring in the airbox and the snorkel, and leave its mouth
//! as the intake's note. What the tract does to the pressure at the throttle comes back to it, so a
//! snorkel tuned to the engine's speed helps it breathe there.
//!
//! The throttle's flow is a sink in the tract's end cell, and the solver brings a flow that large to
//! rest against the closed end only by drawing that cell down several kPa below the pressure beside it,
//! where a real duct carries its air through the throttle with almost no loss. So the throttle sees
//! the atmosphere's pressure plus the swing at the tract's end about its slowly tracked mean: the
//! tract's waves, without its numerical loss. There is no filter element, and no loss for one.
//!
//! The air rushing in makes its own sound. Past the throttle plate it is a jet, and a jet's
//! turbulence is broadband noise peaking at its Strouhal frequency, `0.2 U / d` for a jet of speed `U`
//! through a gap `d` wide. Nearly shut, the gap is a sliver and the jet near sonic, so it hisses high;
//! wide open, the air is slow through the whole bore and it rushes low. It is injected at the throttle
//! as the exhaust valves' throat turbulence is, so it reaches the air through the airbox and snorkel,
//! coloured by them.

use crate::dsp::Noise;
use crate::euler_pipe::{EulerPipe, EulerPipeOptions, InletKind, OutletKind, ValveState};
use crate::intake::runner_damping;
use crate::math::{self, PI, clamp};
use crate::plenum::{plenum_count_of, throttle_dia_of};
use crate::spec::{EngineSpec, PipeSegment, SegmentKind, SegmentPartial, displacement, gas, make_segment};

/// Airbox volume as a multiple of the engine's swept volume, when not given.
const AIRBOX_VOLUME_RATIO: f64 = 4.0;

/// The duct from the airbox to the throttle, m.
const THROTTLE_DUCT_LENGTH: f64 = 0.35;

/// The snorkel's bore as a multiple of the throttle's, when not given.
const SNORKEL_BORE_RATIO: f64 = 1.1;

/// RMS turbulent fluctuation of the jet past the throttle, as a fraction of the flow through it: as
/// the exhaust valves' throat turbulence, of which it is one more case.
const THROTTLE_TURBULENCE: f64 = 0.1;

/// Time constant of the mean the pressure at the throttle swings about, s: slower than the slowest
/// firing frequency.
const MEAN_TAU: f64 = 0.5;

/// Airbox volume, m^3: the spec's, or sized for the engine; dual plenums' two airboxes' together.
pub fn airbox_volume_of(spec: &EngineSpec) -> f64 {
    if spec.airbox_volume > 0.0 {
        return spec.airbox_volume;
    }
    AIRBOX_VOLUME_RATIO * displacement(spec) * math::max(spec.cylinders as f64, 1.0)
}

/// The snorkel's bore, m, each tract's: the spec's, or a little wider than its throttle.
pub fn snorkel_dia_of(spec: &EngineSpec) -> f64 {
    if spec.snorkel_dia > 0.0 {
        return spec.snorkel_dia;
    }
    SNORKEL_BORE_RATIO * throttle_dia_of(spec)
}

/// How many inlet tracts there are: one for each throttle body.
pub fn inlet_count_of(spec: &EngineSpec) -> usize {
    plenum_count_of(spec)
}

/// Each tract from its throttle out to its snorkel's mouth: a duct at the throttle's bore, the airbox,
/// a round can about as long as it is wide, holding its share of `airbox_volume_of`, and the snorkel.
pub fn inlet_segments(spec: &EngineSpec) -> Vec<PipeSegment> {
    let throttle = throttle_dia_of(spec);
    let volume = airbox_volume_of(spec) / inlet_count_of(spec) as f64;
    let length = clamp(math::cbrt(volume) * 1.5, 0.15, 0.6);
    let body = math::max(math::sqrt((4.0 * volume) / (PI * length)), throttle * 1.5);
    let pipe = |length: f64, dia: f64| {
        make_segment(SegmentPartial {
            kind: Some(SegmentKind::Pipe),
            length: Some(length),
            d_in: Some(dia),
            ..Default::default()
        })
    };
    vec![
        pipe(THROTTLE_DUCT_LENGTH, throttle),
        make_segment(SegmentPartial {
            kind: Some(SegmentKind::Chamber),
            length: Some(length),
            d_in: Some(throttle),
            d_out: Some(body),
            ..Default::default()
        }),
        pipe(math::max(spec.snorkel_length, 0.02), snorkel_dia_of(spec)),
    ]
}

pub struct InletTract {
    pub pipe: EulerPipe,
    /// Volume flow out of the snorkel's mouth, m^3/s: negative as the engine draws.
    pub mouth_flow: f64,
    mean_p: f64,
    mean_c: f64,
    noise: Noise,
    turb1: f64,
    turb2: f64,
}

impl InletTract {
    pub fn new(spec: &EngineSpec, sample_rate: f64, opts: &EulerPipeOptions) -> InletTract {
        InletTract::nth(spec, sample_rate, opts, 0)
    }

    /// Tract `k` of `inlet_count_of`: each the same, but for its jet noise's seed.
    pub fn nth(spec: &EngineSpec, sample_rate: f64, opts: &EulerPipeOptions, k: usize) -> InletTract {
        let segments = inlet_segments(spec);
        let length: f64 = segments.iter().map(|s| s.length).sum();
        let c = crate::spec::speed_of_sound(gas::T_AMB, gas::GAMMA_AIR);
        let damping = runner_damping(snorkel_dia_of(spec) / 2.0, c / (4.0 * length));
        let tract_opts = EulerPipeOptions {
            inlet_kind: Some(InletKind::Valve),
            outlet_kind: Some(OutletKind::Mouth),
            heat_transfer: Some(false),
            initial_port_temp: Some(gas::T_AMB),
            linear_damping: Some(damping),
            port: None,
            inherit_wall: None,
            // The exhaust's material is the exhaust's alone.
            material: None,
            ..opts.clone()
        };
        InletTract {
            pipe: EulerPipe::new(&segments, sample_rate, gas::T_AMB, &tract_opts),
            mouth_flow: 0.0,
            mean_p: gas::P_AMB,
            mean_c: 1.0 - math::exp(-1.0 / (MEAN_TAU * sample_rate)),
            noise: Noise::new((0x51ab3c + 7919 * k) as f64),
            turb1: 0.0,
            turb2: 0.0,
        }
    }

    /// The air the throttle draws on first, m^3: the cell of the tract at its throttle end.
    pub fn throttle_end_volume(&self) -> f64 {
        self.pipe.port_cell_volume()
    }

    /// The pressure the throttle draws from, Pa: the atmosphere's, plus the tract's swing at its end.
    pub fn upstream_pressure(&self) -> f64 {
        gas::P_AMB + (self.pipe.port_pressure() - self.mean_p)
    }

    /// Advance by one sample of `dt`, s, with `throttle_flow`, kg/s, drawn through the throttle into the
    /// plenum, its effective flow area `area`, m^2, and its bore `bore`, m. `turbulence` scales the jet's
    /// noise.
    pub fn advance(&mut self, dt: f64, throttle_flow: f64, area: f64, bore: f64, turbulence: f64) {
        // The jet past the plate: its speed through the gap, no faster than sound, and the gap's width,
        // the area spread round the plate's edge.
        let mut extra = 0.0;
        if area > 0.0 && turbulence > 0.0 {
            let (p_port, t_port, _) = self.pipe.read_port();
            let rho = p_port / (gas::R * t_port);
            let flow = throttle_flow.abs();
            let speed = math::min(flow / math::max(rho * area, 1e-9), math::sqrt(gas::GAMMA_AIR * gas::R * t_port));
            let gap = math::max((2.0 * area) / (PI * math::max(bore, 1e-3)), 1e-4);
            let strouhal_hz = (0.2 * speed) / gap;
            let k = clamp(1.0 - math::exp(-2.0 * PI * strouhal_hz * dt), 0.02, 0.85);
            let white = self.noise.next() * flow * THROTTLE_TURBULENCE * turbulence;
            self.turb1 += k * (white - self.turb1);
            self.turb2 += k * (self.turb1 - self.turb2);
            extra = self.turb2;
        }
        // The valve end's source is flow out of its "cylinder", the plenum, into the tract.
        let valve = ValveState { throat_area: area, extra_mass_flow: extra, ..ValveState::default() };
        let source = -throttle_flow + extra;

        let r = &mut self.pipe;
        let substeps = r.substeps_for(dt);
        let h = dt / substeps as f64;
        let mut mouth = 0.0;
        for _ in 0..substeps {
            r.begin_step(h);
            mouth += r.apply_own_boundaries(h);
            r.set_end_step(h, source);
            r.end_step_set(&valve);
            r.after_step(h);
        }
        if r.recover_if_broken() {
            mouth = 0.0;
        }
        self.mouth_flow = mouth / substeps as f64;
        self.mean_p += self.mean_c * (r.port_pressure() - self.mean_p);
    }
}
