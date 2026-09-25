# Acoustics

This page covers how the exhaust gas is simulated, how heat and friction affect it, and how the
valve's flow noise is shaped.

## The gas solver

The exhaust is simulated with the **[quasi-one-dimensional](glossary.md#quasi-one-dimensional-flow) [Euler equations](glossary.md#euler-equations)**. These are the
basic equations of gas flow (mass, momentum and energy), written for a pipe whose
cross-section can change along its length. They are solved with a **[MUSCL-Hancock](glossary.md#muscl-hancock)**
[finite-volume](glossary.md#finite-volume-method) scheme, which splits the pipe into cells and is [second-order accurate](glossary.md#order-of-accuracy) in space
and time. It uses a **[TVD](glossary.md#tvd) [slope limiter](glossary.md#slope-limiter)** (explained below) on the [primitive variables](glossary.md#primitive-variables), and an
**[HLLC](glossary.md#hllc)** solver at every face between cells. HLLC is a fast, approximate way to work out what
flows between two neighbouring cells.

```
d/dt (A W) + d/dx (A F) = A S + [0, p dA/dx, 0]
W = [rho, rho u, rho E]      F = [rho u, rho u^2 + p, (rho E + p) u]
```

### Wave steepening

Exhaust pulses are big: roughly a bar in the header, at [Mach](glossary.md#mach-number) 0.3–0.8. At that size the
high-pressure part of a wave travels faster than the low-pressure part, so the front steepens
toward a [shock](glossary.md#shock-wave). In tests, a 0.8 bar wave that travels 0.8 m becomes about 1.9x steeper than a
sine wave of the same amplitude, while a small wave stays a sine. That steepening adds
[harmonics](glossary.md#harmonic), which is the brassy crackle of an open pipe.

### The slope limiter

The **slope limiter** stops the solver from ringing. Without one, a second-order scheme
oscillates wildly at every steep front, and you can hear it. It is checked on [Sod's shock
tube](glossary.md#sod-shock-tube) ([Sod 1978](references.md#sod1978)), a standard test problem with a
known exact answer. The [minmod](glossary.md#slope-limiter), monotonized-central and
van Leer limiters all give zero overshoot, and the error shrinks as the grid gets finer. The
default is **monotonized central**. Over 1.3 m of pipe it loses 4.5 dB at 5 kHz, while minmod
loses 12.4 dB, with the same zero overshoot.

### Temperature and flow

Two things come straight out of the equations:

- **Gas temperature is solved.** The hot charge moves and cools as it travels, and carries its
  own local [speed of sound](glossary.md#speed-of-sound).
- **Mean flow is real.** The waves ride on gas that is actually moving.

### Stability

Two details of the [predictor](glossary.md#muscl-hancock) step keep the solver stable:

- The `p dA` source term is in the Hancock **predictor** as well as the corrector. Without it, a
  cavity between two area changes traps the error and feeds itself: 650x growth in acoustic
  energy in a muffler chamber. A single expansion or contraction does not show this; only a
  cavity does.
- The predictor's fluxes are **area-weighted** too. Otherwise gas at rest gains momentum
  wherever the area changes, and the resulting transient is tens of dB too loud.

!!! note
    Total energy is not a useful stability check here. Internal energy (around 1e5 J/m^3)
    swamps it, so the acoustic field can grow a thousandfold while total energy stays
    conserved to 1e-9. The tests track acoustic energy instead.

### Area changes

Area can change by at most 1.6x per cell. This helps stability, but the main reason is that
quasi-1D theory assumes the area changes slowly compared with the radius. Where it doesn't,
the real flow is two-dimensional and a 1D model can't describe it. A chamber's step expansion
still behaves like a step: spread over four cells, it stays acoustically sharp well past a
few kHz.

### The open end

At the open end, reflection weakens above the frequency `c/a` (speed of sound over pipe
radius). High frequencies radiate out instead of bouncing back and forming [standing waves](glossary.md#standing-wave). So
a wide megaphone lets treble out, and a small tailpipe keeps it in.

A [one-pole filter](glossary.md#one-pole-high-pass-and-low-pass-filters) at `c/a` closely matches the [Levine-Schwinger](glossary.md#levineschwinger) result for an unflanged pipe ([Levine and Schwinger 1948](references.md#levine1948))
(the standard reference for this):

| ka  | Measured \|R\| | Levine-Schwinger |
|-----|----------------|------------------|
| 0.8 | 0.77           | 0.80             |
| 1.5 | 0.55           | 0.55             |

A second pole sits at the **[plane-wave cut-on](glossary.md#cut-on-frequency)**. Above that frequency, sound in the pipe is no
longer a simple flat wave, so a plane-wave reflection value means nothing and the energy
should leave. The effect is large: measured energy decay time drops from 188 ms at 84 Hz to
8 ms at 1.9 kHz, which matches the predicted loss per round trip.

Both filter coefficients are worked out from the solver's step length, because the boundary
runs once per solver step. That is one step per audio sample in the app.

### Radiation

Only the small wave coming *back in* from outside is treated as linear acoustics. That is fine
because radiation is weak. The outgoing wave stays fully nonlinear.

The [far-field](glossary.md#far-field) sound is a first-order [highpass](glossary.md#one-pole-high-pass-and-low-pass-filters) at the same `c/a` corner. Below the corner it
reduces to the [monopole](glossary.md#monopole) (point source) result `p = rho/(4 pi r) dQ/dt`. Its 6 dB/octave rise
is why a real exhaust cracks rather than thumps. Above the corner it levels off, like a
piston in a baffle.

Output is band-limited at whichever limit comes first:

- the plane-wave cut-on at `1.84 c/a`, which applies for a wide mouth, or
- the solver's resolution limit `c/(5 dx)`, which applies for a narrow one, where cut-on can
  be above 10 kHz.

Above either limit, the output would be numerical noise, not real sound.

### The exhaust port

The **exhaust port** is modelled as the first length of duct, not as a single lumped volume.
At 55 mm long, its own [quarter-wave resonance](glossary.md#quarter-wave-resonance) is near 3 kHz, right in the range that matters.
Treating it as a simple volume would wrongly cut high frequencies from the source there.
Modelling it as duct also means tuned length is measured from the valve seat, as it is on a
real engine.

## Heat and friction

Heat transfer and friction are modelled in several places. Each one is checked to be active
and correctly sized.

### In the cylinder

The cylinder uses the **[Woschni](glossary.md#woschni-model)** model ([Woschni 1967](references.md#woschni1967)), a standard formula for heat loss from gas to the
cylinder walls. Heat loss peaks at 33.5 kW at 69 bar. This puts the compression curve at a
[polytropic exponent](glossary.md#polytropic-exponent) of 1.28–1.36 rather than the [adiabatic](glossary.md#adiabatic-and-isentropic) (no heat loss) 1.35.

### In the pipe

The pipe uses a three-stage heat chain. The wall temperature is *solved*, not fixed:

```
gas --Dittus-Boelter Nu = 3 × 0.023 Re^0.8 Pr^0.4--> wall --convection + radiation--> ambient
```

Gas to wall uses the [Dittus–Boelter correlation](glossary.md#dittusboelter-correlation)
([Dittus and Boelter 1930](references.md#dittus1930)) for turbulent pipe flow, tripled because exhaust
flow pulses rather than flowing steadily. Wall to air combines outside convection with
[radiation](glossary.md#stefanboltzmann-law-and-emissivity).

Heat that leaves the gas goes *into* the wall instead of vanishing, so the two together
conserve energy. Radiation matters: oxidised steel at 800 K radiates about 18 kW/m², the same
as a [convection coefficient](glossary.md#heat-transfer-coefficient) of h = 36 W/(m²K), several times natural convection. Outside
convection uses a [Hilpert](glossary.md#hilpert-correlation) fit, giving 9 W/(m²K) in still air and 134 at 25 m/s.

A solved wall temperature gives three things a fixed one can't:

- **Warm-up.** From cold, 1.2 mm tubing takes tens of seconds to heat up (measured 293 → 646 K
  over 60 s). Gas temperature sets the speed of sound, so the tuning rises with it,
  82 → 100 Hz. The app starts with a warm wall so it sounds right straight away. Set
  `initialWallTemp` for a cold start.
- **A real temperature gradient.** The wall runs at 710 K at the flange and 510 K at the
  mouth, and the gas cools with it.
- **Airflow effects.** 30 m/s of airflow lowers the mean wall temperature by 174 K, and the
  tuning drops with it. Wall thickness sets the [thermal mass](glossary.md#thermal-mass), so a thin-wall pipe warms up
  much faster.

Wall temperature is kept when you edit the geometry. It takes about half a minute to build
up, so resetting it on every handle drag would make the tuning jump.

### Friction

Friction is split by what it acts on:

- A **linear** [boundary-layer](glossary.md#boundary-layer) term damps the *sound* only. It acts on the difference between
  the velocity and the mean flow.
- **[Darcy](glossary.md#darcy-friction)** pipe friction and separation loss at contractions are **quadratic** (they grow
  with velocity squared) and act on the total velocity.

```
du/dt = -k_lin (u - u_mean) - k_quad u
```

This is solved [implicitly](glossary.md#implicit-method) in `u`. Both parts are needed. Darcy friction alone barely damps
small waves, because it is quadratic, while real duct damping of sound is linear in velocity.

Acting on the total velocity, the linear term would also brake the mean flow: 120 m/s would
fall to 1 m/s in 50 ms. With the split, steady flow is almost untouched (286 → 275 m/s as the
coefficient goes 0 → 150), while acoustic damping is 1.59 dB per pass through a 1 m duct.

The mean flow is tracked with a 0.8 s [time constant](glossary.md#time-constant), slow enough to be close to steady. The
lowest sound frequency is the firing frequency, only 3.75 Hz at idle. A faster tracker would
mistake the firing pulses for mean flow and leave them undamped.

Friction heating works correctly with no extra code. Friction removes momentum but not energy,
so the lost kinetic energy turns into heat. In a test slowing gas from 120 to 1 m/s, the
temperature rose from 600 to 612 K, with total energy conserved to 0.0000%.


## Why it is not shrill

Turbulence noise at the valve throat is **band-limited**. It rolls off above the [Strouhal](glossary.md#strouhal-number)
frequency `0.2·U/d` (a rule of thumb for where flow noise peaks, from flow speed `U` and
diameter `d`), with a two-pole slope above it.

This matters a lot. [White noise](glossary.md#white-noise) would be tilted up by 6 dB/octave by the radiation into a
rising hiss, drowning out everything above 2 kHz by more than 10 dB. The noise is shaped for two
reasons:

- Turbulent eddies are small and scattered across the duct. Only the cross-section *average*
  drives the plane wave, so higher frequencies cancel out more.
- [Jet noise](glossary.md#aeroacoustics-and-jet-noise) peaks near a Strouhal number of 0.2 and falls off above it.

The level is also kept modest: 10% of the mean flow. That is below the ~15% turbulence
measured locally in a valve jet, for the same averaging reason.

[`test/timbre.test.ts`](https://github.com/tahoward/engine-simulator/blob/main/test/timbre.test.ts)
guards all of this.

## Heat transfer every sample

Gas-to-wall heat transfer is applied every audio sample. Only the costly [Nusselt](glossary.md#nusselt-reynolds-and-prandtl-numbers) calculation and
the slow wall update are batched, every 16 samples: the wall moves only about 0.007 K in that
time. Batching the gas transfer as well would put a regular energy kick into the pipe at
48000/16 = 3000 Hz, which is audible as a whistle.

## Sources

- Gas solver: [Toro 2009](references.md#toro2009); MUSCL: [van Leer 1979](references.md#vanleer1979);
  HLLC: [Toro, Spruce and Speares 1994](references.md#toro1994); limiters:
  [van Leer 1974](references.md#vanleer1974), [van Leer 1977](references.md#vanleer1977),
  [Roe 1986](references.md#roe1986); TVD: [Harten 1983](references.md#harten1983).
- Open end and radiation: [Levine and Schwinger 1948](references.md#levine1948),
  [Kinsler et al. 2000](references.md#kinsler2000), [Pierce 2019](references.md#pierce2019).
- Heat transfer: [Woschni 1967](references.md#woschni1967),
  [Dittus and Boelter 1930](references.md#dittus1930), [Hilpert 1933](references.md#hilpert1933).
- Flow noise: [Lighthill 1952](references.md#lighthill1952), [Tam 1998](references.md#tam1998).
