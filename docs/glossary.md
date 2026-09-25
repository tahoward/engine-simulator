# Glossary

Plain definitions of the technical terms used in these docs, grouped by subject. Each entry links to
a fuller definition, and to the published source where the method comes from one (see
[References](references.md)).

## Gas flow

### Euler equations

The equations for how a gas moves when viscosity is ignored: conservation of mass, momentum and
energy. The exhaust solver uses them in *quasi-one-dimensional* form (below).
[Wikipedia](https://en.wikipedia.org/wiki/Euler_equations_%28fluid_dynamics%29) ·
[Toro 2009](references.md#toro2009)

### Quasi-one-dimensional flow

Flow treated as varying only along a pipe's length, with the pipe's cross-section area allowed to
change. It is accurate when the area changes slowly compared with the pipe's radius.
[Toro 2009](references.md#toro2009)

### Conservation law

An equation saying a quantity (mass, momentum or energy) is neither created nor destroyed, only
moved around. A solver that respects it loses nothing to rounding in the bookkeeping.
[Wikipedia](https://en.wikipedia.org/wiki/Conservation_law)

### Speed of sound

How fast a small pressure wave travels through a gas. In an ideal gas it depends only on
temperature, so hot exhaust carries sound faster than cold air.
[Wikipedia](https://en.wikipedia.org/wiki/Speed_of_sound)

### Mach number

Flow speed divided by the local speed of sound. Mach 1 is the speed of sound.
[Wikipedia](https://en.wikipedia.org/wiki/Mach_number)

### Shock wave

A pressure front so steep that pressure, density and temperature jump almost instantly across it.
Large exhaust pulses steepen toward shocks as they travel.
[Wikipedia](https://en.wikipedia.org/wiki/Shock_wave)

### Choked flow

When the flow through a narrowing reaches the speed of sound, lowering the pressure downstream no
longer increases the flow. Exhaust valves choke when they first open.
[Wikipedia](https://en.wikipedia.org/wiki/Choked_flow) · [Heywood 2018](references.md#heywood2018)

### Ideal gas law

`p = ρ R T`: pressure equals density times the gas constant times temperature. The model treats
cylinder and exhaust gas as ideal gases.
[Wikipedia](https://en.wikipedia.org/wiki/Ideal_gas_law)

### Internal energy

The energy stored in a gas's molecular motion, set by its mass and temperature. The cylinder model
tracks internal energy rather than temperature.
[Wikipedia](https://en.wikipedia.org/wiki/Internal_energy)

### Enthalpy

Internal energy plus the work needed to push the gas into place (pressure times volume). It is what
a flow carries through a valve.
[Wikipedia](https://en.wikipedia.org/wiki/Enthalpy)

### Specific heat

The energy needed to warm one kilogram of a substance by one kelvin. `cv` is its value at constant
volume. The ratio of the two specific heats, γ, sets how gas behaves when compressed.
[Specific heat (Wikipedia)](https://en.wikipedia.org/wiki/Specific_heat_capacity) ·
[Heat capacity ratio (Wikipedia)](https://en.wikipedia.org/wiki/Heat_capacity_ratio)

### Adiabatic and isentropic

An *adiabatic* process exchanges no heat with its surroundings. An *isentropic* one is also
reversible, with no friction or mixing. Isentropic compression is the ideal limit a real cylinder
falls just short of.
[Adiabatic (Wikipedia)](https://en.wikipedia.org/wiki/Adiabatic_process) ·
[Isentropic (Wikipedia)](https://en.wikipedia.org/wiki/Isentropic_process)

### Polytropic exponent

The number `n` in `p Vⁿ = constant`, describing how pressure rises as gas is squeezed. Heat loss to
the walls makes `n` smaller than γ, the value with no heat loss.
[Wikipedia](https://en.wikipedia.org/wiki/Polytropic_process) ·
[Heywood 2018](references.md#heywood2018)

### Dissociation

Hot gas molecules splitting apart, which absorbs energy. It matters above about 2200 K and is not
modelled.
[Wikipedia](https://en.wikipedia.org/wiki/Dissociation_%28chemistry%29)

## Numerical methods

### Finite-volume method

A way to solve flow equations by splitting the pipe into cells and tracking how much mass, momentum
and energy flows across each cell boundary per step.
[Wikipedia](https://en.wikipedia.org/wiki/Finite_volume_method) ·
[Toro 2009](references.md#toro2009)

### Rayleigh–Ritz method

A way to find a shape's resonances by writing each one as a sum of simple functions and choosing
the sum that best balances its stored energies. Used here for a chamber's cross-wise modes, with
cosines over the section's bounding box.
[Wikipedia](https://en.wikipedia.org/wiki/Rayleigh%E2%80%93Ritz_method)

### Order of accuracy

How fast a method's error shrinks as the cells and steps get smaller. For a *second-order* method,
halving the cell size cuts the error roughly four times.
[Wikipedia](https://en.wikipedia.org/wiki/Order_of_accuracy)

### Primitive variables

Density, velocity and pressure: the quantities you would measure. The solver stores mass, momentum
and energy per unit volume instead (the *conserved* variables), and converts to primitive ones when
it needs slopes.
[Toro 2009](references.md#toro2009)

### MUSCL-Hancock

A second-order finite-volume scheme. In each cell it estimates how the gas varies across the cell
(the slopes), takes a half step forward in time (the *predictor*), then works out the flows between
cells from those predicted values (the *corrector*).
[Wikipedia](https://en.wikipedia.org/wiki/MUSCL_scheme) ·
[van Leer 1979](references.md#vanleer1979) · [Toro 2009](references.md#toro2009)

### Riemann solver

A method for working out what happens where two different gas states meet: how much mass, momentum
and energy flows between them. A finite-volume solver runs one at every cell boundary, every step.
[Wikipedia](https://en.wikipedia.org/wiki/Riemann_solver)

### HLLC

A fast, approximate Riemann solver that models the waves spreading from a boundary as three: a
left-moving wave, a contact between the two gases, and a right-moving wave.
[Wikipedia](https://en.wikipedia.org/wiki/Riemann_solver#HLLC_solver) ·
[Toro, Spruce and Speares 1994](references.md#toro1994)

### Slope limiter

A rule that reduces the slopes a second-order scheme uses near steep changes, so it does not create
false wiggles (overshoot) around shocks. *Minmod*, *monotonized central (MC)* and *van Leer* are
three common limiters; minmod is the most cautious.
[Wikipedia](https://en.wikipedia.org/wiki/Flux_limiter#Limiter_functions) ·
[van Leer 1974](references.md#vanleer1974) · [van Leer 1977](references.md#vanleer1977) ·
[Roe 1986](references.md#roe1986)

### TVD

*Total variation diminishing.* A property of a scheme that guarantees it never adds new peaks or
dips to the solution. Limited MUSCL schemes are TVD.
[Wikipedia](https://en.wikipedia.org/wiki/Total_variation_diminishing) ·
[Harten 1983](references.md#harten1983)

### Sod shock tube

A standard test: a tube with high pressure on one side and low on the other, suddenly opened. Its
exact solution is known, so a solver's answer can be checked against it.
[Wikipedia](https://en.wikipedia.org/wiki/Sod_shock_tube) · [Sod 1978](references.md#sod1978)

### CFL condition

*Courant–Friedrichs–Lewy condition.* The stability rule for this kind of solver: in one step, no wave
may travel further than one cell. It sets the longest step a given cell size allows. The *CFL number*
is the fraction of that limit actually used (0.85 here).
[Wikipedia](https://en.wikipedia.org/wiki/Courant%E2%80%93Friedrichs%E2%80%93Lewy_condition) ·
[Courant, Friedrichs and Lewy 1928](references.md#cfl1928)

### Substep

A solver step shorter than one audio sample. The app sizes cells so that one step per sample is
enough; the solver only takes extra substeps if a wave ever outruns the design speed.

### Well-balanced

A scheme is well-balanced if gas at rest stays at rest, even where the pipe's area changes. Without
this, a scheme invents flow at every change of diameter.

### Numerical diffusion

Smearing that a numerical scheme adds on its own, which dulls sharp fronts and high frequencies. It
is why the solver has a highest frequency it can reproduce.
[Wikipedia](https://en.wikipedia.org/wiki/Numerical_diffusion)

### Implicit method

A way of stepping an equation forward that uses the *new* value on both sides of the equation. It
stays stable where the simple (explicit) method would overshoot, which is why friction is applied
this way.
[Wikipedia](https://en.wikipedia.org/wiki/Explicit_and_implicit_methods)

### Newton's method

A standard way to refine an estimate of where a function is zero: follow its slope to a better guess,
and repeat. Junctions use two such steps to balance their flows.
[Wikipedia](https://en.wikipedia.org/wiki/Newton%27s_method)

## Sound and signals

### Plane wave

A sound wave whose pressure is the same across the whole cross-section of a pipe, travelling as a
flat front. At low frequencies, all sound in a pipe travels this way.
[Wikipedia](https://en.wikipedia.org/wiki/Plane_wave) · [Kinsler et al. 2000](references.md#kinsler2000)

### Cut-on frequency

The frequency above which a pipe can carry sound patterns other than plane waves. For a round pipe of
radius `a`, the first one starts at `1.84 c / (2π a)` Hz, which the docs write as `1.84 c/a` in
radians per second.
[Wikipedia](https://en.wikipedia.org/wiki/Cutoff_frequency#Waveguides) ·
[Kinsler et al. 2000](references.md#kinsler2000)

### ka

Wavenumber `k = 2π / wavelength` times the pipe radius `a`. It compares pipe size with wavelength: at
small `ka`, an open end reflects almost everything; at large `ka`, sound escapes.
[Wavenumber (Wikipedia)](https://en.wikipedia.org/wiki/Wavenumber)

### Reflection coefficient

The fraction of a wave's amplitude that bounces back at a boundary, written `|R|`. 1 means
everything is reflected; 0 means everything passes through.
[Wikipedia](https://en.wikipedia.org/wiki/Reflection_coefficient)

### Levine–Schwinger

The exact solution for how much sound reflects from the open end of an unflanged round pipe at each
frequency. It is the standard reference the open-end model is checked against.
[Levine and Schwinger 1948](references.md#levine1948)

### Standing wave

A wave pattern that stays in place because waves travelling both ways along a pipe overlap. Pipe
resonances are standing waves.
[Wikipedia](https://en.wikipedia.org/wiki/Standing_wave)

### Quarter-wave resonance

The lowest resonance of a pipe closed at one end and open at the other. Its wavelength is four times
the pipe's length, so its frequency is `c / 4L`.
[Wikipedia](https://en.wikipedia.org/wiki/Acoustic_resonance#Closed_at_one_end)

### Cross-wise mode

A resonance across a duct or can rather than along it, where the pressure differs from one side
of the section to the other. It can only exist above a cut-on frequency set by the section's
width: about `c/2W` for a flat can, and `1.84 c/(π D)` for a round one.
[Wikipedia](https://en.wikipedia.org/wiki/Waveguide_(acoustics)) · [Kinsler et al. 2000](references.md#kinsler2000)

### Monopole

A point source of sound that pushes gas out and in equally in all directions. An exhaust mouth,
small compared with the wavelength, behaves like one: its pressure at distance `r` is
`p = ρ/(4π r) dQ/dt`, where `Q` is the volume flow.
[Wikipedia](https://en.wikipedia.org/wiki/Point_source) · [Kinsler et al. 2000](references.md#kinsler2000)

### Far field

The region far enough from a source that sound simply spreads out and falls off with distance.
[Wikipedia](https://en.wikipedia.org/wiki/Near_and_far_field)

### One-pole, high-pass and low-pass filters

A *low-pass* filter lets low frequencies through and cuts high ones; a *high-pass* filter does the
opposite. A *one-pole* filter is the simplest kind, rolling off at 6 dB per octave past its corner
frequency.
[Low-pass (Wikipedia)](https://en.wikipedia.org/wiki/Low-pass_filter) ·
[High-pass (Wikipedia)](https://en.wikipedia.org/wiki/High-pass_filter)

### Comb filter

What you get by adding a signal to delayed copies of itself: some frequencies add up and others
cancel, giving a comb-like spectrum. Evenly firing cylinders act as one.
[Wikipedia](https://en.wikipedia.org/wiki/Comb_filter)

### Harmonic

A frequency that is a whole-number multiple of a fundamental. Steepening waves add harmonics.
[Wikipedia](https://en.wikipedia.org/wiki/Harmonic)

### Order

A frequency measured as a multiple of the engine's rotation. In these docs, one order is `rpm/120`
Hz, once per four-stroke cycle. The *firing frequency* is `cylinders × rpm/120`, so it is the
`cylinders`-th order. (Not to be confused with [firing order](#firing-order), the sequence the
cylinders fire in.)

### Decibel (dB)

A logarithmic scale for level. +6 dB is roughly double the amplitude; +20 dB is ten times.
[Wikipedia](https://en.wikipedia.org/wiki/Decibel)

### Octave

A doubling of frequency.
[Wikipedia](https://en.wikipedia.org/wiki/Octave)

### RMS

*Root mean square*: the square root of the average of the squared signal. A standard measure of a
signal's overall size.
[Wikipedia](https://en.wikipedia.org/wiki/Root_mean_square)

### Spectral centroid

The "centre of mass" of a sound's spectrum: an average frequency, weighted by level. Brighter sounds
have a higher centroid.
[Wikipedia](https://en.wikipedia.org/wiki/Spectral_centroid)

### White noise

Random noise with equal energy at every frequency.
[Wikipedia](https://en.wikipedia.org/wiki/White_noise)

### Strouhal number

A dimensionless number, `f d / U`, relating a flow's typical frequency `f` to its speed `U` and size
`d`. Jet noise peaks near a Strouhal number of about 0.2.
[Wikipedia](https://en.wikipedia.org/wiki/Strouhal_number) · [Tam 1998](references.md#tam1998)

### Aeroacoustics and jet noise

The study of sound made by moving air itself, such as the turbulent jet through a valve.
[Wikipedia](https://en.wikipedia.org/wiki/Aeroacoustics) · [Lighthill 1952](references.md#lighthill1952)

### Coefficient of variation

Standard deviation divided by the average, as a percentage. Used for how much each engine cycle's
output varies.
[Wikipedia](https://en.wikipedia.org/wiki/Coefficient_of_variation)

### Time constant

How long a smoothly settling quantity takes to cover about 63% of the way to its final value.
[Wikipedia](https://en.wikipedia.org/wiki/Time_constant)

## Heat and friction

### Hydraulic diameter

`4A/P`, four times a duct's area over its wetted perimeter. For a round pipe it is the diameter.
It lets pipe-flow friction and heat-transfer correlations be used for ducts that aren't round.
[Wikipedia](https://en.wikipedia.org/wiki/Hydraulic_diameter)

### Heat transfer coefficient

`h`, the heat flow per square metre per kelvin of temperature difference between a surface and the
gas or air next to it, in W/(m²K).
[Wikipedia](https://en.wikipedia.org/wiki/Heat_transfer_coefficient)

### Nusselt, Reynolds and Prandtl numbers

Dimensionless numbers used in heat transfer correlations. *Reynolds* (`Re`) compares a flow's
inertia with its viscosity; high values mean turbulent flow. *Prandtl* (`Pr`) is a property of the
gas. *Nusselt* (`Nu`) is the heat transfer coefficient in dimensionless form.
[Nusselt](https://en.wikipedia.org/wiki/Nusselt_number) ·
[Reynolds](https://en.wikipedia.org/wiki/Reynolds_number) ·
[Prandtl](https://en.wikipedia.org/wiki/Prandtl_number) (Wikipedia)

### Dittus–Boelter correlation

`Nu = 0.023 Re^0.8 Pr^n`: the standard estimate of heat transfer from turbulent flow in a pipe,
with `n = 0.4` for a fluid being heated and `0.3` for one being cooled. The pipes use the cooling
form, since exhaust gas is hotter than the wall, multiplied by 3 for the pulsing flow in an exhaust.
[Wikipedia](https://en.wikipedia.org/wiki/Nusselt_number#Dittus%E2%80%93Boelter_equation) ·
[Dittus and Boelter 1930](references.md#dittus1930)

### Zukauskas correlation

`Nu = C Re^m Pr^0.37`: heat transfer from a cylinder, such as a pipe, in air flowing across it.
Used for the outside of the exhaust.
[Zukauskas 1972](references.md#zukauskas1972)

### Stefan–Boltzmann law and emissivity

A hot surface radiates heat in proportion to the fourth power of its absolute temperature. The
*emissivity* (0 to 1) says how close it comes to a perfect radiator; oxidised steel is about 0.8.
[Stefan–Boltzmann law](https://en.wikipedia.org/wiki/Stefan%E2%80%93Boltzmann_law) ·
[Emissivity](https://en.wikipedia.org/wiki/Emissivity) (Wikipedia)

### Darcy friction

Pressure lost to friction along a pipe, growing with the square of the flow speed. The *Darcy
friction factor* sets its size; exhaust tubing is about 0.02–0.04.
[Wikipedia](https://en.wikipedia.org/wiki/Darcy%E2%80%93Weisbach_equation)

### Boundary layer

The thin layer of gas next to a wall, slowed by friction with it. It is where sound in a pipe loses
most of its energy.
[Wikipedia](https://en.wikipedia.org/wiki/Boundary_layer)

### Thermal mass

How much heat an object has to take in to warm up. A thick pipe wall has more, so it warms more
slowly.
[Wikipedia](https://en.wikipedia.org/wiki/Thermal_mass)

## Engines

### Four-stroke cycle

Intake, compression, power and exhaust, over two turns of the crank (720°). Each cylinder fires once
per cycle.
[Wikipedia](https://en.wikipedia.org/wiki/Four-stroke_engine)

### TDC

*Top dead centre*: the piston's highest point. Each cylinder passes it twice per cycle, once to fire
and once during valve overlap.
[Wikipedia](https://en.wikipedia.org/wiki/Dead_centre_(engineering))

### Firing order

The sequence in which the cylinders fire, such as 1-3-4-2 on an inline four.
[Wikipedia](https://en.wikipedia.org/wiki/Firing_order)

### Crossplane and flatplane cranks

Two V8 crankshafts. A *crossplane* crank has its pins at 90° intervals; a *flatplane* crank has them
all in one plane, 180° apart. Both fire every 90°, but they share the firings between the banks
differently.
[Crossplane](https://en.wikipedia.org/wiki/Crossplane) ·
[Flat-plane crank](https://en.wikipedia.org/wiki/Flat-plane_crank) (Wikipedia)

### Boxer

A flat engine with its two banks 180° apart and each opposed pair of pistons moving in and out
together.
[Wikipedia](https://en.wikipedia.org/wiki/Flat_engine)

### Valve overlap

The part of the cycle, around TDC between exhaust and intake strokes, when both valves are briefly
open. Exhaust tuning works mostly here.
[Wikipedia](https://en.wikipedia.org/wiki/Valve_timing)

### Filling-and-emptying model

A cylinder model that treats the gas as one well-mixed volume whose size changes with the piston,
with gas entering and leaving through the valves.
[Heywood 2018](references.md#heywood2018)

### Wiebe function

A standard S-shaped curve for how much of the fuel has burned at each crank angle during combustion.
[Heywood 2018](references.md#heywood2018)

### Woschni model

A standard formula for the heat transfer coefficient between cylinder gas and the cylinder walls,
based on pressure, temperature, piston speed and, during combustion, how far the pressure has
risen above the unfired curve.
[Woschni 1967](references.md#woschni1967)

### Dynamometer

A test rig that holds an engine at a set speed and measures its torque.
[Wikipedia](https://en.wikipedia.org/wiki/Dynamometer)

### Piston slap

The knock of a piston rocking across its bore as the force on it changes direction near TDC. It is
loudest under high cylinder pressure.

## Software

### AudioWorklet

The browser feature that runs custom audio code on its own real-time audio thread, separate from the
page.
[MDN](https://developer.mozilla.org/en-US/docs/Web/API/AudioWorklet)

### WebAssembly (Wasm)

A compact, fast compiled code format that browsers run alongside JavaScript.
[MDN](https://developer.mozilla.org/en-US/docs/WebAssembly)

### SIMD

*Single instruction, multiple data*: processor instructions that work on several numbers at once.
WebAssembly's SIMD works on two 64-bit numbers at a time.
[Wikipedia](https://en.wikipedia.org/wiki/Single_instruction,_multiple_data) ·
[WebAssembly SIMD proposal](https://github.com/WebAssembly/simd)

### AssemblyScript

A TypeScript-like language that compiles to WebAssembly. The solver's kernel is written in it.
[assemblyscript.org](https://www.assemblyscript.org/)

### postMessage and SharedArrayBuffer

Two ways for browser threads to share data. `postMessage` copies a message from one thread to
another. `SharedArrayBuffer` shares memory directly, but only on pages served with special
cross-origin isolation headers.
[postMessage (MDN)](https://developer.mozilla.org/en-US/docs/Web/API/MessagePort/postMessage) ·
[SharedArrayBuffer (MDN)](https://developer.mozilla.org/en-US/docs/Web/JavaScript/Reference/Global_Objects/SharedArrayBuffer)
