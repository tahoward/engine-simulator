# References

The published sources behind the models and methods in these docs. Each entry says what it is used
for. Plain-language definitions of the terms are in the [Glossary](glossary.md).

## Gas dynamics and numerical methods

- <a id="toro2009"></a>**Toro, E. F. (2009).** *Riemann Solvers and Numerical Methods for Fluid
  Dynamics: A Practical Introduction*, 3rd ed. Springer.
  [doi:10.1007/b79761](https://doi.org/10.1007/b79761)
  — The standard textbook for the Euler equations, the MUSCL-Hancock scheme, slope limiters and the
  HLLC Riemann solver, all of which the exhaust solver uses.
- <a id="toro1994"></a>**Toro, E. F., Spruce, M. and Speares, W. (1994).** "Restoration of the
  contact surface in the HLL-Riemann solver." *Shock Waves* 4, 25–34.
  [doi:10.1007/BF01414629](https://doi.org/10.1007/BF01414629)
  — The paper that introduced the HLLC solver.
- <a id="vanleer1974"></a>**van Leer, B. (1974).** "Towards the ultimate conservative difference
  scheme. II. Monotonicity and conservation combined in a second-order scheme." *Journal of
  Computational Physics* 14, 361–370.
  [doi:10.1016/0021-9991(74)90019-9](https://doi.org/10.1016/0021-9991(74)90019-9)
  — The van Leer limiter.
- <a id="vanleer1977"></a>**van Leer, B. (1977).** "Towards the ultimate conservative difference
  scheme. IV. A new approach to numerical convection." *Journal of Computational Physics* 23,
  276–299. [doi:10.1016/0021-9991(77)90095-X](https://doi.org/10.1016/0021-9991(77)90095-X)
  — The monotonized-central (MC) limiter, the solver's default.
- <a id="vanleer1979"></a>**van Leer, B. (1979).** "Towards the ultimate conservative difference
  scheme. V. A second-order sequel to Godunov's method." *Journal of Computational Physics* 32,
  101–136. [doi:10.1016/0021-9991(79)90145-1](https://doi.org/10.1016/0021-9991(79)90145-1)
  — The MUSCL approach: second-order finite-volume schemes built on reconstructed slopes.
- <a id="harten1983"></a>**Harten, A. (1983).** "High resolution schemes for hyperbolic
  conservation laws." *Journal of Computational Physics* 49, 357–393.
  [doi:10.1016/0021-9991(83)90136-5](https://doi.org/10.1016/0021-9991(83)90136-5)
  — Total variation diminishing (TVD) schemes.
- <a id="roe1986"></a>**Roe, P. L. (1986).** "Characteristic-based schemes for the Euler
  equations." *Annual Review of Fluid Mechanics* 18, 337–365.
  [doi:10.1146/annurev.fl.18.010186.002005](https://doi.org/10.1146/annurev.fl.18.010186.002005)
  — Survey of limiters, including minmod.
- <a id="sod1978"></a>**Sod, G. A. (1978).** "A survey of several finite difference methods for
  systems of nonlinear hyperbolic conservation laws." *Journal of Computational Physics* 27, 1–31.
  [doi:10.1016/0021-9991(78)90023-2](https://doi.org/10.1016/0021-9991(78)90023-2)
  — The shock tube test problem the tests use.
- <a id="cfl1928"></a>**Courant, R., Friedrichs, K. and Lewy, H. (1928).** "Über die partiellen
  Differenzengleichungen der mathematischen Physik." *Mathematische Annalen* 100, 32–74.
  [doi:10.1007/BF01448839](https://doi.org/10.1007/BF01448839)
  — The CFL condition that limits the solver's step length.

## Acoustics

- <a id="levine1948"></a>**Levine, H. and Schwinger, J. (1948).** "On the radiation of sound from an
  unflanged circular pipe." *Physical Review* 73, 383–406.
  [doi:10.1103/PhysRev.73.383](https://doi.org/10.1103/PhysRev.73.383)
  — The exact reflection coefficient at an open pipe end, which the open-end model is checked
  against.
- <a id="kinsler2000"></a>**Kinsler, L. E., Frey, A. R., Coppens, A. B. and Sanders, J. V.
  (2000).** *Fundamentals of Acoustics*, 4th ed. Wiley.
  — Textbook background for pipe resonances, plane waves and the cut-on of higher duct modes,
  monopole radiation and outdoor sound.
- <a id="pierce2019"></a>**Pierce, A. D. (2019).** *Acoustics: An Introduction to Its Physical
  Principles and Applications*, 3rd ed. Springer.
  [doi:10.1007/978-3-030-11214-1](https://doi.org/10.1007/978-3-030-11214-1)
  — The same background at a more advanced level, including radiation from a baffled piston.
- <a id="lighthill1952"></a>**Lighthill, M. J. (1952).** "On sound generated aerodynamically.
  I. General theory." *Proceedings of the Royal Society A* 211, 564–587.
  [doi:10.1098/rspa.1952.0060](https://doi.org/10.1098/rspa.1952.0060)
  — The foundation of flow-noise theory, behind the valve's turbulence noise.
- <a id="tam1998"></a>**Tam, C. K. W. (1998).** "Jet noise: since 1952." *Theoretical and
  Computational Fluid Dynamics* 10, 393–405.
  [doi:10.1007/s001620050072](https://doi.org/10.1007/s001620050072)
  — Review of jet noise, including its peak near a Strouhal number of about 0.2.

## Heat transfer

- <a id="dittus1930"></a>**Dittus, F. W. and Boelter, L. M. K. (1930).** "Heat transfer in
  automobile radiators of the tubular type." *University of California Publications in
  Engineering* 2, 443–461. Reprinted in *International Communications in Heat and Mass Transfer*
  12 (1985), 3–22.
  [doi:10.1016/0735-1933(85)90003-X](https://doi.org/10.1016/0735-1933(85)90003-X)
  — The gas-to-wall heat transfer correlation used in the pipes.
- <a id="hilpert1933"></a>**Hilpert, R. (1933).** "Wärmeabgabe von geheizten Drähten und Rohren im
  Luftstrom." *Forschung auf dem Gebiete des Ingenieurwesens* 4, 215–224.
  [doi:10.1007/BF02719754](https://doi.org/10.1007/BF02719754)
  — Heat loss from a pipe in moving air, used for the outside of the pipe wall.

## Engines

- <a id="heywood2018"></a>**Heywood, J. B. (2018).** *Internal Combustion Engine Fundamentals*,
  2nd ed. McGraw-Hill Education.
  — The standard engine textbook. Covers the filling-and-emptying cylinder model, the Wiebe burn
  function, compressible flow through valves, polytropic compression, cycle-to-cycle variation
  and residual gas.
- <a id="woschni1967"></a>**Woschni, G. (1967).** "A universally applicable equation for the
  instantaneous heat transfer coefficient in the internal combustion engine." SAE Technical Paper
  670931. [doi:10.4271/670931](https://doi.org/10.4271/670931)
  — The in-cylinder heat loss model.
