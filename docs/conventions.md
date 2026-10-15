# Conventions

These conventions apply to every module. Mixing conventions is the most common source of sign errors in attitude software, so any deviation must be explicit and converted at the interface.

## Units

SI units internally (m, s, kg, N, T). Angles in radians. Degrees and arcseconds appear only in configuration comments and plots.

## Reference frames

| Frame | Symbol | Definition |
|---|---|---|
| Inertial | ECI | GCRF / J2000 |
| Earth-fixed | ECEF | ITRF |
| Local orbital | LVLH | z: nadir (−r̂), y: negative orbit normal (−ĥ), x: completes the triad (≈ velocity) |
| Body | B | Fixed to spacecraft structure, origin at CoM |

## Quaternions

Same convention as B5G-AOCS-EGSE (`QMul`, `QInv`, `M2Q`, `FrameA2B`) and Aerospace Toolbox (`quatmultiply`, `quat2dcm`), so results can be compared without conversion.

**Layout (scalar-first):**

$$
q = \begin{bmatrix} q_s \\ \mathbf{q}_v \end{bmatrix}, \qquad
q_s = \cos\frac{\theta}{2}, \quad \mathbf{q}_v = \hat{e}\sin\frac{\theta}{2}
$$

**Attitude matrix** (maps reference-frame vectors into the body frame, $\mathbf{v}_B = A(q)\,\mathbf{v}_I = q^{-1} \otimes \mathbf{v}_I \otimes q$):

$$
A(q) = \left(q_s^2 - \|\mathbf{q}_v\|^2\right) I_3 + 2\,\mathbf{q}_v\mathbf{q}_v^T - 2\,q_s\,[\mathbf{q}_v\times]
$$

**Product (Hamilton convention):**

$$
q \otimes p = \begin{bmatrix} q_s p_s - \mathbf{q}_v \cdot \mathbf{p}_v \\ q_s\,\mathbf{p}_v + p_s\,\mathbf{q}_v + \mathbf{q}_v \times \mathbf{p}_v \end{bmatrix}
$$

Frame rotations compose left to right: $q_{a2c} = q_{a2b} \otimes q_{b2c}$, hence $A(q \otimes p) = A(p)\,A(q)$.

**Kinematics** ($\boldsymbol{\omega}$ = body rate w.r.t. inertial, in body):

$$
\dot{q} = \tfrac{1}{2}\, q \otimes \begin{bmatrix} 0 \\ \boldsymbol{\omega} \end{bmatrix}, \qquad \dot{A} = -[\boldsymbol{\omega}\times]A
$$

**Discrete propagation** (exact for constant $\boldsymbol{\omega}$ over $\Delta t$):

$$
q_{k+1} = q_k \otimes \begin{bmatrix} \cos\!\left(\tfrac{\|\boldsymbol{\omega}\|\Delta t}{2}\right) \\ \boldsymbol{\psi} \end{bmatrix}, \quad
\boldsymbol{\psi} = \sin\!\left(\tfrac{\|\boldsymbol{\omega}\|\Delta t}{2}\right)\frac{\boldsymbol{\omega}}{\|\boldsymbol{\omega}\|}
$$

**Sign:** `att.fromDCM` returns $q_s \ge 0$ (same as EGSE `M2Q`).

**Attitude error:** `att.errorAngle` returns the total angle of $\delta q = q_A \otimes q_B^{-1}$ in rad. EGSE `QErrAng` instead returns the 3-axis small-angle vector $2\,\delta\mathbf{q}_v$ in arcsec.

## Interoperability with Aerospace Toolbox

`quatmultiply` and `quat2dcm` use the same layout and product but take 1x4 **row** vectors; `att.*` uses 4x1 **columns**. Pass `q'`. Checked by `QuaternionTest/aerospaceToolboxConsistency` when the toolbox is installed. Inside `src/fsw/` use `att.*` only.

## Time systems (`+timesys`)

| Scale | Relation |
|---|---|
| TAI | UTC + ΔAT (leap seconds, MATLAB `leapseconds`; 37 s since 2017-01-01) |
| TT | TAI + 32.184 s |
| UT1 | UTC + DUT1 (EOP) |
| GPS | TAI − 19 s; full week number (no 1024 rollover), epoch 1980-01-06 |

Julian dates are plain doubles, which resolve about 40 µs; this is the precision floor of every JD-based time. Sidereal time `timesys.gmst` uses the IAU-82 formula, the same as EGSE `iers_gmst96`.

## ECI ↔ ECEF (`+frames`, IAU-76/FK5)

Same model as B5G-AOCS-EGSE `ecef2eci_76fk5` (Vallado Sec. 3.7). ECI is J2000 (mean equator and equinox):

$$
\mathbf{r}_{ECI} = P\,N\,R\,W\,\mathbf{r}_{ECEF}, \qquad
\mathbf{v}_{ECI} = P\,N\,R\left(W\,\mathbf{v}_{ECEF} + \boldsymbol{\omega}_\oplus \times W\,\mathbf{r}_{ECEF}\right)
$$

- $P$: IAU-76 precession, $N$: IAU-80 nutation (106 terms), both evaluated in TT
- $R = \mathrm{ROT3}(-\mathrm{GAST})$, GAST from UT1 with the IAU-94 equation-of-equinoxes terms
- $W = \mathrm{ROT1}(y_p)\,\mathrm{ROT2}(x_p)$ (polar motion)
- $\omega_\oplus = 7.292115146706979\times10^{-5}\,(1 - \mathrm{LOD}/86400)$ rad/s
- EOP celestial-pole offsets (dPsi, dEps) are not applied, as in the EGSE `fk5_76.N`

EOP are passed as a struct `eop` with fields `dut1` [s], `xp`, `yp` [rad], `lod` [s]; missing fields default to 0.

**Known difference from EGSE:** `rot_mtx.m` passes a Julian date to `omega_lunar`, which expects Julian centuries. This only changes the two small equation-of-equinoxes terms, by at most 0.0027″ (9 cm at 7000 km; measured 7e-10 rad = 3.5 mm on 2026-10-15). `frames.fk5` uses centuries of TT.

**LVLH:** `frames.eciToLvlh` returns $A_{ECI\to LVLH}$ and the matching quaternion, the same as EGSE `PVi2Qi2l`.

## Orbit propagation (`+orbit`)

- State $x = [\mathbf{r};\,\mathbf{v}]$ in ECI (J2000), SI units; time as seconds since a UTC Julian date `jd0`.
- Gravity: `degree = 0` is two-body with EGM2008 GM $= 3.986004415\times10^{14}$ m³/s². `degree ≥ 2` uses Aerospace Toolbox `gravitysphericalharmonic` (EGM2008, degree = order), evaluated in ECEF and rotated back with `frames.fk5`.
- Integrator: fixed-step classical RK4 (`num.rk4Step`, shared with attitude). Against `ode113` (RelTol 1e-12) with EGM2008 degree 6 at 500 km, the position difference after one orbit is 0.3 mm (dt = 1 s), 0.7 mm (dt = 5 s) and 1.6 cm (dt = 10 s).
- Cost: about 0.5–1 ms per acceleration evaluation, mostly inside `gravitysphericalharmonic`; one day at dt = 5 s with degree 6 takes about 20 s.

## Environment (`+env`)

All functions take an ECI position [m] and a UTC Julian date and return ECI vectors in SI units. Geodetic positions use WGS84 (`ecef2lla`) after `frames.fk5`.

| Function | Model | Notes |
|---|---|---|
| `env.sunPosition` | JPL DE405 (`planetEphemeris`) | Geocentric, evaluated at TT |
| `env.shadowFraction` | Conical Earth shadow, disk overlap | 1 sunlit, 0 umbra; Montenbruck & Gill 3.4.2 |
| `env.magneticField` | IGRF-14 (`igrfmagm`) | Returned in tesla |
| `env.atmosphereDensity` | NRLMSISE-00 (`atmosnrlmsise00`) | `sw` struct: `f107`, `f107a`, `ap` |

## Disturbance torques (`+torque`)

All torques are in the body frame [N m]; `q` is the ECI→body quaternion.

| Function | Model |
|---|---|
| `torque.gravityGradient` | $\boldsymbol{\tau} = \frac{3\mu}{r^5}\,\mathbf{r}_b \times J\,\mathbf{r}_b$ |
| `torque.aerodynamic` | Flat plate, $\mathbf{F} = -\tfrac12\rho C_d A\,\lVert\mathbf{v}_{rel}\rVert\,\mathbf{v}_{rel}$, $\mathbf{v}_{rel} = \mathbf{v} - \boldsymbol{\omega}_\oplus \times \mathbf{r}$ |
| `torque.solarRadiation` | Flat plate facing the Sun, $\mathbf{F} = -P\,(AU/d)^2\,\nu\,C_r A\,\hat{\mathbf{s}}$, $P = 4.56\times10^{-6}$ N/m² |
| `torque.magnetic` | $\boldsymbol{\tau} = \mathbf{m} \times \mathbf{B}_b$ |

Aerodynamic and SRP torques use $\boldsymbol{\tau} = \mathbf{c}_p \times \mathbf{F}$ with the single `sc.cpOffset` (centre of pressure − CoM) from the mission file. Area does not change with attitude; a multi-plate model can replace this later.

## Rigid-body attitude dynamics (`+attitude`)

State $x = [q;\,\boldsymbol{\omega}]$: $q$ ECI→body (scalar-first), $\boldsymbol{\omega}$ body rate w.r.t. ECI in body [rad/s].

$$
\dot{q} = \tfrac12\,q \otimes \begin{bmatrix} 0 \ \boldsymbol{\omega} \end{bmatrix}, \qquad
J\,\dot{\boldsymbol{\omega}} = \boldsymbol{\tau} - \boldsymbol{\omega} \times J\boldsymbol{\omega}
$$

`attitude.propagate` integrates with fixed-step RK4 (`num.rk4Step`) and renormalizes $q$ after each step. The torque is a function `tauFun(t, q, w)` evaluated at every RK4 stage, so `+torque` functions can be plugged in directly. Reaction-wheel momentum is not included yet (Phase 3).
