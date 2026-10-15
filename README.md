# leo-gnc-lab

A modular MATLAB/Simulink sandbox for spacecraft **Guidance, Navigation & Control (GNC)** in **LEO and VLEO**, built step by step from first principles: environment and dynamics, sensor models, attitude and orbit estimation, closed-loop control, mode management with FDIR, and optical navigation.

> **Status:** Phase 1 (foundations) complete — release `v0.1.0`.

## Design principles

- **Truth / flight-software separation.** `src/truth/` models the physical world (environment, dynamics, sensors, actuators). `src/fsw/` holds the onboard algorithms (estimation, guidance, control, modes). They communicate **only** through sensor measurements and actuator commands — flight software never sees the true state.
- **Missions as configuration.** Each scenario is a single file in `missions/` returning a struct. The same code runs a 500 km imaging mission or a 250 km VLEO mission.
- **One attitude convention, defined once.** All quaternion math lives in the `att` namespace (`src/utils/+att/`). See [docs/conventions.md](docs/conventions.md).
- **Multi-rate by design.** Truth dynamics, each sensor, and the FSW loop run at their own rates.
- **Verified, not just plotted.** Every module ships with unit tests; estimators are validated with ±3σ bounds, NEES, and Monte Carlo campaigns.

## Repository layout

```
leo-gnc-lab/
├── missions/              Scenario configuration files
├── src/
│   ├── truth/
│   │   ├── environment/   Gravity, atmosphere, magnetic field, Sun/Moon, eclipse
│   │   ├── dynamics/      Orbit and attitude dynamics, disturbance torques
│   │   ├── sensors/       Gyro, star tracker, sun sensor, magnetometer, GNSS, camera
│   │   └── actuators/     Reaction wheels, magnetorquers, thrusters
│   ├── fsw/
│   │   ├── estimation/    MEKF, orbit determination
│   │   ├── guidance/      Pointing modes, slews
│   │   ├── control/       Detumble, attitude control, momentum management
│   │   └── modes/         Mode manager, FDIR
│   ├── sim/               Multi-rate scheduler, simulation runner
│   └── utils/+att/        Quaternion / attitude utilities
├── tests/                 matlab.unittest test classes
├── scripts/               Demos and analysis campaigns
├── docs/                  Conventions and derivations
└── results/               Curated figures
```

## Getting started

Requires MATLAB R2020b or later. Later phases use Aerospace Toolbox (and optionally Simulink).

```matlab
setupPaths            % add repository folders to the path
runAllTests           % run the full unit test suite
demo_phase1_quaternion
```

### Verification and releases

`buildfile.m` defines a static code check (`check`) and the full test suite (`test`). Run both with `buildtool` in MATLAB before tagging a release. Every tagged release (`v0.1.0`, …) is a state in which all tests pass. If a later change turns a test red, compare against or restore from the last tag:

```bash
git diff v0.1.0 -- src/
```

Quaternion functions are called through their namespace, e.g.:

```matlab
q  = att.fromAxisAngle([0;0;1], pi/2);   % scalar-first
A  = att.toDCM(q);                        % ECI -> body
q2 = att.propagate(q, [0;0;0.01], 0.1);   % exact for constant rate
```

Time and frame transformations:

```matlab
jd = timesys.jd([2026 10 15 3 25 7.5]);              % UTC Julian date
[w, sow] = timesys.utcToGps(jd);                     % GPS week, seconds of week
eop = struct('dut1', 0.08, 'xp', 0, 'yp', 0);        % EOP (rad for xp, yp)
[rEci, vEci] = frames.ecefToEci(rEcef, vEcef, jd, eop);
[A, q] = frames.eciToLvlh(rEci, vEci);               % ECI -> LVLH
```

## Roadmap

### v1.0 — Core attitude GNC
- [x] **Phase 1 – Foundations:** quaternion utilities ✅, frame transformations (ECI/ECEF IAU-76/FK5, LVLH) ✅, time systems (UTC/UT1/TT/GPS) ✅, unit tests ✅, buildtool verification ✅
- [ ] **Phase 2 – Environment & dynamics:** orbit propagation (two-body, EGM2008 spherical harmonics, RK4) ✅, atmosphere (NRLMSISE-00) ✅, magnetic field (IGRF-14) ✅, Sun/eclipse (DE405, conical shadow) ✅, gravity-gradient / aerodynamic / SRP / magnetic disturbance torques ✅; disturbance-vs-altitude study
- [ ] **Phase 3 – Sensors & actuators:** Farrenkopf gyro, star tracker, sun sensor, magnetometer, GNSS; reaction wheels (saturation, friction), magnetorquers, thrusters
- [ ] **Phase 4 – Attitude determination:** MEKF with ±3σ consistency, NEES, Monte Carlo
- [ ] **Phase 5 – Attitude guidance & control:** B-dot detumble, quaternion feedback, nadir / target pointing, large-angle slews, momentum unloading

### v2.0 — Navigation, VLEO, autonomy
- [ ] **Phase 6 – Orbit determination:** GNSS-based EKF; drag / ballistic-coefficient estimation for VLEO
- [ ] **Phase 7 – VLEO topics:** aerodynamic torque effects, aero-stabilisation, drag make-up and orbit maintenance, ΔV budget
- [ ] **Phase 8 – Modes & FDIR:** mode manager, safe mode, fault injection (gyro failure, wheel jam) and recovery
- [ ] **Phase 9 – Optical navigation:** pinhole camera model, landmark-based attitude/pointing-bias estimation; optional extension to lunar descent

### Analysis practices adopted throughout
- Pointing performance expressed with ECSS-E-ST-60-10C (Control performance) terminology
- Fidelity increased incrementally: a simple end-to-end loop first, then one model upgrade at a time with its performance impact documented

## Conventions (summary)

| Item | Convention |
|---|---|
| Units | SI internally, angles in rad |
| Quaternion | Scalar-first `[qs q1 q2 q3]'` (same as B5G-AOCS-EGSE) |
| Product | Hamilton: `Qa2c = Qa2b ⊗ Qb2c` |
| Attitude matrix | `A(q)` maps reference (ECI) → body (passive) |
| Angular rate | Body w.r.t. ECI, expressed in body |

Full details in [docs/conventions.md](docs/conventions.md).

## References

- J. L. Crassidis and J. L. Junkins, *Optimal Estimation of Dynamic Systems*, 2nd ed., CRC Press, 2011.
- F. L. Markley and J. L. Crassidis, *Fundamentals of Spacecraft Attitude Determination and Control*, Springer, 2014.
- D. A. Vallado, *Fundamentals of Astrodynamics and Applications*, 4th ed., Microcosm Press, 2013.
- ECSS-E-ST-60-10C, *Space engineering – Control performance*, ESA.

## Disclaimer

All parameters are illustrative values from textbooks and public datasheets. This repository contains no proprietary, program-specific, or export-controlled information.

## License

MIT — see [LICENSE](LICENSE).
