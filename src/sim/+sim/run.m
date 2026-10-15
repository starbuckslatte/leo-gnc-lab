function out = run(m, fsw)
%SIM.RUN Closed-loop simulation: truth dynamics + sensors + FSW + actuators.
%   out = sim.run(m, fsw)
%   m   : mission struct (missions/*.m)
%   fsw : flight software (see sim.nullFsw for the interface)
%
%   Timing. Every period must be an integer multiple of m.sim.dt; events
%   are triggered by integer tick counters, so rates never drift.
%     attitude truth   every m.sim.dt (RK4, wheels coupled, torque held over dt)
%     orbit + env      every m.sim.orbitDt (RK4; Sun, shadow, field and
%                      density held constant in between)
%     each sensor      at its own rate, sampled from the truth at that tick
%     FSW              at m.fsw.rate; commands held until the next FSW step
%   At a tick where several events coincide the order is:
%   env update -> sensor sampling -> FSW -> actuators -> orbit and attitude
%   steps, so a command issued at a tick acts from that tick on.
%   Thrust is held in the body frame of the orbit-step start over orbitDt.
%
%   meas (passed to fsw.step) holds the latest sample of each sensor with
%   its sample time t and a flag new (= sampled since the previous FSW step):
%     gyro.w, str.q / str.valid, mag.b, sun.s / sun.valid, gnss.fix
%
%   out: logs at the FSW rate (t, r, v, q, w, h, tau, meas, cmd), plus
%   P (periods in ticks) and dt.
rng(m.sim.seed);
dt = m.sim.dt;
nTick = round(m.sim.duration / dt);
P.orbit = periodTicks(m.sim.orbitDt, dt);
P.fsw   = periodTicks(1/m.fsw.rate, dt);
P.gyro  = periodTicks(1/m.sensors.gyro.rate, dt);
P.str   = periodTicks(1/m.sensors.starTracker.rate, dt);
P.mag   = periodTicks(1/m.sensors.magnetometer.rate, dt);
P.sun   = periodTicks(1/m.sensors.sunSensor.rate, dt);
P.gnss  = periodTicks(1/m.sensors.gnss.rate, dt);

eop  = struct();
jd0  = juliandate(m.epoch);
dist = m.sim.disturbances;
J    = m.sc.inertia;
G    = actuators.rwConfig(m.actuators.rw);

% --- initial conditions ---------------------------------------------------
RE = 6378137;
raan = m.orbit.raan;
if isnan(raan), raan = orbit.raanFromLtan(jd0, m.orbit.ltan); end
[r, v] = orbit.keplerToRv(RE + m.orbit.altitude, m.orbit.ecc, m.orbit.inc, ...
                          raan, m.orbit.argp, m.orbit.trueAnomaly);
xOrb = [r; v];
q0 = m.sim.q0;
if isempty(q0), [~, q0] = frames.eciToLvlh(r, v); end
xAtt = [q0(:); m.sim.w0(:); m.sim.rwH0(:)];

S.gyro = sensors.gyroInit(m.sensors.gyro);
S.str  = sensors.starTrackerInit(m.sensors.starTracker);
S.mag  = sensors.magnetometerInit(m.sensors.magnetometer);
S.sun  = sensors.sunSensorInit(m.sensors.sunSensor);
S.gnss = sensors.gnssInit(m.sensors.gnss);
meas = struct('gyro', struct('w', nan(3, 1), 't', NaN, 'new', false), ...
              'str',  struct('q', nan(4, 1), 'valid', false, 't', NaN, 'new', false), ...
              'mag',  struct('b', nan(3, 1), 't', NaN, 'new', false), ...
              'sun',  struct('s', nan(3, 1), 'valid', false, 't', NaN, 'new', false), ...
              'gnss', struct('fix', [], 't', NaN, 'new', false));
fswState = fsw.init(m);
nW = size(G, 2);
nT = size(m.actuators.thruster.directions, 2);
cmd = struct('rw', zeros(nW, 1), 'mtq', zeros(3, 1), 'thruster', zeros(nT, 1));
mtq = zeros(3, 1); fThr = zeros(3, 1); tauThr = zeros(3, 1);
E = struct();

% --- log --------------------------------------------------------------------
nLog = floor(nTick / P.fsw) + 1;
L.t = zeros(1, nLog); L.r = zeros(3, nLog); L.v = zeros(3, nLog);
L.q = zeros(4, nLog); L.w = zeros(3, nLog); L.h = zeros(nW, nLog);
L.tau = struct('gg', zeros(3, nLog), 'aero', zeros(3, nLog), 'srp', zeros(3, nLog), ...
               'mag', zeros(3, nLog), 'thr', zeros(3, nLog));
L.meas = struct('gyro', zeros(3, nLog), 'gyroT', zeros(1, nLog), ...
                'str', zeros(4, nLog), 'strValid', false(1, nLog), 'strT', zeros(1, nLog), ...
                'mag', zeros(3, nLog), 'sun', zeros(3, nLog), 'sunValid', false(1, nLog));
L.cmd = struct('rw', zeros(nW, nLog), 'mtq', zeros(3, nLog), 'thruster', zeros(nT, nLog));
iLog = 0;

for k = 0:nTick
    t = k*dt;
    q = xAtt(1:4); w = xAtt(5:7); h = xAtt(8:end);

    % --- orbit / environment (slow rate) ---------------------------------
    if mod(k, P.orbit) == 0
        jd = jd0 + t/86400;
        r = xOrb(1:3); v = xOrb(4:6);
        E.rSun = env.sunPosition(jd);
        E.nu   = env.shadowFraction(r, E.rSun);
        E.B    = env.magneticField(r, jd, eop);
        if dist.aero
            E.rho = env.atmosphereDensity(r, jd, eop, m.env.atmosphere);
        else
            E.rho = 0;
        end
    end

    % --- sensors (sampled from the truth at this tick) -------------------
    if mod(k, P.gyro) == 0
        [meas.gyro.w, S.gyro] = sensors.gyro(w, S.gyro, m.sensors.gyro, P.gyro*dt);
        meas.gyro.t = t; meas.gyro.new = true;
    end
    if mod(k, P.str) == 0
        [meas.str.q, meas.str.valid, S.str] = sensors.starTracker(q, r, E.rSun, S.str, m.sensors.starTracker);
        meas.str.t = t; meas.str.new = true;
    end
    if mod(k, P.mag) == 0
        [meas.mag.b, S.mag] = sensors.magnetometer(E.B, q, S.mag, m.sensors.magnetometer, mtq);
        meas.mag.t = t; meas.mag.new = true;
    end
    if mod(k, P.sun) == 0
        [meas.sun.s, meas.sun.valid, S.sun] = sensors.sunSensor(q, r, E.rSun, E.nu, S.sun, m.sensors.sunSensor);
        meas.sun.t = t; meas.sun.new = true;
    end
    if mod(k, P.gnss) == 0
        [meas.gnss.fix, S.gnss] = sensors.gnss(r, v, jd0 + t/86400, eop, S.gnss, m.sensors.gnss);
        meas.gnss.t = t; meas.gnss.new = true;
    end

    % --- flight software + actuators, log --------------------------------
    isFsw = mod(k, P.fsw) == 0;
    if isFsw
        [c, fswState] = fsw.step(fswState, meas, t);
        for f = ["rw", "mtq", "thruster"]
            if ~isempty(c.(f)), cmd.(f) = c.(f)(:); end
        end
        for f = ["gyro", "str", "mag", "sun", "gnss"]
            meas.(f).new = false;
        end
        mtq = actuators.mtqDipole(cmd.mtq, m.actuators.mtq);
        [fThr, tauThr] = actuators.thruster(cmd.thruster, m.actuators.thruster, P.fsw*dt);

        iLog = iLog + 1;
        L.t(iLog) = t; L.r(:, iLog) = r; L.v(:, iLog) = v;
        L.q(:, iLog) = q; L.w(:, iLog) = w; L.h(:, iLog) = h;
        L.meas.gyro(:, iLog) = meas.gyro.w; L.meas.gyroT(iLog) = meas.gyro.t;
        L.meas.str(:, iLog) = meas.str.q;   L.meas.strValid(iLog) = meas.str.valid;
        L.meas.strT(iLog) = meas.str.t;
        L.meas.mag(:, iLog) = meas.mag.b;
        L.meas.sun(:, iLog) = meas.sun.s;   L.meas.sunValid(iLog) = meas.sun.valid;
        L.cmd.rw(:, iLog) = cmd.rw; L.cmd.mtq(:, iLog) = mtq; L.cmd.thruster(:, iLog) = cmd.thruster;
    end
    if k == nTick, break; end

    % --- orbit step (after actuators, so thrust commanded now acts now) ---
    if mod(k, P.orbit) == 0
        aThr = att.toDCM(q)' * fThr / m.sc.mass;
        fo = @(tt, x) orbit.derivative(tt, x, jd0, eop, m.env.gravity.degree, aThr);
        xOrb = num.rk4Step(fo, t, xOrb, m.sim.orbitDt);
    end

    % --- external torque (held over dt) and attitude step -----------------
    tau = struct('gg', zeros(3, 1), 'aero', zeros(3, 1), 'srp', zeros(3, 1), ...
                 'mag', zeros(3, 1), 'thr', tauThr);
    if dist.gravityGradient, tau.gg = torque.gravityGradient(r, q, J); end
    if dist.aero, tau.aero = torque.aerodynamic(r, v, q, E.rho, m.sc); end
    if dist.srp, tau.srp = torque.solarRadiation(r, E.rSun, q, E.nu, m.sc); end
    dipole = mtq;
    if dist.magnetic, dipole = dipole + m.sc.residualDipole(:); end
    tau.mag = torque.magnetic(E.B, q, dipole);
    if isFsw
        for f = ["gg", "aero", "srp", "mag", "thr"]
            L.tau.(f)(:, iLog) = tau.(f);
        end
    end
    tauTot = tau.gg + tau.aero + tau.srp + tau.mag + tau.thr;
    fa = @(~, x) attitude.derivative(x, J, tauTot, G, ...
        actuators.rwTorque(cmd.rw, x(8:end), m.actuators.rw));
    xAtt = num.rk4Step(fa, t, xAtt, dt);
    xAtt(1:4) = att.normalize(xAtt(1:4));
end
out = L;
out.P = P;
out.dt = dt;
end

function n = periodTicks(T, dt)
% Period T in integer ticks of dt; errors if T is not an exact multiple.
n = round(T / dt);
if n < 1 || abs(n*dt - T) > 1e-9*max(T, 1)
    error('sim:run:rate', 'Period %.6g s is not an integer multiple of dt = %.6g s.', T, dt);
end
end
