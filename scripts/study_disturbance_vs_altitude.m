%STUDY_DISTURBANCE_VS_ALTITUDE Peak disturbance torques vs circular-orbit altitude.
%   For each altitude, samples one circular orbit (inclination and LTAN from
%   the mission file) and records the peak magnitude of each torque.
%
%   Attitude: LVLH (nadir pointing) tilted by TILT_DEG about the LVLH x axis.
%   With diagonal inertia and zero tilt the gravity-gradient torque is zero;
%   45 deg is its worst case.
%   Aerodynamic and SRP torques use the worst-case lever arm |F| * |cpOffset|,
%   so the result does not depend on how cpOffset happens to align with the
%   flow or the Sun line. Aerodynamic torque is shown for low / moderate /
%   high solar activity.
%   Output: figure and results/disturbance_vs_altitude.png
setupPaths();

TILT_DEG  = 45;
ALTITUDES = (200:25:800)*1e3;      % m
N_SAMPLES = 120;                   % per orbit
SOLAR = struct('name', {'F10.7 = 70', 'F10.7 = 150', 'F10.7 = 250'}, ...
               'f107', {70, 150, 250}, 'f107a', {70, 150, 250}, 'ap', {4, 4, 15});

m   = leo_500km_imaging();
MU  = 3.986004415e14;
RE  = 6378137;
eop = struct();
jd0 = juliandate(m.epoch);
rSun = env.sunPosition(jd0);       % fixed over one orbit
% RAAN from LTAN: ascending node at local time LTAN
raan = atan2(rSun(2), rSun(1)) + (m.orbit.ltan - 12)*pi/12;
qTilt = att.fromAxisAngle([1; 0; 0], deg2rad(TILT_DEG));
cosI = cos(m.orbit.inc); sinI = sin(m.orbit.inc);
R = [cos(raan) -sin(raan) 0; sin(raan) cos(raan) 0; 0 0 1] * [1 0 0; 0 cosI -sinI; 0 sinI cosI];

lever = norm(m.sc.cpOffset);
nAlt = numel(ALTITUDES);
peak = struct('gg', zeros(1, nAlt), 'srp', zeros(1, nAlt), 'mag', zeros(1, nAlt), ...
              'aero', zeros(numel(SOLAR), nAlt));
for i = 1:nAlt
    a = RE + ALTITUDES(i);
    n = sqrt(MU/a^3);
    for u = linspace(0, 2*pi, N_SAMPLES + 1)
        r = R * a*[cos(u); sin(u); 0];
        v = R * a*n*[-sin(u); cos(u); 0];
        jd = jd0 + u/n/86400;
        [~, qL] = frames.eciToLvlh(r, v);
        q = att.multiply(qL, qTilt);
        peak.gg(i)  = max(peak.gg(i),  norm(torque.gravityGradient(r, q, m.sc.inertia)));
        nu = env.shadowFraction(r, rSun);
        [~, f] = torque.solarRadiation(r, rSun, q, nu, m.sc);
        peak.srp(i) = max(peak.srp(i), norm(f) * lever);
        B = env.magneticField(r, jd, eop);
        peak.mag(i) = max(peak.mag(i), norm(torque.magnetic(B, q, m.sc.residualDipole)));
        for s = 1:numel(SOLAR)
            rho = env.atmosphereDensity(r, jd, eop, SOLAR(s));
            [~, f] = torque.aerodynamic(r, v, q, rho, m.sc);
            peak.aero(s, i) = max(peak.aero(s, i), norm(f) * lever);
        end
    end
end

%% Report
h = ALTITUDES/1e3;
fprintf('\nPeak disturbance torque [N m], tilt %g deg, %s\n', TILT_DEG, m.name);
fprintf('%6s %10s %10s %10s %10s %10s %10s\n', 'h[km]', 'GG', 'Mag', 'SRP', 'Aero lo', 'Aero mid', 'Aero hi');
for hk = [250 300 400 500 600 700 800]
    k = find(h == hk);
    fprintf('%6d %10.2e %10.2e %10.2e %10.2e %10.2e %10.2e\n', hk, peak.gg(k), peak.mag(k), ...
        peak.srp(k), peak.aero(:, k));
end
for s = 1:numel(SOLAR)
    k = find(peak.aero(s, :) < max(peak.gg, peak.mag), 1);
    if isempty(k)
        fprintf('%s: aero dominates over the whole range\n', SOLAR(s).name);
    else
        fprintf('%s: aero no longer the largest above %d km\n', SOLAR(s).name, h(k));
    end
end

figure('Name', 'Disturbance torque vs altitude');
semilogy(h, peak.gg, 'LineWidth', 1.5); hold on;
semilogy(h, peak.mag, 'LineWidth', 1.5);
semilogy(h, peak.srp, 'LineWidth', 1.5);
styles = {':', '-', '--'};
for s = 1:numel(SOLAR)
    semilogy(h, peak.aero(s, :), ['k' styles{s}], 'LineWidth', 1.2);
end
grid on; xlabel('altitude [km]'); ylabel('peak torque [N m]');
legend(['Gravity gradient (' num2str(TILT_DEG) '^\circ tilt)'], 'Residual dipole', 'SRP', ...
    ['Aero, ' SOLAR(1).name], ['Aero, ' SOLAR(2).name], ['Aero, ' SOLAR(3).name], ...
    'Location', 'northeast');
title(sprintf('%s: peak disturbance torque', m.name), 'Interpreter', 'none');
root = fileparts(fileparts(mfilename('fullpath')));
exportgraphics(gcf, fullfile(root, 'results', 'disturbance_vs_altitude.png'), 'Resolution', 150);
