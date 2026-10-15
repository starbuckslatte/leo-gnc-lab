%DEMO_PHASE1_QUATERNION Propagate a tumbling attitude and check consistency.
%   Compares discrete propagation (att.propagate) with ode45 integration
%   of the quaternion kinematics, and plots the angular difference.
setupPaths();

q0 = att.fromAxisAngle([1; 1; 0], deg2rad(30));
wFun = @(t) [0.05*sin(0.01*t); 0.03*cos(0.02*t); 0.01];   % rad/s, body frame
dt = 0.1;  T = 600;  t = 0:dt:T;

q = zeros(4, numel(t));  q(:, 1) = q0;
for k = 1:numel(t)-1
    % Midpoint rate: second-order accurate for time-varying w
    q(:, k+1) = att.propagate(q(:, k), wFun(t(k) + dt/2), dt);
end

opts = odeset('RelTol', 1e-12, 'AbsTol', 1e-12);
[~, Qref] = ode45(@(tt, x) att.derivative(x, wFun(tt)), t, q0, opts);

err = arrayfun(@(k) att.errorAngle(q(:, k), Qref(k, :)'), 1:numel(t));

figure('Name', 'Phase 1: quaternion propagation');
subplot(2, 1, 1);
plot(t, q'); grid on;
legend('q_1', 'q_2', 'q_3', 'q_4 (scalar)'); ylabel('quaternion');
title('Attitude quaternion (scalar-last, ECI \rightarrow body)');
subplot(2, 1, 2);
semilogy(t, rad2deg(err)*3600 + eps); grid on;
xlabel('time [s]'); ylabel('error [arcsec]');
title('Discrete propagation vs ode45');
