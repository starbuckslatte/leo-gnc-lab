function [t, Q, W, H] = propagate(q0, w0, J, tauFun, dt, tEnd, rw)
%ATTITUDE.PROPAGATE Fixed-step RK4 rigid-body attitude propagation.
%   q0: initial ECI->body quaternion (scalar-first); w0: body rate [rad/s]
%   J: inertia [kg m^2]; tauFun(t, q, w) -> external body torque [N m],
%   evaluated at every RK4 stage; dt step [s]; tEnd duration [s].
%   rw (optional): reaction wheels, struct with fields
%       G      spin axes (actuators.rwConfig), h0 initial momenta [N m s],
%       p      wheel parameters (actuators.rwTorque),
%       cmdFun(t, q, w, h) -> commanded motor torque per wheel [N m].
%   The quaternion is renormalized after each step.
%   Returns t (1xN), Q (4xN), W (3xN), H (nWheel x N, empty without wheels).
n = round(tEnd/dt);
t = (0:n)*dt;
if nargin < 7
    x0 = [q0(:); w0(:)];
    f = @(tt, x) attitude.derivative(x, J, tauFun(tt, x(1:4), x(5:7)));
else
    x0 = [q0(:); w0(:); rw.h0(:)];
    f = @(tt, x) attitude.derivative(x, J, tauFun(tt, x(1:4), x(5:7)), rw.G, ...
        actuators.rwTorque(rw.cmdFun(tt, x(1:4), x(5:7), x(8:end)), x(8:end), rw.p));
end
X = zeros(numel(x0), n+1);
X(:, 1) = x0;
for k = 1:n
    x = num.rk4Step(f, t(k), X(:, k), dt);
    x(1:4) = att.normalize(x(1:4));
    X(:, k+1) = x;
end
Q = X(1:4, :);
W = X(5:7, :);
H = X(8:end, :);
end
