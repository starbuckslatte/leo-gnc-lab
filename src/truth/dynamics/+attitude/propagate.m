function [t, Q, W] = propagate(q0, w0, J, tauFun, dt, tEnd)
%ATTITUDE.PROPAGATE Fixed-step RK4 rigid-body attitude propagation.
%   q0: initial ECI->body quaternion (scalar-first); w0: body rate [rad/s]
%   J: inertia [kg m^2]; tauFun(t, q, w) -> body torque [N m], evaluated at
%   every RK4 stage; dt step [s]; tEnd duration [s].
%   The quaternion is renormalized after each step.
%   Returns t (1xN), Q (4xN), W (3xN).
n = round(tEnd/dt);
t = (0:n)*dt;
X = zeros(7, n+1);
X(:, 1) = [q0(:); w0(:)];
f = @(tt, x) attitude.derivative(x, J, tauFun(tt, x(1:4), x(5:7)));
for k = 1:n
    x = num.rk4Step(f, t(k), X(:, k), dt);
    x(1:4) = att.normalize(x(1:4));
    X(:, k+1) = x;
end
Q = X(1:4, :);
W = X(5:7, :);
end
