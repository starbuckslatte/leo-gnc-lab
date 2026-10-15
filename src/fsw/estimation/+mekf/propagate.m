function f = propagate(f, wMeas, dt, sigmaV, sigmaU)
%MEKF.PROPAGATE Time update with one gyro sample.
%   q_hat <- q_hat (x) exp((wMeas - beta) dt),  beta constant
%   Error dynamics: dtheta' = -[w x] dtheta - dbeta - eta_v,  dbeta' = eta_u
%   Phi = expm(F dt); Q discrete (Crassidis & Junkins, Eq. 7.43, small dt).
%   sigmaV [rad/s^0.5] (ARW), sigmaU [rad/s^1.5] (RRW): gyro noise parameters.
w = wMeas(:) - f.beta;
f.q = att.propagate(f.q, w, dt);
F = [-att.skew(w), -eye(3); zeros(3), zeros(3)];
Phi = expm(F*dt);
I3 = eye(3);
Q = [(sigmaV^2*dt + sigmaU^2*dt^3/3)*I3, -(sigmaU^2*dt^2/2)*I3;
     -(sigmaU^2*dt^2/2)*I3,               (sigmaU^2*dt)*I3];
f.P = Phi*f.P*Phi' + Q;
f.P = (f.P + f.P')/2;
end
