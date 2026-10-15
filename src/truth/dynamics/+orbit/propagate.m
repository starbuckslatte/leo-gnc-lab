function [t, X] = propagate(x0, jd0, dt, tEnd, eop, degree)
%ORBIT.PROPAGATE Fixed-step RK4 orbit propagation in ECI (J2000).
%   x0    : initial state [r; v] [m, m/s] at UTC Julian date jd0
%   dt    : step [s]; tEnd: duration [s] (rounded to a whole number of steps)
%   eop   : see frames.fk5; degree: see orbit.gravityAccel
%   Returns t (1xN) [s since jd0] and X (6xN).
n = round(tEnd/dt);
t = (0:n)*dt;
X = zeros(6, n+1);
X(:, 1) = x0(:);
f = @(tt, x) orbit.derivative(tt, x, jd0, eop, degree);
for k = 1:n
    X(:, k+1) = num.rk4Step(f, t(k), X(:, k), dt);
end
end
