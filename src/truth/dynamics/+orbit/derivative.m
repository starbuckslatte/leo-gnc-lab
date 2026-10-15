function xdot = derivative(t, x, jd0, eop, degree, aExt)
%ORBIT.DERIVATIVE State derivative [v; a] for ECI state x = [r; v] (SI).
%   t: seconds since epoch jd0 (UTC Julian date).
%   aExt (optional): additional ECI acceleration [m/s^2], e.g. thrust.
a = orbit.gravityAccel(x(1:3), jd0 + t/86400, eop, degree);
if nargin > 5
    a = a + aExt(:);
end
xdot = [x(4:6); a];
end
