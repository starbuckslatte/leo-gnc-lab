function xdot = derivative(t, x, jd0, eop, degree)
%ORBIT.DERIVATIVE State derivative [v; a] for ECI state x = [r; v] (SI).
%   t: seconds since epoch jd0 (UTC Julian date).
xdot = [x(4:6);
        orbit.gravityAccel(x(1:3), jd0 + t/86400, eop, degree)];
end
