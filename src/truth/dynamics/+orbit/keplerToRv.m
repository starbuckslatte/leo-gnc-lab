function [r, v] = keplerToRv(a, e, inc, raan, argp, nu)
%ORBIT.KEPLERTORV Position [m] and velocity [m/s] in ECI from classical elements.
%   a [m], e, inc, raan, argp, nu (true anomaly) [rad]; two-body, EGM2008 GM.
MU = 3.986004415e14;
p = a*(1 - e^2);
rPf = p/(1 + e*cos(nu)) * [cos(nu); sin(nu); 0];
vPf = sqrt(MU/p) * [-sin(nu); e + cos(nu); 0];
R3 = @(x) [cos(x) -sin(x) 0; sin(x) cos(x) 0; 0 0 1];
R1 = @(x) [1 0 0; 0 cos(x) -sin(x); 0 sin(x) cos(x)];
C = R3(raan) * R1(inc) * R3(argp);              % perifocal -> ECI
r = C * rPf;
v = C * vPf;
end
