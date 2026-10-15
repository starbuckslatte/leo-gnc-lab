function [tau, fBody] = solarRadiation(rEci, rSun, q, nu, sc)
%TORQUE.SOLARRADIATION Solar radiation pressure torque [N m] and force [N], body frame.
%   Cannonball-style flat plate facing the Sun:
%       F = -P (AU/d)^2 nu Cr A s_hat,   tau = cpOffset x F
%   s_hat: spacecraft -> Sun unit vector; nu: env.shadowFraction.
%   sc fields: srpArea [m^2], Cr, cpOffset [m] (CoP - CoM, body).
P_AU = 4.56e-6;             % N/m^2, solar pressure at 1 AU
AU   = 149597870700;        % m
s = rSun(:) - rEci(:);
d = norm(s);
sb = att.toDCM(q) * (s / d);
fBody = -P_AU * (AU/d)^2 * nu * sc.Cr * sc.srpArea * sb;
tau = cross(sc.cpOffset(:), fBody);
end
