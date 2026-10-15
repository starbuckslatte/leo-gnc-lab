function aEci = gravityAccel(rEci, jdUtc, eop, degree)
%ORBIT.GRAVITYACCEL Earth gravity acceleration [m/s^2] in ECI (J2000).
%   rEci  : position [m], 3x1
%   degree: 0 = two-body (EGM2008 GM); >= 2 = EGM2008 spherical harmonics
%           to that degree and order (Aerospace Toolbox
%           gravitysphericalharmonic, evaluated in ECEF via frames.fk5).
MU = 3.986004415e14;    % m^3/s^2, EGM2008
r = rEci(:);
if degree == 0
    aEci = -MU * r / norm(r)^3;
    return
end
T = frames.fk5(jdUtc, eop);
M = T.P * T.N * T.R * T.W;          % ECEF -> ECI
rEcef = M' * r;
[gx, gy, gz] = gravitysphericalharmonic(rEcef', 'EGM2008', degree, 'None');
aEci = M * [gx; gy; gz];
end
