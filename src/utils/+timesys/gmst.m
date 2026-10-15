function theta = gmst(jdUt1)
%TIMESYS.GMST Greenwich mean sidereal time [rad] in [0, 2*pi), IAU-82.
%   Same formula as B5G-AOCS-EGSE iers_gmst96 (Vallado Eq. 3-47).
T = timesys.century(jdUt1);
sec = 67310.54841 + (876600*3600 + 8640184.812866)*T + 0.093104*T^2 - 6.2e-6*T^3;
theta = mod(sec, 86400) / 240 * pi/180;
end
