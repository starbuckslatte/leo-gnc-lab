function T = fk5(jdUtc, eop)
%FRAMES.FK5 IAU-76/FK5 reduction matrices at UTC Julian date jdUtc.
%   r_ECI = P * N * R * W * r_ECEF   (ECI = J2000 mean equator/equinox)
%   eop fields: dut1 [s], xp, yp [rad], lod [s]. Missing fields default to 0.
%   Same composition as B5G-AOCS-EGSE ecef2eci_76fk5 (Vallado Sec. 3.7).
%   Returns struct with P, N, R, W, gast [rad], omegaEarth [rad/s].
eop  = fillEop(eop);
jcTT = timesys.century(timesys.utcToTT(jdUtc));
jdUt1 = timesys.utcToUT1(jdUtc, eop.dut1);

[zeta, z, theta] = precession(jcTT);
T.P = frames.rot3(zeta) * frames.rot2(-theta) * frames.rot3(z);

eps0 = meanObliquity(jcTT);
[dpsi, deps] = frames.nut80(jcTT);
T.N = frames.rot1(-eps0) * frames.rot3(dpsi) * frames.rot1(eps0 + deps);

% Equation of the equinoxes with IAU-1994 terms (Vallado Eq. 3-79)
om = frames.lunarNode(jcTT);
DAS2R = pi/180/3600;
T.gast = timesys.gmst(jdUt1) + dpsi*cos(eps0) ...
       + 0.00264*DAS2R*sin(om) + 0.000063*DAS2R*sin(2*om);
T.R = frames.rot3(-T.gast);

T.W = frames.rot1(eop.yp) * frames.rot2(eop.xp);
T.omegaEarth = 7.292115146706979e-5 * (1 - eop.lod/86400);
end

function [zeta, z, theta] = precession(t)
DAS2R = pi/180/3600;
zeta  = (2306.2181*t + 0.30188*t^2 + 0.017998*t^3) * DAS2R;
z     = (2306.2181*t + 1.09468*t^2 + 0.018203*t^3) * DAS2R;
theta = (2004.3109*t - 0.42665*t^2 - 0.041833*t^3) * DAS2R;
end

function eps0 = meanObliquity(t)
eps0 = (84381.448 - 46.8150*t - 0.00059*t^2 + 0.001813*t^3) * pi/180/3600;
end

function eop = fillEop(eop)
for f = ["dut1", "xp", "yp", "lod"]
    if ~isfield(eop, f), eop.(f) = 0; end
end
end
