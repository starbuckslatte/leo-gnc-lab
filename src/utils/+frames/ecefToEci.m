function [rEci, vEci] = ecefToEci(rEcef, vEcef, jdUtc, eop)
%FRAMES.ECEFTOECI ITRF -> J2000 position/velocity, IAU-76/FK5.
%   Units follow the input (m or km). eop: see frames.fk5.
T = frames.fk5(jdUtc, eop);
w = [0; 0; T.omegaEarth];
rPef = T.W * rEcef(:);
vPef = T.W * vEcef(:);
PNR  = T.P * T.N * T.R;
rEci = PNR * rPef;
vEci = PNR * (vPef + cross(w, rPef));
end
