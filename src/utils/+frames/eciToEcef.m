function [rEcef, vEcef] = eciToEcef(rEci, vEci, jdUtc, eop)
%FRAMES.ECITOECEF J2000 -> ITRF position/velocity, IAU-76/FK5.
%   Inverse of frames.ecefToEci. Units follow the input (m or km).
T = frames.fk5(jdUtc, eop);
w = [0; 0; T.omegaEarth];
PNR  = T.P * T.N * T.R;
rPef = PNR' * rEci(:);
vPef = PNR' * vEci(:) - cross(w, rPef);
rEcef = T.W' * rPef;
vEcef = T.W' * vPef;
end
