function [A, q] = eciToLvlh(rEci, vEci)
%FRAMES.ECITOLVLH Attitude of LVLH w.r.t. ECI from position/velocity.
%   z = -r/|r| (nadir), y = -h/|h| (negative orbit normal), x = y cross z.
%   A maps ECI vectors into LVLH (v_lvlh = A*v_eci); q = att.fromDCM(A),
%   scalar-first. Same definition as B5G-AOCS-EGSE PVi2Qi2l.
r = rEci(:); v = vEci(:);
h = cross(r, v);
z = -r / norm(r);
y = -h / norm(h);
x = cross(y, z);
A = [x'; y'; z'];
q = att.fromDCM(A);
end
