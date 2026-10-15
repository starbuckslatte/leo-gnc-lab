function r = multiply(q, p)
%ATT.MULTIPLY Quaternion product r = q (x) p, Hamilton convention, scalar-first.
%   Same formula as EGSE QMul and Aerospace Toolbox quatmultiply.
%   Frame rotations compose left to right:
%       Qa2c = att.multiply(Qa2b, Qb2c)
%       att.toDCM(Qa2c) == att.toDCM(Qb2c) * att.toDCM(Qa2b)
%   See docs/conventions.md.
q = q(:); p = p(:);
qs = q(1); qv = q(2:4);
ps = p(1); pv = p(2:4);
r = [qs*ps - dot(qv, pv);
     qs*pv + ps*qv + cross(qv, pv)];
end
