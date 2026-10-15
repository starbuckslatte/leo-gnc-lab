function r = multiply(q, p)
%ATT.MULTIPLY Quaternion product r = q (x) p, Shuster convention.
%   Defined so that attitude matrices compose in the same order:
%       att.toDCM(att.multiply(q, p)) == att.toDCM(q) * att.toDCM(p)
%   i.e. p is applied first, then q.
%
%   NOTE: This differs from the Hamilton product used by quatmultiply
%   (Aerospace Toolbox). See docs/conventions.md.
q = q(:); p = p(:);
qv = q(1:3); q4 = q(4);
pv = p(1:3); p4 = p(4);
r = [q4*pv + p4*qv - cross(qv, pv);
     q4*p4 - dot(qv, pv)];
end
