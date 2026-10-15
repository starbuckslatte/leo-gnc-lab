function qsf = toScalarFirst(q)
%ATT.TOSCALARFIRST Convert to Aerospace Toolbox layout: 1x4 row [q4 q1 q2 q3].
%   Only reorders components. Both conventions give the same DCM, but
%   quaternion MULTIPLICATION order differs; see docs/conventions.md.
q = q(:);
qsf = [q(4), q(1:3)'];
end
