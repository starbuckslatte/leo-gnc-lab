function q = fromScalarFirst(qsf)
%ATT.FROMSCALARFIRST Convert from Aerospace Toolbox layout [q0 q1 q2 q3].
qsf = qsf(:);
q = [qsf(2:4); qsf(1)];
end
