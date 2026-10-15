function qc = conj(q)
%ATT.CONJ Quaternion conjugate (= inverse for a unit quaternion).
%   qc = att.conj(q) for a 4xN array, scalar-last.
qc = [-q(1:3, :); q(4, :)];
end
