function qc = conj(q)
%ATT.CONJ Quaternion conjugate (= inverse for a unit quaternion).
%   qc = att.conj(q) for a 4xN array, scalar-first. Same as EGSE QInv.
qc = [q(1, :); -q(2:4, :)];
end
