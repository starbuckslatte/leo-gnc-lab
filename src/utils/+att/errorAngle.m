function ang = errorAngle(qA, qB)
%ATT.ERRORANGLE Total rotation angle [rad] between two attitudes.
%   Uses dq = qA (x) qB^-1; insensitive to the q / -q sign ambiguity.
dq  = att.multiply(qA, att.conj(qB));
ang = 2*atan2(norm(dq(2:4)), abs(dq(1)));
end
