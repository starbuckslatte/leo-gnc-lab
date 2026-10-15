function ang = errorAngle(qA, qB)
%ATT.ERRORANGLE Total rotation angle [rad] between two attitudes.
%   Uses dq = qA (x) qB^-1; insensitive to the q / -q sign ambiguity.
dq  = att.multiply(qA, att.conj(qB));
ang = 2*atan2(norm(dq(1:3)), abs(dq(4)));
end
