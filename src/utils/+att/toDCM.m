function A = toDCM(q)
%ATT.TODCM Attitude matrix (DCM) from quaternion, scalar-last.
%   A maps vectors from the reference (e.g. ECI) frame to the body frame:
%       v_body = A * v_ref        (passive / frame rotation)
%   A(q) = (q4^2 - |qv|^2) I + 2 qv qv' - 2 q4 [qv x]
q = q(:);
qv = q(1:3); q4 = q(4);
A = (q4^2 - qv'*qv)*eye(3) + 2*(qv*qv') - 2*q4*att.skew(qv);
end
