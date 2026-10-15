function A = toDCM(q)
%ATT.TODCM Attitude matrix (DCM) from quaternion, scalar-first.
%   A maps vectors from the reference (e.g. ECI) frame to the body frame:
%       v_body = A * v_ref        (passive / frame rotation)
%   Same as EGSE FrameA2B / CoordEciToBody and Aerospace Toolbox quat2dcm.
%   A(q) = (qs^2 - |qv|^2) I + 2 qv qv' - 2 qs [qv x]
q = q(:);
qs = q(1); qv = q(2:4);
A = (qs^2 - qv'*qv)*eye(3) + 2*(qv*qv') - 2*qs*att.skew(qv);
end
