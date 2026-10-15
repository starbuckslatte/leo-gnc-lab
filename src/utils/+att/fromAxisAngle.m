function q = fromAxisAngle(axis, angle)
%ATT.FROMAXISANGLE Quaternion for a frame rotation of ANGLE [rad] about AXIS.
%   q = [cos(angle/2); e*sin(angle/2)], e = axis/|axis|
axis = axis(:);
nrm = norm(axis);
if nrm < eps
    error('att:fromAxisAngle:zeroAxis', 'Rotation axis must be non-zero.');
end
e = axis / nrm;
q = [cos(angle/2); e*sin(angle/2)];
end
