function [axis, angle] = toAxisAngle(q)
%ATT.TOAXISANGLE Rotation axis (unit) and angle [rad] in [0, pi].
q = att.normalize(q(:));
if q(1) < 0
    q = -q;
end
s = norm(q(2:4));
angle = 2*atan2(s, q(1));
if s < eps
    axis = [1; 0; 0];   % arbitrary for a zero rotation
else
    axis = q(2:4) / s;
end
end
