function [axis, angle] = toAxisAngle(q)
%ATT.TOAXISANGLE Rotation axis (unit) and angle [rad] in [0, pi].
q = att.normalize(q(:));
if q(4) < 0
    q = -q;
end
s = norm(q(1:3));
angle = 2*atan2(s, q(4));
if s < eps
    axis = [1; 0; 0];   % arbitrary for a zero rotation
else
    axis = q(1:3) / s;
end
end
