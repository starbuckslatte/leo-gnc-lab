function M = rot2(a)
%FRAMES.ROT2 Frame rotation of angle a [rad] about y (passive, Vallado ROT2).
M = [cos(a) 0 -sin(a); 0 1 0; sin(a) 0 cos(a)];
end
