function M = rot1(a)
%FRAMES.ROT1 Frame rotation of angle a [rad] about x (passive, Vallado ROT1).
M = [1 0 0; 0 cos(a) sin(a); 0 -sin(a) cos(a)];
end
