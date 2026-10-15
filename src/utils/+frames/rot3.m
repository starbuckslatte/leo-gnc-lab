function M = rot3(a)
%FRAMES.ROT3 Frame rotation of angle a [rad] about z (passive, Vallado ROT3).
M = [cos(a) sin(a) 0; -sin(a) cos(a) 0; 0 0 1];
end
