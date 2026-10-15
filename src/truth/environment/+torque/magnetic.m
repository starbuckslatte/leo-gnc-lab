function tau = magnetic(bEci, q, dipole)
%TORQUE.MAGNETIC Residual magnetic dipole torque [N m] in body frame.
%   tau = m x B_b;  bEci [T] (env.magneticField), dipole [A m^2] in body.
tau = cross(dipole(:), att.toDCM(q) * bEci(:));
end
