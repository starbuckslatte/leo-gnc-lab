function tau = gravityGradient(rEci, q, J)
%TORQUE.GRAVITYGRADIENT Gravity-gradient torque [N m] in body frame.
%   tau = 3 mu / |r|^5 * (r_b x J r_b),  r_b = A(q) r
%   rEci: position [m]; q: ECI->body quaternion (scalar-first); J: inertia [kg m^2].
MU = 3.986004415e14;
rb = att.toDCM(q) * rEci(:);
tau = 3*MU / norm(rb)^5 * cross(rb, J*rb);
end
