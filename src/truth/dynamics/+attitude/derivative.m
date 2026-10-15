function xdot = derivative(x, J, tau)
%ATTITUDE.DERIVATIVE Rigid-body attitude state derivative.
%   x = [q; w]: q ECI->body quaternion (scalar-first), w body rate [rad/s]
%   w.r.t. ECI in body. J inertia [kg m^2], tau external torque [N m] (body).
%       qdot = 0.5 q (x) [0; w]
%       wdot = J \ (tau - w x J w)       (Euler's equations)
q = x(1:4); w = x(5:7);
xdot = [att.derivative(q, w);
        J \ (tau(:) - cross(w, J*w))];
end
