function xdot = derivative(x, J, tau, G, hDot)
%ATTITUDE.DERIVATIVE Rigid-body attitude state derivative, optional wheels.
%   x = [q; w]: q ECI->body quaternion (scalar-first), w body rate [rad/s]
%   w.r.t. ECI in body. J inertia [kg m^2] (whole spacecraft incl. wheels),
%   tau external torque [N m] (body).
%       qdot = 0.5 q (x) [0; w]
%       wdot = J \ (tau - w x J w)       (Euler's equations)
%   With reaction wheels, x = [q; w; h] (h axial wheel momenta), G spin axes
%   (3-by-n) and hDot the wheel momentum rates (actuators.rwTorque):
%       wdot = J \ (tau - G hDot - w x (J w + G h))
q = x(1:4); w = x(5:7);
if nargin < 4
    xdot = [att.derivative(q, w);
            J \ (tau(:) - cross(w, J*w))];
    return
end
h = x(8:end);
xdot = [att.derivative(q, w);
        J \ (tau(:) - G*hDot - cross(w, J*w + G*h));
        hDot];
end
