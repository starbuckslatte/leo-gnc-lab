function qdot = derivative(q, w)
%ATT.DERIVATIVE Quaternion kinematics: qdot = 0.5 * q (x) [0; w].
%   w: angular velocity of body w.r.t. reference, expressed in body [rad/s].
%   q: reference-to-body attitude, scalar-first.
q = q(:); w = w(:);
qs = q(1); qv = q(2:4);
qdot = 0.5*[-dot(qv, w);
            qs*w + cross(qv, w)];
end
