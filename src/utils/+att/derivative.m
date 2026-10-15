function qdot = derivative(q, w)
%ATT.DERIVATIVE Quaternion kinematics: qdot = 0.5 * Omega(w) * q.
%   w: angular velocity of body w.r.t. reference, expressed in body [rad/s].
%   Omega(w) = [ -[w x]  w ;
%                 -w'    0 ]
q = q(:); w = w(:);
qv = q(1:3); q4 = q(4);
qdot = 0.5*[-att.skew(w)*qv + q4*w;
            -w'*qv];
end
