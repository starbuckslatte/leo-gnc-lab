function q = propagate(q, w, dt)
%ATT.PROPAGATE Discrete quaternion propagation, exact for constant w over dt.
%   q(k+1) = q(k) (x) dq,   dq = [cos(|w|dt/2); sin(|w|dt/2) * w/|w|]
%   w: body rate w.r.t. reference, in body [rad/s]. Scalar-first, Hamilton.
q = q(:); w = w(:);
wn = norm(w);
c  = cos(0.5*wn*dt);
if wn > 1e-12
    psi = sin(0.5*wn*dt) / wn * w;
else
    psi = 0.5*dt*w;     % small-angle limit
end
q = att.normalize(att.multiply(q, [c; psi]));
end
