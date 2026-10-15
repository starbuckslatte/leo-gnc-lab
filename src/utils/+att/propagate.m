function q = propagate(q, w, dt)
%ATT.PROPAGATE Discrete quaternion propagation, exact for constant w over dt.
%   q(k+1) = OmegaBar(w, dt) * q(k)
%   OmegaBar = [ cos(|w|dt/2) I - [psi x]   psi ;
%                -psi'                      cos(|w|dt/2) ]
%   psi = sin(|w|dt/2) * w/|w|
%   This is the propagation step used in the MEKF (Crassidis & Junkins).
q = q(:); w = w(:);
wn = norm(w);
c  = cos(0.5*wn*dt);
if wn > 1e-12
    psi = sin(0.5*wn*dt) / wn * w;
else
    psi = 0.5*dt*w;     % small-angle limit
end
OmegaBar = [c*eye(3) - att.skew(psi), psi;
            -psi',                    c  ];
q = att.normalize(OmegaBar*q);
end
