function hDot = rwTorque(cmd, h, p)
%ACTUATORS.RWTORQUE Rate of change of wheel axial momentum [N m] per wheel.
%   cmd: commanded motor torque per wheel [N m]; h: wheel momentum [N m s].
%   1. Motor torque limited to +/- maxTorque.
%   2. Momentum limit: no further torque in the direction of a wheel already
%      at maxMomentum (speed limit).
%   3. Friction (optional, default off): Coulomb + viscous on wheel speed
%      h / wheelInertia (body-rate contribution to wheel speed neglected).
%   p fields: maxTorque, maxMomentum; optional wheelInertia [kg m^2],
%   coulomb [N m], viscous [N m s].
tau = max(-p.maxTorque, min(p.maxTorque, cmd(:)));
atLimit = abs(h(:)) >= p.maxMomentum & sign(tau) == sign(h(:));
tau(atLimit) = 0;
hDot = tau;
if isfield(p, 'wheelInertia') && p.wheelInertia > 0
    ww = h(:) / p.wheelInertia;
    hDot = hDot - (p.coulomb*sign(ww) + p.viscous*ww);
end
end
