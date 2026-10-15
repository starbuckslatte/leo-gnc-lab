function [sMeas, valid, state] = sunSensor(q, rEci, rSun, nu, state, p)
%SENSORS.SUNSENSOR Sun direction (unit vector) in body frame.
%   True direction s = A(q) (rSun - rEci)/|.|, perturbed by small-angle noise
%   of 1-sigma p.sigma [rad] on each of the two axes normal to s.
%   valid = false outside the field of view (half-angle p.fovHalf about
%   p.boresight) or in umbra (nu = 0, env.shadowFraction).
%   Earth albedo is not modelled.
%   p fields: sigma, fovHalf [rad], boresight (body); delaySteps via sensors.sunSensorInit.
s = att.toDCM(q) * (rSun(:) - rEci(:));
s = s / norm(s);
n = randn(3, 1);
n = n - dot(n, s)*s;                            % noise normal to s
sMeas = s + p.sigma*n;
sMeas = sMeas / norm(sMeas);
b = p.boresight(:) / norm(p.boresight);
valid = nu > 0 && acos(max(-1, min(1, dot(b, s)))) <= p.fovHalf;
[y, state.buf] = sensors.delay([sMeas; valid], state.buf);
sMeas = y(1:3);
valid = y(4) > 0.5;
end
