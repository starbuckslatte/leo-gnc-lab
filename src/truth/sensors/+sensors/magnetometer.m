function [bMeas, state] = magnetometer(bEci, q, state, p, mtqDipole)
%SENSORS.MAGNETOMETER Three-axis magnetometer [T] in body frame.
%   bMeas = A(q) bEci + bias + mtqCoupling * mtqDipole + sigma * N
%   bEci: field [T] (env.magneticField); q: ECI->body (scalar-first).
%   mtqDipole (optional): magnetorquer dipole [A m^2] currently applied;
%   mtqCoupling [T/(A m^2)] models the MTQ field seen by the sensor, the
%   reason flight software blanks the magnetometer while the MTQs are on.
%   p fields: sigma [T]; optional bias [T], mtqCoupling (3x3),
%   quantization [T], delaySteps via sensors.magnetometerInit.
bMeas = att.toDCM(q) * bEci(:) + p.sigma*randn(3, 1);
if isfield(p, 'bias'), bMeas = bMeas + p.bias(:); end
if nargin > 4 && isfield(p, 'mtqCoupling')
    bMeas = bMeas + p.mtqCoupling * mtqDipole(:);
end
if isfield(p, 'quantization')
    bMeas = sensors.quantize(bMeas, p.quantization);
end
[bMeas, state.buf] = sensors.delay(bMeas, state.buf);
end
