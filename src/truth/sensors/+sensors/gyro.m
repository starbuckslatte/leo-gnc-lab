function [wMeas, state] = gyro(wTrue, state, p, dt)
%SENSORS.GYRO Three-axis rate gyro, Farrenkopf model (Crassidis & Junkins 4.7).
%   wMeas = w + 0.5*(b_k + b_{k+1}) + sqrt(sigmaV^2/dt + sigmaU^2*dt/12) * Nv
%   b_{k+1} = b_k + sigmaU*sqrt(dt) * Nu
%   wTrue: true body rate [rad/s]; dt: sample interval [s].
%   p fields: sigmaV [rad/s^0.5] (ARW), sigmaU [rad/s^1.5] (RRW),
%   quantization [rad/s] (optional, 0 = off), delaySteps via sensors.gyroInit.
%   state: from sensors.gyroInit.
bOld = state.bias;
state.bias = bOld + p.sigmaU*sqrt(dt)*randn(3, 1);
wMeas = wTrue(:) + 0.5*(bOld + state.bias) ...
        + sqrt(p.sigmaV^2/dt + p.sigmaU^2*dt/12)*randn(3, 1);
if isfield(p, 'quantization')
    wMeas = sensors.quantize(wMeas, p.quantization);
end
[wMeas, state.buf] = sensors.delay(wMeas, state.buf);
end
