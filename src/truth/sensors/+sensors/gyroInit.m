function state = gyroInit(p)
%SENSORS.GYROINIT Initial gyro state: bias and delay buffer.
%   p fields: initialBias [rad/s], delaySteps (optional, default 0).
state.bias = p.initialBias(:);
n = 0;
if isfield(p, 'delaySteps'), n = p.delaySteps; end
state.buf = repmat(zeros(3, 1), 1, n);
end
