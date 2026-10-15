function state = magnetometerInit(p)
%SENSORS.MAGNETOMETERINIT Initial magnetometer state (delay buffer).
n = 0;
if isfield(p, 'delaySteps'), n = p.delaySteps; end
state.buf = repmat(zeros(3, 1), 1, n);
end
