function state = sunSensorInit(p)
%SENSORS.SUNSENSORINIT Initial sun sensor state (delay buffer; delayed outputs invalid).
n = 0;
if isfield(p, 'delaySteps'), n = p.delaySteps; end
state.buf = repmat(zeros(4, 1), 1, n);
end
