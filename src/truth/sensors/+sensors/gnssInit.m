function state = gnssInit(p)
%SENSORS.GNSSINIT Initial GNSS state (delay buffer).
n = 0;
if isfield(p, 'delaySteps'), n = p.delaySteps; end
state.buf = repmat(zeros(8, 1), 1, n);
end
