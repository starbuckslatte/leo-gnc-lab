function state = starTrackerInit(p)
%SENSORS.STARTRACKERINIT Initial star tracker state (delay buffer).
%   Delayed outputs start as invalid identity quaternions.
n = 0;
if isfield(p, 'delaySteps'), n = p.delaySteps; end
state.buf = repmat([1; 0; 0; 0; 0], 1, n);
end
