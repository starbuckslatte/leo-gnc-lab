function fsw = estimatorFsw()
%MEKF.ESTIMATORFSW Flight software that runs the MEKF and commands nothing.
%   Uses only measurements and nominal parameters (never the truth):
%   - initialises on the first valid star-tracker sample (beta = 0, P0 from
%     m.fsw.mekf.P0)
%   - propagates with each new gyro sample, using the average of the
%     previous and current samples over the interval between them
%     (trapezoid; the first sample alone would leave a 0.5*wdot*dt error
%     that the filter mistakes for bias)
%   - updates with each new valid star-tracker sample
%   fsw.log(state) returns [q; beta; sqrt(diag(P))] (13x1, NaN before init)
%   for sim.run to record.
fsw.init = @init;
fsw.step = @step;
fsw.log  = @logState;
end

function s = init(m)
s.p = m;
s.f = [];
s.wPrev = []; s.tPrev = NaN;
s.R = mekf.starTrackerCovariance(m.sensors.starTracker);
end

function [cmd, s] = step(s, meas, ~)
cmd = struct('rw', [], 'mtq', [], 'thruster', []);
g = s.p.sensors.gyro;
if isempty(s.f)
    if meas.str.new && meas.str.valid
        s.f = mekf.init(meas.str.q, zeros(3, 1), s.p.fsw.mekf.P0);
        s.wPrev = meas.gyro.w; s.tPrev = meas.gyro.t;
    end
    return
end
if meas.gyro.new
    dt = meas.gyro.t - s.tPrev;
    s.f = mekf.propagate(s.f, 0.5*(s.wPrev + meas.gyro.w), dt, g.sigmaV, g.sigmaU);
    s.wPrev = meas.gyro.w; s.tPrev = meas.gyro.t;
end
if meas.str.new && meas.str.valid
    s.f = mekf.updateAttitude(s.f, meas.str.q, s.R);
end
end

function y = logState(s)
if isempty(s.f)
    y = nan(13, 1);
else
    y = [s.f.q; s.f.beta; sqrt(diag(s.f.P))];
end
end
