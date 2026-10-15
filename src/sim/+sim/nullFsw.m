function fsw = nullFsw()
%SIM.NULLFSW Flight software that commands nothing (open-loop runs).
%   Interface every FSW must provide:
%       state        = fsw.init(m)
%       [cmd, state] = fsw.step(state, meas, t)
%   cmd fields: rw (wheel motor torque, n x 1) [N m], mtq (dipole, 3 x 1)
%   [A m^2], thruster (thrust per thruster) [N]. An empty field keeps the
%   previous command. See sim.run for meas.
fsw.init = @(~) struct();
fsw.step = @step;
end

function [cmd, state] = step(state, ~, ~)
cmd = struct('rw', [], 'mtq', [], 'thruster', []);
end
