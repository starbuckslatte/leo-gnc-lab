function [fBody, tauBody, thrust] = thruster(cmd, p, dt)
%ACTUATORS.THRUSTER Force [N] and torque [N m] in body frame from thrusters.
%   cmd: commanded thrust per thruster [N], averaged over control period dt.
%   Each thruster only pushes (0 <= thrust <= maxThrust). On/off valves
%   realise the average by an on-time; with minOnTime > 0 the on-time is
%   rounded to a multiple of minOnTime (minimum impulse bit).
%   p fields: positions (3xn, m, from CoM), directions (3xn, unit force
%   direction on the spacecraft), maxThrust [N]; optional minOnTime [s].
%   Returns per-thruster average thrust as well.
thrust = max(0, min(p.maxThrust, cmd(:)));
if isfield(p, 'minOnTime') && p.minOnTime > 0
    onTime = sensors.quantize(thrust/p.maxThrust*dt, p.minOnTime);
    thrust = min(onTime, dt)/dt * p.maxThrust;
end
F = p.directions .* thrust';
fBody = sum(F, 2);
tauBody = sum(cross(p.positions, F), 2);
end
