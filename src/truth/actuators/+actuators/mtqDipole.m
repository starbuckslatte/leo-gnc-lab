function m = mtqDipole(cmd, p)
%ACTUATORS.MTQDIPOLE Magnetorquer dipole actually produced [A m^2], body frame.
%   Each axis is an independent coil, so each is limited to +/- maxDipole
%   separately (this can change the direction of a large command).
%   p fields: maxDipole [A m^2]; optional quantization [A m^2] (PWM/DAC step).
%   Torque on the spacecraft: torque.magnetic(bEci, q, m).
m = max(-p.maxDipole, min(p.maxDipole, cmd(:)));
if isfield(p, 'quantization')
    m = sensors.quantize(m, p.quantization);
end
end
