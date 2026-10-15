function G = rwConfig(p)
%ACTUATORS.RWCONFIG Wheel spin-axis matrix G (3-by-n, unit columns, body frame).
%   Total wheel momentum in body = G * h, h = axial momentum of each wheel.
%   p.config: 'pyramid' (4 wheels at azimuth 45/135/225/315 deg, each tilted
%   p.cantAngle from body +z) or 'orthogonal' (3 wheels along x, y, z).
switch p.config
    case 'pyramid'
        az = deg2rad([45 135 225 315]);
        b = p.cantAngle;
        G = [sin(b)*cos(az); sin(b)*sin(az); cos(b)*ones(1, 4)];
    case 'orthogonal'
        G = eye(3);
    otherwise
        error('actuators:rwConfig:unknown', 'Unknown wheel configuration "%s".', p.config);
end
end
