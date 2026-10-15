function [qMeas, valid, state] = starTracker(qTrue, rEci, rSun, state, p)
%SENSORS.STARTRACKER Star tracker attitude measurement with exclusion zones.
%   qMeas = qTrue (x) dq, dq a small rotation with 1-sigma noise sigmaCross
%   about the two axes normal to the boresight and sigmaRoll about it.
%   valid = false if the boresight is within sunExcl of the Sun or within
%   earthExcl of the Earth limb.
%   qTrue: ECI->body (scalar-first); rEci, rSun: geocentric positions [m].
%   p fields: sigmaCross, sigmaRoll [rad], boresight (body unit vector),
%   sunExcl, earthExcl [rad]; delaySteps via sensors.starTrackerInit.
RE = 6378137;
b = p.boresight(:) / norm(p.boresight);
A = att.toDCM(qTrue);
% Noise in a sensor frame whose z axis is the boresight
x = cross(b, [1; 0; 0]);
if norm(x) < 0.1, x = cross(b, [0; 1; 0]); end
x = x / norm(x);
S = [x, cross(b, x), b];                        % sensor -> body
dth = S * [p.sigmaCross*randn(2, 1); p.sigmaRoll*randn];
if norm(dth) > 0
    qMeas = att.multiply(qTrue, att.fromAxisAngle(dth, norm(dth)));
else
    qMeas = qTrue(:);
end
% Exclusion zones (directions in body frame)
sunB = A * (rSun(:) - rEci(:));
nadirB = -A * rEci(:);
earthRadius = asin(RE / norm(rEci));
valid = acos(dot(b, sunB)/norm(sunB)) > p.sunExcl && ...
        acos(dot(b, nadirB)/norm(nadirB)) > earthRadius + p.earthExcl;
[y, state.buf] = sensors.delay([qMeas; valid], state.buf);
qMeas = y(1:4);
valid = y(5) > 0.5;
end
