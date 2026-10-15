function R = starTrackerCovariance(p)
%MEKF.STARTRACKERCOVARIANCE Star tracker noise covariance in body frame [rad^2].
%   sigmaCross on the two axes normal to the boresight, sigmaRoll about it,
%   using the same sensor frame as sensors.starTracker.
b = p.boresight(:) / norm(p.boresight);
x = cross(b, [1; 0; 0]);
if norm(x) < 0.1, x = cross(b, [0; 1; 0]); end
x = x / norm(x);
S = [x, cross(b, x), b];
R = S * diag([p.sigmaCross^2, p.sigmaCross^2, p.sigmaRoll^2]) * S';
end
