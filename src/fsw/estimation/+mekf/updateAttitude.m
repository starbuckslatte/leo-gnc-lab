function [f, innov] = updateAttitude(f, qMeas, R)
%MEKF.UPDATEATTITUDE Measurement update with a full attitude quaternion
%   (e.g. star tracker). Residual: dtheta_meas = 2 vec(q_hat^-1 (x) qMeas),
%   H = [I 0]. R: 3x3 measurement covariance in body frame [rad^2].
%   The estimated error is folded back into q_hat and beta (reset).
%   Returns the innovation (3x1, rad) for monitoring.
dq = att.multiply(att.conj(f.q), qMeas(:));
if dq(1) < 0, dq = -dq; end
innov = 2*dq(2:4);
H = [eye(3), zeros(3)];
S = H*f.P*H' + R;
K = f.P*H' / S;
dx = K*innov;
I6 = eye(6);
f.P = (I6 - K*H)*f.P*(I6 - K*H)' + K*R*K';         % Joseph form
f.P = (f.P + f.P')/2;
dth = dx(1:3);
f.q = att.normalize(att.multiply(f.q, [1; 0.5*dth]));
f.beta = f.beta + dx(4:6);
end
