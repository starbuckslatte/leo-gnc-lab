function f = init(q0, beta0, P0)
%MEKF.INIT Multiplicative EKF state.
%   q0: initial ECI->body quaternion estimate (scalar-first)
%   beta0: initial gyro bias estimate [rad/s]
%   P0: 6x6 covariance of the error state [dtheta; dbeta] (body frame, rad, rad/s)
%   Error convention: q_true = q_hat (x) dq(dtheta).
f.q = att.normalize(q0(:));
f.beta = beta0(:);
f.P = P0;
end
