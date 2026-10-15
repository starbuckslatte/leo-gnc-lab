function q = normalize(q)
%ATT.NORMALIZE Normalize quaternion(s) to unit length.
%   q = att.normalize(q) normalizes each column of a 4xN array.
n = vecnorm(q, 2, 1);
if any(n < eps)
    error('att:normalize:zeroNorm', 'Cannot normalize a zero-norm quaternion.');
end
q = q ./ n;
end
