function [y, buf] = delay(x, buf)
%SENSORS.DELAY Fixed delay of N samples (hardware effect).
%   buf: n-by-N matrix of past samples, oldest first; empty = no delay.
%   Create with repmat(x0, 1, N). Returns the sample from N calls ago.
if isempty(buf)
    y = x;
    return
end
y = buf(:, 1);
buf = [buf(:, 2:end), x(:)];
end
