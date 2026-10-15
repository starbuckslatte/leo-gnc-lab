function y = quantize(x, lsb)
%SENSORS.QUANTIZE Round to the nearest multiple of lsb (hardware effect).
%   lsb = 0 disables quantization.
if lsb > 0
    y = lsb * round(x / lsb);
else
    y = x;
end
end
