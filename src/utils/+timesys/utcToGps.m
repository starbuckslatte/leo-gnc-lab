function [week, sow] = utcToGps(jdUtc)
%TIMESYS.UTCTOGPS GPS week (no 1024 rollover) and seconds of week.
%   GPS = TAI - 19 s; GPS epoch 1980-01-06 00:00:00 UTC (JD 2444244.5).
gpsSec = (jdUtc - 2444244.5)*86400 + timesys.taiMinusUtc(jdUtc) - 19;
week = floor(gpsSec / 604800);
sow  = gpsSec - week*604800;
end
