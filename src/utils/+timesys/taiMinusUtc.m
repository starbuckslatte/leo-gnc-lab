function dat = taiMinusUtc(jdUtc)
%TIMESYS.TAIMINUSUTC TAI - UTC [s] (leap seconds) at UTC Julian date, from 1972.
%   Uses MATLAB's built-in leapseconds table. Each leap second is inserted
%   at the end of the listed day, so it takes effect at 00:00 the next day.
%   The table is converted to Julian dates once and cached (hot path of
%   every frame transformation).
persistent jdEffective
if isempty(jdEffective)
    tt = leapseconds();
    jdEffective = juliandate(tt.Date + days(1));
end
dat = 10 + sum(jdEffective <= jdUtc);
end
