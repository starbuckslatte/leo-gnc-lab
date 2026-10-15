function dat = taiMinusUtc(jdUtc)
%TIMESYS.TAIMINUSUTC TAI - UTC [s] (leap seconds) at UTC Julian date, from 1972.
%   Uses MATLAB's built-in leapseconds table. Each leap second is inserted
%   at the end of the listed day, so it takes effect at 00:00 the next day.
tt = leapseconds();
t = datetime(jdUtc, 'ConvertFrom', 'juliandate');
dat = 10 + sum(tt.Date + days(1) <= t);
end
