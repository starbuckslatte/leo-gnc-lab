function rho = atmosphereDensity(rEci, jdUtc, eop, sw)
%ENV.ATMOSPHEREDENSITY Total mass density [kg/m^3], NRLMSISE-00.
%   Aerospace Toolbox atmosnrlmsise00 at the WGS84 geodetic position of
%   rEci [m]. sw: space weather with fields f107 (daily), f107a (81-day
%   average) and ap (daily). eop: see frames.fk5.
T = frames.fk5(jdUtc, eop);
M = T.P * T.N * T.R * T.W;                  % ECEF -> ECI
lla = ecef2lla((M' * rEci(:))');
dt = datetime(jdUtc, 'ConvertFrom', 'juliandate');
doy = day(dt, 'dayofyear');
utSec = seconds(timeofday(dt));
[~, d] = atmosnrlmsise00(lla(3), lla(1), lla(2), year(dt), doy, utSec, ...
                         sw.f107a, sw.f107, sw.ap);
rho = d(6);
end
