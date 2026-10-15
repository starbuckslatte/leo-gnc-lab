function bEci = magneticField(rEci, jdUtc, eop)
%ENV.MAGNETICFIELD Geomagnetic field [T] in ECI (J2000), IGRF-14.
%   Aerospace Toolbox igrfmagm at the WGS84 geodetic position of rEci [m].
%   eop: see frames.fk5.
T = frames.fk5(jdUtc, eop);
M = T.P * T.N * T.R * T.W;                  % ECEF -> ECI
lla = ecef2lla((M' * rEci(:))');
lat = deg2rad(lla(1)); lon = deg2rad(lla(2));
dt = datetime(jdUtc, 'ConvertFrom', 'juliandate');
bNed = igrfmagm(lla(3), lla(1), lla(2), decyear(dt), 14)' * 1e-9;
% NED basis vectors expressed in ECEF (columns)
C = [-sin(lat)*cos(lon), -sin(lon), -cos(lat)*cos(lon);
     -sin(lat)*sin(lon),  cos(lon), -cos(lat)*sin(lon);
      cos(lat),           0,        -sin(lat)];
bEci = M * C * bNed;
end
