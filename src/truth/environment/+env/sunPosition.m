function rSun = sunPosition(jdUtc)
%ENV.SUNPOSITION Geocentric Sun position [m] in ECI (J2000/ICRF).
%   JPL DE405 via Aerospace Toolbox planetEphemeris, evaluated at TT
%   (TT - TDB < 2 ms, negligible). ICRF and J2000 differ by ~20 mas.
rSun = planetEphemeris(timesys.utcToTT(jdUtc), 'Earth', 'Sun', '405', 'km')' * 1e3;
end
