function raan = raanFromLtan(jdUtc, ltan)
%ORBIT.RAANFROMLTAN Right ascension of the ascending node [rad] for a given
%   local time of ascending node LTAN [h] at UTC Julian date jdUtc.
%   Uses the Sun's right ascension (mean solar time, equation of time ignored).
rSun = env.sunPosition(jdUtc);
raan = mod(atan2(rSun(2), rSun(1)) + (ltan - 12)*pi/12, 2*pi);
end
