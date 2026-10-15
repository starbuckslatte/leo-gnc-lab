function T = century(jd)
%TIMESYS.CENTURY Julian centuries since J2000.0 (JD 2451545.0).
T = (jd - 2451545.0) / 36525;
end
