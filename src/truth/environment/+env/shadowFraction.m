function nu = shadowFraction(rSc, rSun)
%ENV.SHADOWFRACTION Fraction of the Sun disk visible from the spacecraft.
%   1 = full sunlight, 0 = umbra, in between = penumbra. Conical Earth
%   shadow with apparent-disk overlap (Montenbruck & Gill, Sec. 3.4.2).
%   rSc, rSun: geocentric positions [m], same frame.
R_SUN   = 696000e3;
R_EARTH = 6378137;
s = rSun(:) - rSc(:);                       % spacecraft -> Sun
a = asin(R_SUN / norm(s));                  % apparent Sun radius
b = asin(R_EARTH / norm(rSc));              % apparent Earth radius
c = acos(max(-1, min(1, dot(-rSc(:), s) / (norm(rSc)*norm(s)))));  % separation
if c >= a + b
    nu = 1;                                 % no occultation
elseif c <= b - a
    nu = 0;                                 % total (umbra)
elseif c <= a - b
    nu = 1 - b^2/a^2;                       % annular (not possible in LEO)
else
    x = (c^2 + a^2 - b^2) / (2*c);
    y = sqrt(max(a^2 - x^2, 0));
    A = a^2*acos(x/a) + b^2*acos((c - x)/b) - c*y;
    nu = 1 - A/(pi*a^2);
end
end
