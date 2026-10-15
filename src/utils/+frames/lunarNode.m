function om = lunarNode(jcTT)
%FRAMES.LUNARNODE Mean longitude of the Moon's ascending node [rad], IAU 1980.
%   jcTT: Julian centuries of TT since J2000.
DAS2R = pi/180/3600;
t = jcTT;
om = (450160.280 + (-482890.539 + (7.455 + 0.008*t)*t)*t)*DAS2R + mod(-5*t, 1)*2*pi;
om = mod(om + pi, 2*pi) - pi;
end
