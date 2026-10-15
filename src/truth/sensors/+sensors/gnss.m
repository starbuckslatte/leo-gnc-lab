function [meas, state] = gnss(rEci, vEci, jdUtc, eop, state, p)
%SENSORS.GNSS GNSS receiver navigation solution in ECEF.
%   Receivers report position/velocity in ECEF (ITRF) with a GPS time tag,
%   so the flight software must convert to ECI itself.
%   meas fields: rEcef [m], vEcef [m/s], week, sow [s] (GPS time of the fix).
%   With delaySteps > 0 the whole fix, including its time tag, is from an
%   earlier sample.
%   p fields: sigmaPos [m], sigmaVel [m/s] per axis; delaySteps via sensors.gnssInit.
[r, v] = frames.eciToEcef(rEci, vEci, jdUtc, eop);
[week, sow] = timesys.utcToGps(jdUtc);
y = [r + p.sigmaPos*randn(3, 1); v + p.sigmaVel*randn(3, 1); week; sow];
[y, state.buf] = sensors.delay(y, state.buf);
meas = struct('rEcef', y(1:3), 'vEcef', y(4:6), 'week', y(7), 'sow', y(8));
end
