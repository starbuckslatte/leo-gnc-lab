function jdUt1 = utcToUT1(jdUtc, dut1)
%TIMESYS.UTCTOUT1 UT1 Julian date from UTC Julian date. dut1 = UT1 - UTC [s].
jdUt1 = jdUtc + dut1/86400;
end
