function jdTT = utcToTT(jdUtc)
%TIMESYS.UTCTOTT TT Julian date from UTC Julian date. TT = TAI + 32.184 s.
jdTT = jdUtc + (timesys.taiMinusUtc(jdUtc) + 32.184)/86400;
end
