function jd = jd(dateVec)
%TIMESYS.JD Julian date from [year month day hour minute second].
%   Valid 1900-2100 (Vallado Alg. 14). The time scale of the result is the
%   time scale of the input. Same formula as B5G-AOCS-EGSE DateTime2JD.
Y = dateVec(1); M = dateVec(2); D = dateVec(3);
h = dateVec(4); m = dateVec(5); s = dateVec(6);
jd = 367*Y - fix(7*(Y + fix((M + 9)/12))/4) + fix(275*M/9) + D ...
     + 1721013.5 + ((s/60 + m)/60 + h)/24;
end
