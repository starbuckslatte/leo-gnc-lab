classdef TimeSystemTest < matlab.unittest.TestCase
    %TIMESYSTEMTEST Unit tests for the +timesys package.
    %   Reference values: Vallado, Fundamentals of Astrodynamics, 4th ed.

    methods (TestClassSetup)
        function addSourcePath(testCase)
            root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(root, 'src', 'utils')));
        end
    end

    methods (Test)
        function julianDateVallado(testCase)
            % Vallado Ex. 3-4. Every frame/ephemeris computation starts here.
            testCase.verifyEqual(timesys.jd([1996 10 26 14 20 0]), 2450383.09722222, ...
                'AbsTol', 1e-8);
        end

        function j2000EpochIsZeroCentury(testCase)
            testCase.verifyEqual(timesys.century(timesys.jd([2000 1 1 12 0 0])), 0, ...
                'AbsTol', 1e-15);
        end

        function gmstVallado(testCase)
            % Vallado Ex. 3-5. 1e-6 deg ~ 0.7 m at GEO radius; the residual
            % 7.6e-8 deg comes from the book's rounded JD.
            g = rad2deg(timesys.gmst(timesys.jd([1992 8 20 12 14 0])));
            testCase.verifyEqual(g, 152.578787886, 'AbsTol', 1e-6);
        end

        function leapSecondTakesEffectNextDay(testCase)
            % The 2016-12-31 leap second applies from 2017-01-01 00:00 UTC.
            % An off-by-one-day error here shifts TT/GPS by 1 s for a whole day.
            testCase.verifyEqual(timesys.taiMinusUtc(timesys.jd([2016 12 31 23 59 59])), 36);
            testCase.verifyEqual(timesys.taiMinusUtc(timesys.jd([2017 1 1 0 0 0])), 37);
        end

        function ttOffset(testCase)
            % TT - UTC = 37 + 32.184 s after 2017. Tolerance 1e-4 s: a double
            % JD (~2.46e6) resolves only ~40 us, the floor of all JD-based time.
            jd = timesys.jd([2026 1 1 0 0 0]);
            testCase.verifyEqual((timesys.utcToTT(jd) - jd)*86400, 69.184, 'AbsTol', 1e-4);
        end

        function gpsEpochIsWeekZero(testCase)
            [w, s] = timesys.utcToGps(timesys.jd([1980 1 6 0 0 0]));
            testCase.verifyEqual([w, s], [0, 0], 'AbsTol', 1e-5);
        end

        function gpsWeekWithLeapSeconds(testCase)
            % 2017-01-01 is the start of GPS week 1930; GPS - UTC = 18 s.
            % GPS receivers report full weeks; FSW must not apply mod 1024 here.
            [w, s] = timesys.utcToGps(timesys.jd([2017 1 1 0 0 0]));
            testCase.verifyEqual(w, 1930);
            testCase.verifyEqual(s, 18, 'AbsTol', 1e-5);
        end
    end
end
