classdef EnvironmentTest < matlab.unittest.TestCase
    %ENVIRONMENTTEST Unit tests for the +env package (Sun, shadow, IGRF, NRLMSISE-00).
    %   Each test checks a physical property that a frame, sign or unit
    %   error would break, not just the toolbox output itself.

    properties (Constant)
        Eop = struct()
        Sw  = struct('f107', 150, 'f107a', 150, 'ap', 4)
    end

    methods (TestClassSetup)
        function addSourcePath(testCase)
            testCase.assumeTrue(exist('igrfmagm', 'file') == 2, ...
                'Aerospace Toolbox not available; skipping.');
            root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(root, 'src'), 'IncludingSubfolders', true));
        end
    end

    methods (Static, Access = private)
        function r = eciAbove(latDeg, lonDeg, alt, jd)
            r = frames.ecefToEci(lla2ecef([latDeg lonDeg alt])', [0; 0; 0], jd, struct());
        end
    end

    methods (Test)
        function sunMatchesVallado(testCase)
            % Vallado Ex. 5-1 (2006-04-02, mean-of-date). The 0.08 deg
            % difference is precession from J2000 to 2006, so 0.15 deg
            % separates "right frame" from "wrong frame or wrong epoch".
            rs = env.sunPosition(timesys.jd([2006 4 2 0 0 0]));
            ref = [146186178; 28789146; 12481196] * 1e3;
            ang = acosd(dot(rs, ref) / (norm(rs)*norm(ref)));
            testCase.verifyLessThan(ang, 0.15);
            testCase.verifyEqual(norm(rs), norm(ref), 'RelTol', 1e-4);
        end

        function shadowLimits(testCase)
            rs = env.sunPosition(timesys.jd([2026 3 20 0 0 0]));
            u = rs / norm(rs);
            testCase.verifyEqual(env.shadowFraction(7e6*u, rs), 1);
            testCase.verifyEqual(env.shadowFraction(-7e6*u, rs), 0);
        end

        function eclipseDurationBetaZero(testCase)
            % 500 km circular orbit with the Sun in the orbit plane (beta = 0):
            % umbra lasts ~36 min of a 94.6 min orbit and each penumbra
            % crossing takes a few seconds, never minutes.
            rs = env.sunPosition(timesys.jd([2026 3 20 0 0 0]));
            u = rs / norm(rs);
            p = cross(u, [0; 0; 1]); p = p / norm(p);
            R = 6878137;
            P = 2*pi*sqrt(R^3 / 3.986004415e14);
            th = linspace(0, 2*pi, 20001);
            nu = arrayfun(@(a) env.shadowFraction(R*(cos(a)*u + sin(a)*p), rs), th);
            testCase.verifyEqual(mean(nu == 0)*P/60, 35.6, 'AbsTol', 0.5);
            penumbraSec = sum(nu > 0 & nu < 1) / numel(th) * P / 2;
            testCase.verifyGreaterThan(penumbraSec, 3);
            testCase.verifyLessThan(penumbraSec, 20);
        end

        function magneticFieldPointsDownInNorth(testCase)
            % The geomagnetic field enters the Earth in the northern
            % hemisphere and leaves it in the southern one. A sign error in
            % the NED -> ECI conversion flips both.
            jd = timesys.jd([2026 1 1 0 0 0]);
            rN = EnvironmentTest.eciAbove(60, 0, 500e3, jd);
            rS = EnvironmentTest.eciAbove(-60, 0, 500e3, jd);
            bN = env.magneticField(rN, jd, testCase.Eop);
            bS = env.magneticField(rS, jd, testCase.Eop);
            testCase.verifyLessThan(dot(bN, rN/norm(rN)), 0);
            testCase.verifyGreaterThan(dot(bS, rS/norm(rS)), 0);
        end

        function magneticFieldMagnitude(testCase)
            % Dipole-like: 20-35 uT near the equator, 35-55 uT near the pole
            % at 500 km. A nT/T mix-up misses by 1e9.
            jd = timesys.jd([2026 1 1 0 0 0]);
            bEq = norm(env.magneticField(EnvironmentTest.eciAbove(0, 90, 500e3, jd), jd, testCase.Eop));
            bPo = norm(env.magneticField(EnvironmentTest.eciAbove(89, 0, 500e3, jd), jd, testCase.Eop));
            testCase.verifyGreaterThan(bEq, 20e-6); testCase.verifyLessThan(bEq, 35e-6);
            testCase.verifyGreaterThan(bPo, 35e-6); testCase.verifyLessThan(bPo, 55e-6);
        end

        function densityDecreasesWithAltitude(testCase)
            jd = timesys.jd([2026 1 1 12 0 0]);
            rho = arrayfun(@(h) env.atmosphereDensity( ...
                EnvironmentTest.eciAbove(0, 0, h, jd), jd, testCase.Eop, testCase.Sw), ...
                [300 400 500 600]*1e3);
            testCase.verifyTrue(all(diff(rho) < 0));
            % Order of magnitude at 500 km for F10.7 = 150 (kg/m^3)
            testCase.verifyGreaterThan(rho(3), 1e-13);
            testCase.verifyLessThan(rho(3), 1e-11);
        end

        function densityDayNightBulge(testCase)
            % The diurnal bulge makes the sunlit side denser; checks that the
            % geodetic position / local time passed to NRLMSISE-00 is right.
            jd = timesys.jd([2026 3 20 14 46 0]);
            rs = env.sunPosition(jd);
            r = 6.9e6 * rs / norm(rs);
            testCase.verifyGreaterThan( ...
                env.atmosphereDensity(r, jd, testCase.Eop, testCase.Sw), ...
                2*env.atmosphereDensity(-r, jd, testCase.Eop, testCase.Sw));
        end
    end
end
