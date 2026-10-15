classdef OrbitTest < matlab.unittest.TestCase
    %ORBITTEST Unit tests for the +orbit package (gravity + fixed-step RK4).

    properties (Constant)
        MU  = 3.986004415e14    % m^3/s^2, EGM2008
        RE  = 6378136.3         % m, EGM2008
        J2  = 1.0826266835e-3   % -sqrt(5)*C20, EGM2008
        Jd0 = 2461041.5         % 2026-01-01 00:00 UTC
    end

    methods (TestClassSetup)
        function addSourcePath(testCase)
            root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(root, 'src'), 'IncludingSubfolders', true));
        end
    end

    methods (Static, Access = private)
        function [x0, P] = circularOrbit(alt, inc)
            r = OrbitTest.RE + alt;
            v = sqrt(OrbitTest.MU / r);
            x0 = [r; 0; 0; 0; v*cos(inc); v*sin(inc)];
            P = 2*pi*sqrt(r^3 / OrbitTest.MU);
        end
    end

    methods (Test)
        function twoBodyConservesEnergyAndMomentum(testCase)
            % Integrator sanity: with a central field, specific energy and
            % angular momentum must stay constant (drift = integration error).
            [x0, P] = OrbitTest.circularOrbit(500e3, deg2rad(97.4));
            [~, X] = orbit.propagate(x0, testCase.Jd0, 10, P, struct(), 0);
            E = vecnorm(X(4:6, :)).^2/2 - testCase.MU ./ vecnorm(X(1:3, :));
            h = vecnorm(cross(X(1:3, :), X(4:6, :)));
            testCase.verifyLessThan(max(abs(E/E(1) - 1)), 1e-9);
            testCase.verifyLessThan(max(abs(h/h(1) - 1)), 1e-9);
        end

        function rk4IsFourthOrder(testCase)
            % Two-body returns exactly to its start after one period. Halving
            % dt must cut that error by ~2^4 = 16; a wrong RK4 coefficient
            % drops the ratio to 4 or less.
            [x0, P] = OrbitTest.circularOrbit(500e3, deg2rad(97.4));
            err = zeros(1, 2);
            dts = [P/100, P/200];
            for k = 1:2
                [~, X] = orbit.propagate(x0, testCase.Jd0, dts(k), P, struct(), 0);
                err(k) = norm(X(1:3, end) - x0(1:3));
            end
            testCase.verifyGreaterThan(err(1)/err(2), 12);
            testCase.verifyLessThan(err(1)/err(2), 20);
        end

        function j2NodalRegression(testCase)
            % EGM2008 to degree 2 must reproduce the secular J2 node drift
            %   dRAAN/dt = -1.5 n J2 (Re/a)^2 cos(i)
            % This also checks the ECI<->ECEF handling of the gravity field:
            % a wrongly rotated field gives the wrong drift. 3% allows for
            % the osculating/mean element difference, C22 and pole tilt.
            testCase.assumeTrue(exist('gravitysphericalharmonic', 'file') == 2, ...
                'Aerospace Toolbox not available; skipping.');
            inc = deg2rad(53);
            [x0, P] = OrbitTest.circularOrbit(600e3, inc);
            [t, X] = orbit.propagate(x0, testCase.Jd0, 60, 86400, struct(), 2);
            h = cross(X(1:3, :), X(4:6, :));
            raan = unwrap(atan2(h(1, :), -h(2, :)));
            c = polyfit(t, raan, 1);
            a = norm(x0(1:3));
            expected = -1.5 * (2*pi/P) * testCase.J2 * (testCase.RE/a)^2 * cos(inc);
            testCase.verifyEqual(c(1), expected, 'RelTol', 0.03);
        end
    end
end
