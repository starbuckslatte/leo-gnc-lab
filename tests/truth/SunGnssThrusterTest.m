classdef SunGnssThrusterTest < matlab.unittest.TestCase
    %SUNGNSSTHRUSTERTEST Sun sensor, GNSS receiver and thruster models.

    properties (Constant)
        Sun  = struct('sigma', deg2rad(0.5), 'fovHalf', deg2rad(60), 'boresight', [0; 0; 1])
        Gnss = struct('sigmaPos', 5, 'sigmaVel', 0.05)
        Jd   = 2461041.5                     % 2026-01-01 00:00 UTC
    end

    methods (TestClassSetup)
        function addSourcePath(testCase)
            root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(root, 'src'), 'IncludingSubfolders', true));
        end
    end

    methods (TestMethodSetup)
        function fixSeed(~)
            rng(5, 'twister');
        end
    end

    methods (Test)
        function sunSensorDirectionAndNoise(testCase)
            % Sun on body +z (boresight). Two noise axes of sigma each give an
            % RMS angular error of sqrt(2)*sigma; the mean is the true direction.
            r = [7e6; 0; 0]; rSun = r + [0; 0; 1.5e11];
            s = sensors.sunSensorInit(testCase.Sun);
            N = 5000; ang = zeros(1, N); v = zeros(3, N);
            for k = 1:N
                [v(:, k), ok, s] = sensors.sunSensor(att.identity(), r, rSun, 1, s, testCase.Sun);
                testCase.assertTrue(ok);
                ang(k) = acos(min(1, v(3, k)));
            end
            testCase.verifyEqual(sqrt(mean(ang.^2)), sqrt(2)*testCase.Sun.sigma, 'RelTol', 0.05);
            m = mean(v, 2);
            testCase.verifyEqual(m/norm(m), [0; 0; 1], 'AbsTol', 1e-3);
        end

        function sunSensorValidity(testCase)
            % Invalid outside the field of view or in umbra.
            r = [7e6; 0; 0];
            s = sensors.sunSensorInit(testCase.Sun);
            [~, inFov]  = sensors.sunSensor(att.identity(), r, r + [0; 0; 1.5e11], 1, s, testCase.Sun);
            [~, outFov] = sensors.sunSensor(att.identity(), r, r + [1.5e11; 0; 0], 1, s, testCase.Sun);
            [~, umbra]  = sensors.sunSensor(att.identity(), r, r + [0; 0; 1.5e11], 0, s, testCase.Sun);
            testCase.verifyTrue(inFov);
            testCase.verifyFalse(outFov);
            testCase.verifyFalse(umbra);
        end

        function gnssReportsEcefWithNoise(testCase)
            % The fix is in ECEF (as real receivers report), not ECI: mapping
            % the mean back with frames.ecefToEci recovers the ECI truth.
            rEci = [6.878e6; 1e5; -2e5]; vEci = [10; 7600; 50];
            [rE, vE] = frames.eciToEcef(rEci, vEci, testCase.Jd, struct());
            s = sensors.gnssInit(testCase.Gnss);
            N = 4000; R = zeros(3, N); V = zeros(3, N);
            for k = 1:N
                [m, s] = sensors.gnss(rEci, vEci, testCase.Jd, struct(), s, testCase.Gnss);
                R(:, k) = m.rEcef; V(:, k) = m.vEcef;
            end
            testCase.verifyEqual(mean(R, 2), rE, 'AbsTol', 4*5/sqrt(N));
            testCase.verifyEqual(std(R, 0, 2), 5*ones(3, 1), 'RelTol', 0.05);
            testCase.verifyEqual(mean(V, 2), vE, 'AbsTol', 4*0.05/sqrt(N));
            testCase.verifyEqual(std(V, 0, 2), 0.05*ones(3, 1), 'RelTol', 0.05);
            testCase.verifyGreaterThan(norm(rE - rEci), 1e5);
            [rBack, vBack] = frames.ecefToEci(mean(R, 2), mean(V, 2), testCase.Jd, struct());
            testCase.verifyEqual(rBack, rEci, 'AbsTol', 1);
            testCase.verifyEqual(vBack, vEci, 'AbsTol', 0.01);
        end

        function gnssTimeTag(testCase)
            % GPS time of 2026-01-01 00:00 UTC: week 2399, GPS = UTC + 18 s.
            s = sensors.gnssInit(testCase.Gnss);
            m = sensors.gnss([7e6; 0; 0], [0; 7.5e3; 0], testCase.Jd, struct(), s, testCase.Gnss);
            [w, sow] = timesys.utcToGps(testCase.Jd);
            testCase.verifyEqual([m.week, m.sow], [w, sow]);
            testCase.verifyEqual(w, 2399);
            testCase.verifyEqual(mod(sow, 86400), 18, 'AbsTol', 1e-4);
        end

        function thrusterPushOnlyAndLeverArm(testCase)
            % A thruster cannot pull; torque = r x F.
            p = struct('positions', [-0.5; 0.2; 0], 'directions', [1; 0; 0], 'maxThrust', 0.1);
            [f0, t0] = actuators.thruster(-0.05, p, 1);
            testCase.verifyEqual([f0; t0], zeros(6, 1));
            [f, t] = actuators.thruster(0.08, p, 1);
            testCase.verifyEqual(f, [0.08; 0; 0], 'AbsTol', 1e-15);
            testCase.verifyEqual(t, cross([-0.5; 0.2; 0], [0.08; 0; 0]), 'AbsTol', 1e-15);
        end

        function thrusterPairGivesPureForce(testCase)
            % Two thrusters placed symmetrically about the CoM: net torque 0.
            p = struct('positions', [-0.5 -0.5; 0.2 -0.2; 0 0], ...
                       'directions', [1 1; 0 0; 0 0], 'maxThrust', 0.1);
            [f, t] = actuators.thruster([0.1; 0.1], p, 1);
            testCase.verifyEqual(f, [0.2; 0; 0], 'AbsTol', 1e-15);
            testCase.verifyEqual(t, zeros(3, 1), 'AbsTol', 1e-15);
        end

        function thrusterMinimumImpulseBit(testCase)
            % 33% duty with on-time steps of 10% of the period -> 30%.
            p = struct('positions', [0; 0; 0], 'directions', [1; 0; 0], ...
                       'maxThrust', 0.1, 'minOnTime', 0.01);
            [~, ~, thrust] = actuators.thruster(0.033, p, 0.1);
            testCase.verifyEqual(thrust, 0.03, 'AbsTol', 1e-15);
        end
    end
end
