classdef SensorTest < matlab.unittest.TestCase
    %SENSORTEST Unit tests for the +sensors package (gyro, star tracker, hardware effects).
    %   Noise tests check that the generated statistics match the stated
    %   1-sigma parameters, because the MEKF (Phase 4) is tuned with them.

    properties (Constant)
        Gyro = struct('sigmaV', sqrt(10)*1e-7, 'sigmaU', sqrt(10)*1e-10, ...
                      'initialBias', [1e-5; -2e-5; 3e-5])
        Str  = struct('sigmaCross', deg2rad(6/3600), 'sigmaRoll', deg2rad(40/3600), ...
                      'boresight', [0; 1; 0], 'sunExcl', deg2rad(30), 'earthExcl', deg2rad(20))
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
            rng(7, 'twister');
        end
    end

    methods (Test)
        function gyroWhiteNoiseLevel(testCase)
            % With RRW off, the noise std is sigmaV/sqrt(dt) around the
            % constant initial bias.
            p = testCase.Gyro; p.sigmaU = 0;
            dt = 0.1; N = 20000;
            s = sensors.gyroInit(p);
            e = zeros(3, N);
            for k = 1:N
                [w, s] = sensors.gyro(zeros(3, 1), s, p, dt);
                e(:, k) = w;
            end
            testCase.verifyEqual(mean(e, 2), p.initialBias, 'AbsTol', 4*p.sigmaV/sqrt(dt)/sqrt(N));
            testCase.verifyEqual(std(e, 0, 2), p.sigmaV/sqrt(dt)*ones(3, 1), 'RelTol', 0.03);
        end

        function gyroBiasRandomWalk(testCase)
            % Bias variance grows as sigmaU^2 * t (Monte Carlo over runs).
            p = testCase.Gyro; p.initialBias = zeros(3, 1);
            dt = 1; nStep = 100; nRun = 2000;
            b = zeros(3, nRun);
            for r = 1:nRun
                s = sensors.gyroInit(p);
                for k = 1:nStep
                    [~, s] = sensors.gyro(zeros(3, 1), s, p, dt);
                end
                b(:, r) = s.bias;
            end
            testCase.verifyEqual(std(b(:)), p.sigmaU*sqrt(nStep*dt), 'RelTol', 0.05);
        end

        function delayReturnsOldSample(testCase)
            % N-sample delay: output k equals input k-N. Models the sensor
            % latency that FSW must compensate (e.g. star tracker).
            buf = repmat(0, 1, 3);
            out = zeros(1, 10);
            for k = 1:10
                [out(k), buf] = sensors.delay(k, buf);
            end
            testCase.verifyEqual(out, [0 0 0 1:7]);
        end

        function quantizationOnLsbGrid(testCase)
            y = sensors.quantize([0.26; -0.74; 1.0], 0.5);
            testCase.verifyEqual(y, [0.5; -0.5; 1.0]);
            testCase.verifyEqual(sensors.quantize(0.123, 0), 0.123);
        end

        function starTrackerNoiseAxes(testCase)
            % Error statistics in the sensor frame: cross-boresight sigma on
            % body x and z (boresight = body y), roll sigma about body y.
            q = att.fromAxisAngle([1; 2; 3], 0.4);
            r = [7e6; 0; 0]; rSun = [0; 0; 1.5e11];
            s = sensors.starTrackerInit(testCase.Str);
            N = 5000; e = zeros(3, N);
            for k = 1:N
                [qm, ~, s] = sensors.starTracker(q, r, rSun, s, testCase.Str);
                dq = att.multiply(att.conj(q), qm);          % body-frame error
                e(:, k) = 2*sign(dq(1))*dq(2:4);
            end
            sd = std(e, 0, 2);
            testCase.verifyEqual(sd([1 3]), testCase.Str.sigmaCross*[1; 1], 'RelTol', 0.05);
            testCase.verifyEqual(sd(2), testCase.Str.sigmaRoll, 'RelTol', 0.05);
        end

        function starTrackerExclusionZones(testCase)
            % Boresight (body y) toward the Sun or the Earth -> invalid;
            % toward deep space -> valid.
            p = testCase.Str; p.sigmaCross = 0; p.sigmaRoll = 0;
            r = [7e6; 0; 0];
            q = att.identity();                              % body = ECI, boresight +y
            s = sensors.starTrackerInit(p);
            [~, vSun] = sensors.starTracker(q, r, r + [0; 1.5e11; 0], s, p);
            [~, vSpace] = sensors.starTracker(q, r, [0; 0; 1.5e11], s, p);
            qEarth = att.fromAxisAngle([0; 0; 1], pi/2);    % body y -> ECI -x (nadir)
            testCase.assertEqual(att.toDCM(qEarth)' * [0; 1; 0], [-1; 0; 0], 'AbsTol', 1e-15);
            [~, vEarth] = sensors.starTracker(qEarth, r, [0; 0; 1.5e11], s, p);
            testCase.verifyFalse(vSun);
            testCase.verifyTrue(vSpace);
            testCase.verifyFalse(vEarth);
        end
    end
end
