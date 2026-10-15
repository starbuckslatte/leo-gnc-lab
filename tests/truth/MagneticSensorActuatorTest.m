classdef MagneticSensorActuatorTest < matlab.unittest.TestCase
    %MAGNETICSENSORACTUATORTEST Magnetometer and magnetorquer models.

    properties (Constant)
        Mag = struct('sigma', 100e-9)
        Mtq = struct('maxDipole', 15)
        B   = [20e-6; -5e-6; 30e-6]     % T, ECI
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
            rng(11, 'twister');
        end
    end

    methods (Test)
        function magnetometerMeasuresBodyFrameField(testCase)
            % Mean = A(q) B (body frame), std = sigma. A frame error would
            % pass a zero-attitude test, so use a non-trivial q.
            q = att.fromAxisAngle([1; 2; -1], 1.1);
            s = sensors.magnetometerInit(testCase.Mag);
            N = 5000; y = zeros(3, N);
            for k = 1:N
                [y(:, k), s] = sensors.magnetometer(testCase.B, q, s, testCase.Mag);
            end
            testCase.verifyEqual(mean(y, 2), att.toDCM(q)*testCase.B, 'AbsTol', 4*100e-9/sqrt(N));
            testCase.verifyEqual(std(y, 0, 2), 100e-9*ones(3, 1), 'RelTol', 0.05);
        end

        function magnetometerSeesMtqField(testCase)
            % With MTQ coupling, the reading shifts by K * m. This is the
            % effect that forces magnetometer blanking during MTQ actuation.
            p = testCase.Mag; p.sigma = 0;
            p.mtqCoupling = 1e-7 * eye(3);
            s = sensors.magnetometerInit(p);
            m = [10; 0; -5];
            yOff = sensors.magnetometer(testCase.B, att.identity(), s, p, zeros(3, 1));
            yOn  = sensors.magnetometer(testCase.B, att.identity(), s, p, m);
            testCase.verifyEqual(yOn - yOff, 1e-7*m, 'AbsTol', 1e-15);
        end

        function mtqClampsEachCoil(testCase)
            m = actuators.mtqDipole([30; -2; -20], testCase.Mtq);
            testCase.verifyEqual(m, [15; -2; -15]);
        end

        function mtqCannotTorqueAlongField(testCase)
            % tau = m x B is always normal to B: magnetic control has no
            % authority about the field direction (why B-dot / MTQ-only
            % control is underactuated).
            q = att.fromAxisAngle([0; 1; 1], 0.6);
            rng(3);
            for k = 1:20
                m = actuators.mtqDipole(20*randn(3, 1), testCase.Mtq);
                tau = torque.magnetic(testCase.B, q, m);
                bB = att.toDCM(q) * testCase.B;
                testCase.verifyEqual(dot(tau, bB) / (norm(tau)*norm(bB)), 0, 'AbsTol', 1e-12);
            end
        end
    end
end
