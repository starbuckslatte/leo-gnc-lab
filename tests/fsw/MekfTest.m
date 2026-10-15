classdef MekfTest < matlab.unittest.TestCase
    %MEKFTEST Multiplicative EKF (gyro propagation + star tracker update).
    %   Driven directly (no sim.run) so it runs fast. The consistency test is
    %   the key one: an estimator whose errors exceed its own 3-sigma is
    %   over-confident and will mislead FDIR and control.

    properties (Constant)
        Gyro = struct('sigmaV', sqrt(10)*1e-7, 'sigmaU', sqrt(10)*1e-10, ...
                      'initialBias', deg2rad(0.1/3600)*[1; -2; 1.5])
        Str  = struct('sigmaCross', deg2rad(6/3600), 'sigmaRoll', deg2rad(40/3600), ...
                      'boresight', [0; 1; 0], 'sunExcl', 0, 'earthExcl', -pi)
        J    = diag([12, 14, 10])
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
            rng(21, 'twister');
        end
    end

    methods (Static, Access = private)
        function e = attitudeError(qTrue, qHat)
            % Body-frame small-angle error dtheta with q_true = q_hat (x) dq
            dq = att.multiply(att.conj(qHat), qTrue);
            e = 2*sign(dq(1))*dq(2:4);
        end
    end

    methods (Test)
        function estimatorFswRunsInClosedLoop(testCase)
            % Wiring check through sim.run: the FSW only sees measurements,
            % initialises from the star tracker, logs [q; beta; sigma], and
            % ends within 3 sigma of the truth.
            root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture(fullfile(root, 'missions')));
            m = leo_500km_imaging();
            m.sim.duration = 20;
            out = sim.run(m, mekf.estimatorFsw());
            testCase.verifySize(out.fsw, [13, numel(out.t)]);
            testCase.assumeFalse(isnan(out.fsw(1, end)), 'Star tracker never valid in this window.');
            e = MekfTest.attitudeError(out.q(:, end), out.fsw(1:4, end));
            testCase.verifyLessThan(abs(e), 3*out.fsw(8:10, end));
        end

        function noiselessPropagationTracksTruth(testCase)
            % Constant rate, perfect gyro: the estimate follows the truth exactly.
            w = [0.01; -0.02; 0.015];
            q = att.fromAxisAngle([1; 1; 0], 0.3);
            f = mekf.init(q, zeros(3, 1), eye(6)*1e-6);
            for k = 1:1000
                q = att.propagate(q, w, 0.1);
                f = mekf.propagate(f, w, 0.1, 0, 0);
            end
            testCase.verifyLessThan(att.errorAngle(q, f.q), 1e-12);
        end

        function covarianceGrowsAsGyroNoise(testCase)
            % At zero rate with no updates, the attitude variance grows as
            % P0 + sigmaV^2 T + sigmaU^2 T^3/3 (+ bias term P0b T^2). Checks Q and Phi.
            p = testCase.Gyro;
            P0 = diag([1e-8*ones(1, 3), 1e-12*ones(1, 3)]);
            f = mekf.init(att.identity(), zeros(3, 1), P0);
            dt = 0.1; T = 100;
            for k = 1:round(T/dt)
                f = mekf.propagate(f, zeros(3, 1), dt, p.sigmaV, p.sigmaU);
            end
            expected = 1e-8 + 1e-12*T^2 + p.sigmaV^2*T + p.sigmaU^2*T^3/3;
            testCase.verifyEqual(f.P(1, 1), expected, 'RelTol', 1e-6);
        end

        function perfectMeasurementCorrectsAttitude(testCase)
            % Large prior, near-perfect measurement: the estimate jumps to
            % the measurement and the attitude variance drops to ~R.
            qT = att.fromAxisAngle([0; 0; 1], 0.3);
            qHat = att.multiply(qT, att.fromAxisAngle([1; 0; 0], deg2rad(0.5)));
            f = mekf.init(qHat, zeros(3, 1), diag([1e-4*ones(1, 3), 1e-10*ones(1, 3)]));
            R = 1e-14*eye(3);
            f = mekf.updateAttitude(f, qT, R);
            testCase.verifyLessThan(att.errorAngle(qT, f.q), 1e-6);
            testCase.verifyEqual(diag(f.P(1:3, 1:3)), 1e-14*ones(3, 1), 'RelTol', 1e-3);
        end

        function tumblingConsistencyAndBiasConvergence(testCase)
            % Torque-free tumble, gyro at 10 Hz, star tracker at 2 Hz, 1 deg
            % initial attitude error and unknown bias. After convergence,
            % >= 97% of errors per axis lie within +/-3 sigma, and the bias
            % estimate ends within 3 sigma of the true bias.
            p = testCase.Gyro;
            dt = 0.1; T = 600; n = round(T/dt);
            [~, Q, W] = attitude.propagate(att.fromAxisAngle([1; 2; 3], 0.5), ...
                deg2rad([0.5; -0.3; 0.2]), testCase.J, @(~, ~, ~) zeros(3, 1), dt, T);
            gs = sensors.gyroInit(p);
            ss = sensors.starTrackerInit(testCase.Str);
            R = mekf.starTrackerCovariance(testCase.Str);
            q0 = att.multiply(Q(:, 1), att.fromAxisAngle([1; -1; 1], deg2rad(1)));
            f = mekf.init(q0, zeros(3, 1), diag([deg2rad(2)^2*ones(1, 3), deg2rad(1/3600)^2*ones(1, 3)]));
            err = zeros(3, n); sig = zeros(3, n);
            [wm, gs] = sensors.gyro(W(:, 1), gs, p, dt);
            for k = 1:n
                % Trapezoid: average of the samples bounding the interval.
                % Using only the first one leaves a 0.5*wdot*dt error that
                % the filter mistakes for gyro bias.
                wPrev = wm;
                [wm, gs] = sensors.gyro(W(:, k+1), gs, p, dt);
                f = mekf.propagate(f, 0.5*(wPrev + wm), dt, p.sigmaV, p.sigmaU);
                if mod(k, 5) == 0
                    [qm, ok, ss] = sensors.starTracker(Q(:, k+1), [7e6; 0; 0], [0; 0; 1.5e11], ss, testCase.Str);
                    if ok, f = mekf.updateAttitude(f, qm, R); end
                end
                err(:, k) = MekfTest.attitudeError(Q(:, k+1), f.q);
                sig(:, k) = sqrt(diag(f.P(1:3, 1:3)));
            end
            settled = 1001:n;                                   % after 100 s
            inside = mean(abs(err(:, settled)) < 3*sig(:, settled), 2);
            testCase.verifyGreaterThanOrEqual(inside, 0.97*ones(3, 1));
            sb = sqrt(diag(f.P(4:6, 4:6)));
            testCase.verifyLessThan(abs(f.beta - gs.bias), 3*sb);
        end
    end
end
