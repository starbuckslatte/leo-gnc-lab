classdef SimulationTest < matlab.unittest.TestCase
    %SIMULATIONTEST Closed-loop runner: scheduling, sensor timing, wiring.
    %   Timing errors (wrong sample, extra latency, drifting rates) bias
    %   every estimator and controller downstream without raising an error,
    %   so they are pinned here with ideal (noise-free) sensors.

    methods (TestClassSetup)
        function addPaths(testCase)
            root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(root, 'src'), 'IncludingSubfolders', true));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(root, 'missions')));
        end
    end

    methods (Static, Access = private)
        function m = idealMission(duration)
            % Noise-free sensors, no latency, short run.
            m = leo_500km_imaging();
            m.sim.duration = duration;
            m.sensors.gyro.sigmaV = 0; m.sensors.gyro.sigmaU = 0;
            m.sensors.gyro.initialBias = zeros(3, 1);
            m.sensors.starTracker.sigmaCross = 0; m.sensors.starTracker.sigmaRoll = 0;
        end

        function m = isolated(m)
            % No external torque at all: total angular momentum must be conserved.
            m.sim.disturbances = struct('gravityGradient', false, 'aero', false, ...
                                        'srp', false, 'magnetic', false);
        end

        function fsw = constantFsw(cmd)
            fsw.init = @(~) struct();
            fsw.step = @(s, ~, ~) deal(cmd, s);
        end

        function H = inertialMomentum(out, m)
            G = actuators.rwConfig(m.actuators.rw);
            H = zeros(3, numel(out.t));
            for k = 1:numel(out.t)
                H(:, k) = att.toDCM(out.q(:, k))' * (m.sc.inertia*out.w(:, k) + G*out.h(:, k));
            end
        end
    end

    methods (Test)
        function rejectsNonIntegerRate(testCase)
            % 3 Hz is not a multiple of dt = 0.01 s; silently rounding it
            % would create a drifting, beating sample pattern.
            m = SimulationTest.idealMission(1);
            m.sensors.gyro.rate = 3;
            testCase.verifyError(@() sim.run(m, sim.nullFsw()), 'sim:run:rate');
        end

        function idealSensorsSampleTruthAtTick(testCase)
            % Zero noise and zero delay: the gyro reading the FSW sees equals
            % the true rate at that tick, and each star-tracker sample equals
            % the true attitude at its own sample time.
            m = SimulationTest.idealMission(5);
            out = sim.run(m, sim.nullFsw());
            testCase.verifyEqual(out.meas.gyro, out.w, 'AbsTol', 1e-15);
            testCase.verifyEqual(out.meas.gyroT, out.t);
            for k = 1:numel(out.t)
                j = find(abs(out.t - out.meas.strT(k)) < 1e-9);
                testCase.verifyLessThan(att.errorAngle(out.meas.str(:, k), out.q(:, j)), 1e-12);
            end
        end

        function delayStepsShiftBySamples(testCase)
            % delaySteps = 2 at 10 Hz: the reading is the truth from 0.2 s
            % earlier, while its time tag is the time it was delivered
            % (the FSW is not told about the latency).
            m = SimulationTest.idealMission(3);
            m.sensors.gyro.delaySteps = 2;
            out = sim.run(m, sim.nullFsw());
            testCase.verifyEqual(out.meas.gyro(:, 3:end), out.w(:, 1:end-2), 'AbsTol', 1e-15);
            testCase.verifyEqual(out.meas.gyroT, out.t);
        end

        function wheelCommandRoutedAndMomentumConserved(testCase)
            % Constant command on wheel 1 from t = 0: h1 = tau*t, and with no
            % external torque the inertial total momentum stays constant.
            m = SimulationTest.isolated(SimulationTest.idealMission(20));
            cmd = struct('rw', [0.01; 0; 0; 0], 'mtq', [], 'thruster', []);
            out = sim.run(m, SimulationTest.constantFsw(cmd));
            testCase.verifyEqual(out.h(1, end), 0.01*out.t(end), 'RelTol', 1e-9);
            H = SimulationTest.inertialMomentum(out, m);
            testCase.verifyEqual(H(:, end), H(:, 1), 'AbsTol', 1e-10);
        end

        function thrustChangesOrbitVelocity(testCase)
            % All four 0.1 N thrusters for 10 s on 100 kg: |dv| =
            % 4*0.1*cos(10 deg)*10/100 = 0.0394 m/s along body +x.
            m = SimulationTest.isolated(SimulationTest.idealMission(10));
            m.sim.w0 = zeros(3, 1);
            m.env.gravity.degree = 0;
            base = sim.run(m, sim.nullFsw());
            cmd = struct('rw', [], 'mtq', [], 'thruster', 0.1*ones(4, 1));
            burn = sim.run(m, SimulationTest.constantFsw(cmd));
            dv = burn.v(:, end) - base.v(:, end);
            xBody = att.toDCM(base.q(:, 1))' * [1; 0; 0];
            testCase.verifyEqual(norm(dv), 4*0.1*cosd(10)*10/100, 'RelTol', 1e-3);
            testCase.verifyGreaterThan(dot(dv/norm(dv), xBody), 0.9999);
        end
    end
end
