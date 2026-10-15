classdef ReactionWheelTest < matlab.unittest.TestCase
    %REACTIONWHEELTEST Reaction wheel model and wheel/body coupling.

    properties (Constant)
        J  = diag([12, 14, 10])
        Rw = struct('config', 'pyramid', 'cantAngle', deg2rad(54.7356), ...
                    'maxTorque', 0.02, 'maxMomentum', 0.5)
    end

    methods (TestClassSetup)
        function addSourcePath(testCase)
            root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(root, 'src'), 'IncludingSubfolders', true));
        end
    end

    methods (Static, Access = private)
        function tau = noTorque(~, ~, ~)
            tau = zeros(3, 1);
        end
    end

    methods (Test)
        function pyramidIsSingleFailureTolerant(testCase)
            % Unit spin axes; any 3 of the 4 wheels still span 3-D, so one
            % wheel can fail without losing 3-axis control.
            G = actuators.rwConfig(testCase.Rw);
            testCase.verifyEqual(vecnorm(G), ones(1, 4), 'AbsTol', 1e-15);
            for k = 1:4
                testCase.verifyEqual(rank(G(:, setdiff(1:4, k))), 3);
            end
        end

        function totalMomentumConservedWithWheels(testCase)
            % Wheel torques are internal: inertial A'(J w + G h) must stay
            % constant while the wheels exchange momentum with the body.
            % Fails if the reaction -G hDot or the gyroscopic G h term is wrong.
            rw.G  = actuators.rwConfig(testCase.Rw);
            rw.h0 = [0.1; -0.05; 0.02; 0.0];
            rw.p  = testCase.Rw;
            rw.cmdFun = @(t, ~, ~, ~) 0.01*[sin(0.05*t); cos(0.03*t); -sin(0.02*t); 0.5];
            q0 = att.fromAxisAngle([1; -1; 2], 0.3);
            [~, Q, W, H] = attitude.propagate(q0, [0.01; -0.02; 0.005], testCase.J, ...
                @ReactionWheelTest.noTorque, 0.1, 300, rw);
            Hi = @(k) att.toDCM(Q(:, k))' * (testCase.J*W(:, k) + rw.G*H(:, k));
            testCase.verifyEqual(Hi(size(Q, 2)), Hi(1), 'AbsTol', 1e-10);
            testCase.verifyGreaterThan(max(abs(H(:) - repelem(rw.h0, size(H, 2)))), 0.1);
        end

        function wheelTorqueSpinsBodyOpposite(testCase)
            % One wheel on body z, from rest: body rate = -tau t / Jz.
            rw.G = [0; 0; 1]; rw.h0 = 0;
            rw.p = struct('maxTorque', 0.02, 'maxMomentum', 0.5);
            rw.cmdFun = @(~, ~, ~, ~) 0.01;
            [t, ~, W, H] = attitude.propagate(att.identity(), zeros(3, 1), testCase.J, ...
                @ReactionWheelTest.noTorque, 0.1, 20, rw);
            testCase.verifyEqual(W(3, end), -0.01*t(end)/testCase.J(3, 3), 'RelTol', 1e-12);
            testCase.verifyEqual(H(end), 0.01*t(end), 'RelTol', 1e-12);
        end

        function torqueAndMomentumLimits(testCase)
            p = struct('maxTorque', 0.02, 'maxMomentum', 0.5);
            testCase.verifyEqual(actuators.rwTorque([0.05; -0.05; 0.01], [0; 0; 0], p), ...
                [0.02; -0.02; 0.01]);
            % At the speed limit: torque that would spin faster is blocked,
            % torque that slows the wheel is allowed.
            testCase.verifyEqual(actuators.rwTorque([0.01; -0.01], [0.5; 0.5], p), [0; -0.01]);
        end

        function frictionSpinsDownFreeWheel(testCase)
            % Viscous only: h(t) = h0 exp(-viscous t / Iw). Coulomb only:
            % linear decay at coulomb [N m]. Both oppose the wheel speed.
            p = struct('maxTorque', 0.02, 'maxMomentum', 0.5, 'wheelInertia', 1e-3, ...
                       'coulomb', 0, 'viscous', 1e-6);
            rw = struct('G', [0; 0; 1], 'h0', 0.3, 'p', p, 'cmdFun', @(~, ~, ~, ~) 0);
            [t, ~, ~, H] = attitude.propagate(att.identity(), zeros(3, 1), testCase.J, ...
                @ReactionWheelTest.noTorque, 1, 100, rw);
            testCase.verifyEqual(H(end), 0.3*exp(-1e-6*t(end)/1e-3), 'RelTol', 1e-8);
            rw.p.coulomb = 1e-4; rw.p.viscous = 0;
            [t, ~, ~, H] = attitude.propagate(att.identity(), zeros(3, 1), testCase.J, ...
                @ReactionWheelTest.noTorque, 1, 100, rw);
            testCase.verifyEqual(H(end), 0.3 - 1e-4*t(end), 'RelTol', 1e-10);
        end
    end
end
