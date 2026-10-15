classdef AttitudeDynamicsTest < matlab.unittest.TestCase
    %ATTITUDEDYNAMICSTEST Unit tests for the +attitude package (Euler + quaternion, RK4).

    properties (Constant)
        J = diag([12, 14, 10])      % kg m^2: max y, intermediate x, min z
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
        function torqueFreeConservesMomentumAndEnergy(testCase)
            % Inertial angular momentum A' J w and kinetic energy are
            % invariants. A wrong w x Jw sign or a body/inertial mix-up in
            % the quaternion breaks the first.
            q0 = att.fromAxisAngle([1; 2; 3], 0.7);
            w0 = [0.05; -0.02; 0.08];
            [~, Q, W] = attitude.propagate(q0, w0, testCase.J, @AttitudeDynamicsTest.noTorque, 0.1, 1000);
            H0 = att.toDCM(Q(:, 1))' * testCase.J * W(:, 1);
            Hend = att.toDCM(Q(:, end))' * testCase.J * W(:, end);
            T = @(w) 0.5 * w' * testCase.J * w;
            testCase.verifyEqual(Hend, H0, 'AbsTol', 1e-9 * norm(H0));
            testCase.verifyEqual(T(W(:, end)), T(W(:, 1)), 'RelTol', 1e-9);
        end

        function axisymmetricNutation(testCase)
            % J1 = J2: the transverse rate rotates in the body at
            % lambda = (J3 - J1)/J1 * w3 (closed-form solution of Euler's equations).
            Jsym = diag([10, 10, 20]);
            w0 = [0.01; 0; 0.1];
            [t, ~, W] = attitude.propagate(att.identity(), w0, Jsym, @AttitudeDynamicsTest.noTorque, 0.1, 100);
            lambda = (20 - 10)/10 * 0.1;
            testCase.verifyEqual(W(1, :), 0.01*cos(lambda*t), 'AbsTol', 1e-9);
            testCase.verifyEqual(W(2, :), 0.01*sin(lambda*t), 'AbsTol', 1e-9);
        end

        function intermediateAxisIsUnstable(testCase)
            % Spin about the major or minor axis stays put; spin about the
            % intermediate axis (x here) tumbles (tennis-racket theorem).
            eps0 = 1e-6;
            growth = zeros(1, 3);
            for ax = 1:3
                w0 = eps0 * ones(3, 1);
                w0(ax) = 0.1;
                [~, ~, W] = attitude.propagate(att.identity(), w0, testCase.J, ...
                    @AttitudeDynamicsTest.noTorque, 0.1, 600);
                off = setdiff(1:3, ax);
                growth(ax) = max(vecnorm(W(off, :))) / (eps0*sqrt(2));
            end
            testCase.verifyGreaterThan(growth(1), 100);   % intermediate
            testCase.verifyLessThan(growth(2), 10);       % major
            testCase.verifyLessThan(growth(3), 10);       % minor
        end

        function constantTorqueFromRest(testCase)
            % tau about +z from rest: w_z = alpha t, rotation angle 0.5 alpha t^2,
            % i.e. q = fromAxisAngle(+z, 0.5 alpha t^2). Pins the direction of
            % the torque -> rate -> quaternion chain.
            tauZ = 1e-3;
            alpha = tauZ / testCase.J(3, 3);
            [t, Q, W] = attitude.propagate(att.identity(), zeros(3, 1), testCase.J, ...
                @(~, ~, ~) [0; 0; tauZ], 0.5, 100);
            testCase.verifyEqual(W(3, end), alpha*t(end), 'RelTol', 1e-12);
            expected = att.fromAxisAngle([0; 0; 1], 0.5*alpha*t(end)^2);
            testCase.verifyLessThan(att.errorAngle(Q(:, end), expected), 1e-10);
        end
    end
end
