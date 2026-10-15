classdef QuaternionTest < matlab.unittest.TestCase
    %QUATERNIONTEST Unit tests for the +att quaternion package.
    %   Convention under test: scalar-first, Hamilton product, passive DCM
    %   (same as B5G-AOCS-EGSE QMul / FrameA2B / M2Q).
    %   Run with:  runtests('tests')   or   runAllTests

    properties (Constant)
        Tol = 1e-12
        NumRandom = 100
    end

    methods (TestClassSetup)
        function addSourcePath(testCase)
            root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(root, 'src', 'utils')));
        end
    end

    methods (TestMethodSetup)
        function fixSeed(~)
            rng(42, 'twister');
        end
    end

    methods (Static, Access = private)
        function q = randomQuat()
            q = att.normalize(randn(4, 1));
        end
    end

    %% ---------------- Algebra ----------------
    methods (Test)
        function identityIsNeutral(testCase)
            q = QuaternionTest.randomQuat();
            testCase.verifyEqual(att.multiply(q, att.identity()), q, 'AbsTol', testCase.Tol);
            testCase.verifyEqual(att.multiply(att.identity(), q), q, 'AbsTol', testCase.Tol);
        end

        function conjugateGivesIdentity(testCase)
            q = QuaternionTest.randomQuat();
            testCase.verifyEqual(att.multiply(q, att.conj(q)), att.identity(), 'AbsTol', testCase.Tol);
        end

        function normalizeRejectsZero(testCase)
            testCase.verifyError(@() att.normalize(zeros(4, 1)), 'att:normalize:zeroNorm');
        end

        function hamiltonBasisProduct(testCase)
            % Pins the product convention: Hamilton gives i (x) j = +k
            % (Shuster would give -k). Must match EGSE QMul.
            i = [0; 1; 0; 0]; j = [0; 0; 1; 0]; k = [0; 0; 0; 1];
            testCase.verifyEqual(att.multiply(i, j), k, 'AbsTol', testCase.Tol);
        end

        function skewMatchesCross(testCase)
            a = randn(3, 1); b = randn(3, 1);
            testCase.verifyEqual(att.skew(a)*b, cross(a, b), 'AbsTol', testCase.Tol);
        end
    end

    %% ---------------- DCM ----------------
    methods (Test)
        function compositionMatchesDcmProduct(testCase)
            % Left-to-right frame composition: Qa2c = Qa2b (x) Qb2c,
            % so A(q (x) p) = A(p) A(q).
            for k = 1:testCase.NumRandom
                q = QuaternionTest.randomQuat();
                p = QuaternionTest.randomQuat();
                testCase.verifyEqual(att.toDCM(att.multiply(q, p)), ...
                    att.toDCM(p)*att.toDCM(q), 'AbsTol', 1e-12);
            end
        end

        function dcmIsProperRotation(testCase)
            A = att.toDCM(QuaternionTest.randomQuat());
            testCase.verifyEqual(A'*A, eye(3), 'AbsTol', testCase.Tol);
            testCase.verifyEqual(det(A), 1, 'AbsTol', testCase.Tol);
        end

        function knownRotationAboutZ(testCase)
            % Frame rotated +90 deg about z: inertial x-axis appears as -y in body.
            A = att.toDCM(att.fromAxisAngle([0; 0; 1], pi/2));
            expected = [ 0  1  0;
                        -1  0  0;
                         0  0  1];
            testCase.verifyEqual(A, expected, 'AbsTol', testCase.Tol);
            testCase.verifyEqual(A*[1; 0; 0], [0; -1; 0], 'AbsTol', testCase.Tol);
        end

        function dcmRoundTripRandom(testCase)
            for k = 1:testCase.NumRandom
                q = QuaternionTest.randomQuat();
                q2 = att.fromDCM(att.toDCM(q));
                testCase.verifyLessThan(att.errorAngle(q, q2), 1e-10);
                testCase.verifyGreaterThanOrEqual(q2(1), 0);
            end
        end

        function dcmRoundTripNear180deg(testCase)
            % Exercises every branch of Shepperd's method.
            axes = {[1;0;0], [0;1;0], [0;0;1], [1;1;1]};
            for k = 1:numel(axes)
                q = att.fromAxisAngle(axes{k}, pi - 1e-3);
                q2 = att.fromDCM(att.toDCM(q));
                testCase.verifyLessThan(att.errorAngle(q, q2), 1e-10);
            end
        end

        function axisAngleRoundTrip(testCase)
            e = randn(3, 1); e = e/norm(e);
            theta = 1.234;
            [e2, theta2] = att.toAxisAngle(att.fromAxisAngle(e, theta));
            testCase.verifyEqual(e2, e, 'AbsTol', 1e-12);
            testCase.verifyEqual(theta2, theta, 'AbsTol', 1e-12);
        end
    end

    %% ---------------- Kinematics ----------------
    methods (Test)
        function derivativeMatchesDcmKinematics(testCase)
            % dA/dt = -[w x] A  (numerical check of qdot = 0.5*Omega(w)*q)
            q = QuaternionTest.randomQuat();
            w = 0.1*randn(3, 1);
            h = 1e-6;
            qd = att.derivative(q, w);
            Adot = (att.toDCM(q + h*qd) - att.toDCM(q - h*qd)) / (2*h);
            testCase.verifyEqual(Adot, -att.skew(w)*att.toDCM(q), 'AbsTol', 1e-8);
        end

        function propagateMatchesComposition(testCase)
            % Constant rate: q(t+dt) = q(t) (x) dq(w*dt)
            q0 = QuaternionTest.randomQuat();
            w  = [0.02; -0.05; 0.1];
            dt = 0.5;
            expected = att.multiply(q0, att.fromAxisAngle(w, norm(w)*dt));
            testCase.verifyEqual(att.propagate(q0, w, dt), expected, 'AbsTol', testCase.Tol);
        end

        function propagateMatchesOde45(testCase)
            q0 = QuaternionTest.randomQuat();
            w  = [0.03; 0.01; -0.02];
            T  = 10; dt = 0.1;
            q  = q0;
            for k = 1:round(T/dt)
                q = att.propagate(q, w, dt);
            end
            opts = odeset('RelTol', 1e-12, 'AbsTol', 1e-12);
            [~, Q] = ode45(@(t, x) att.derivative(x, w), [0 T], q0, opts);
            testCase.verifyLessThan(att.errorAngle(q, Q(end, :)'), 1e-8);
        end

        function propagatePreservesNorm(testCase)
            q = QuaternionTest.randomQuat();
            dt = 0.01;
            for k = 1:10000
                t = k*dt;
                w = [0.1*sin(0.01*t); 0.05*cos(0.02*t); 0.02];
                q = att.propagate(q, w, dt);
            end
            testCase.verifyEqual(norm(q), 1, 'AbsTol', testCase.Tol);
        end

        function propagateZeroRate(testCase)
            q = QuaternionTest.randomQuat();
            testCase.verifyEqual(att.propagate(q, zeros(3, 1), 1.0), q, 'AbsTol', testCase.Tol);
        end
    end

    %% ---------------- Error metrics ----------------
    methods (Test)
        function errorAngleKnownValue(testCase)
            q1 = QuaternionTest.randomQuat();
            q2 = att.multiply(att.fromAxisAngle([1; 2; 3], 0.1), q1);
            testCase.verifyEqual(att.errorAngle(q2, q1), 0.1, 'AbsTol', 1e-12);
        end

        function errorAngleSignInvariant(testCase)
            q = QuaternionTest.randomQuat();
            testCase.verifyEqual(att.errorAngle(q, -q), 0, 'AbsTol', 1e-12);
        end
    end

    %% ---------------- Aerospace Toolbox interoperability ----------------
    methods (Test)
        function aerospaceToolboxConsistency(testCase)
            % Skipped automatically if Aerospace Toolbox is not installed.
            testCase.assumeTrue(exist('quat2dcm', 'file') == 2, ...
                'Aerospace Toolbox not available; skipping interop test.');
            for k = 1:20
                q = QuaternionTest.randomQuat();
                p = QuaternionTest.randomQuat();
                % Same layout and product; toolbox uses 1x4 rows.
                testCase.verifyEqual(quat2dcm(q'), att.toDCM(q), 'AbsTol', 1e-12);
                testCase.verifyEqual(quatmultiply(q', p'), att.multiply(q, p)', 'AbsTol', 1e-12);
            end
        end
    end
end
