classdef DisturbanceTorqueTest < matlab.unittest.TestCase
    %DISTURBANCETORQUETEST Unit tests for the +torque package.
    %   Each test pins a closed-form value or a sign that a frame,
    %   quaternion-direction or unit error would break.

    properties (Constant)
        MU = 3.986004415e14
        J  = diag([12, 14, 10])
        Sc = struct('dragArea', 0.5, 'Cd', 2.2, 'srpArea', 1.0, 'Cr', 1.3, ...
                    'cpOffset', [0.05; 0; 0])
    end

    methods (TestClassSetup)
        function addSourcePath(testCase)
            root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(root, 'src'), 'IncludingSubfolders', true));
        end
    end

    methods (Test)
        function gravityGradientZeroOnPrincipalAxis(testCase)
            % Nadir along a principal axis is an equilibrium.
            tau = torque.gravityGradient([6.878e6; 0; 0], att.identity(), testCase.J);
            testCase.verifyEqual(tau, zeros(3, 1), 'AbsTol', 1e-20);
        end

        function gravityGradientAt45deg(testCase)
            % Body rotated 45 deg about y: |tau| = 1.5 mu/r^3 |Jx - Jz|,
            % about the y axis (the textbook maximum).
            r = 6.878e6;
            q = att.fromAxisAngle([0; 1; 0], pi/4);
            tau = torque.gravityGradient([r; 0; 0], q, testCase.J);
            expected = 1.5 * testCase.MU / r^3 * abs(testCase.J(1,1) - testCase.J(3,3));
            testCase.verifyEqual(abs(tau(2)), expected, 'RelTol', 1e-12);
            testCase.verifyEqual(tau([1 3]), [0; 0], 'AbsTol', 1e-20);
        end

        function gravityGradientRestoresMinAxisToNadir(testCase)
            % The axis of minimum inertia (z, 10 kg m^2) is pulled toward the
            % radial direction: a small tilt away from it gives a torque that
            % turns it back. Fails if A(q) is used as body->ECI instead of ECI->body.
            r = [6.878e6; 0; 0];
            q0 = att.fromDCM([0 0 1; 0 1 0; -1 0 0]);   % body z along -r (nadir)
            dq = att.fromAxisAngle([0; 1; 0], deg2rad(5));
            tau = torque.gravityGradient(r, att.multiply(q0, dq), testCase.J);
            testCase.verifyLessThan(tau(2), 0);
        end

        function aeroDragOpposesRelativeVelocity(testCase)
            % Force magnitude 0.5 rho Cd A v^2 against the flow; the
            % co-rotating atmosphere lowers v_rel for a prograde equatorial orbit.
            r = [6.878e6; 0; 0]; v = [0; 7612.6; 0]; rho = 1e-12;
            [tau, f] = torque.aerodynamic(r, v, att.identity(), rho, testCase.Sc);
            vRel = v - cross([0; 0; 7.292115146706979e-5], r);
            testCase.verifyEqual(norm(f), 0.5*rho*2.2*0.5*norm(vRel)^2, 'RelTol', 1e-12);
            testCase.verifyEqual(f/norm(f), -vRel/norm(vRel), 'AbsTol', 1e-12);
            testCase.verifyLessThan(norm(vRel), norm(v));
            testCase.verifyEqual(tau, cross(testCase.Sc.cpOffset, f), 'AbsTol', 1e-20);
        end

        function srpPushesAwayFromSunAndVanishesInUmbra(testCase)
            rSun = [149597870700; 0; 0];
            r = [0; 6.878e6; 0];
            [~, f] = torque.solarRadiation(r, rSun, att.identity(), 1, testCase.Sc);
            s = (rSun - r) / norm(rSun - r);
            testCase.verifyLessThan(dot(f, s), 0);
            testCase.verifyEqual(norm(f), 4.56e-6*1.3*1.0, 'RelTol', 1e-4);
            [tau0, f0] = torque.solarRadiation(r, rSun, att.identity(), 0, testCase.Sc);
            testCase.verifyEqual([tau0; f0], zeros(6, 1));
        end

        function magneticIsDipoleCrossField(testCase)
            % m along x, B along y (body = ECI) -> tau = |m||B| along z.
            tau = torque.magnetic([0; 30e-6; 0], att.identity(), [0.1; 0; 0]);
            testCase.verifyEqual(tau, [0; 0; 3e-6], 'AbsTol', 1e-20);
        end
    end
end
