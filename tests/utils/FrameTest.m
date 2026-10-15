classdef FrameTest < matlab.unittest.TestCase
    %FRAMETEST Unit tests for the +frames package (IAU-76/FK5, LVLH).

    properties (Constant)
        % Vallado Ex. 3-15, 2004-04-06 07:51:28.386009 UTC
        DateUtc = [2004 4 6 7 51 28.386009]
        rEcef = [-1033.4793830; 7901.2952754; 6380.3565958]     % km
        vEcef = [-3.225636520; -2.872451450; 5.531924446]       % km/s
        rJ2000 = [5102.5096; 6123.01152; 6378.1363]              % km
        vJ2000 = [-4.7432196; 0.7905366; 5.53375619]             % km/s
    end

    methods (TestClassSetup)
        function addSourcePath(testCase)
            root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
            testCase.applyFixture(matlab.unittest.fixtures.PathFixture( ...
                fullfile(root, 'src', 'utils')));
        end
    end

    methods (Static, Access = private)
        function eop = valladoEop()
            A2R = pi/180/3600;
            eop = struct('dut1', -0.4399619, 'xp', -0.140682*A2R, ...
                         'yp', 0.333309*A2R, 'lod', 0.0015563);
        end
    end

    methods (Test)
        function ecefToEciVallado(testCase)
            % Full FK5 chain (precession, nutation, sidereal time, polar
            % motion, Earth-rate velocity term). 1e-4 km = 0.1 m is the
            % precision of the published result.
            jd = timesys.jd(testCase.DateUtc);
            [r, v] = frames.ecefToEci(testCase.rEcef, testCase.vEcef, jd, FrameTest.valladoEop());
            testCase.verifyEqual(r, testCase.rJ2000, 'AbsTol', 1e-4);
            testCase.verifyEqual(v, testCase.vJ2000, 'AbsTol', 1e-7);
        end

        function eciEcefRoundTrip(testCase)
            jd = timesys.jd(testCase.DateUtc);
            eop = FrameTest.valladoEop();
            [r, v] = frames.ecefToEci(testCase.rEcef, testCase.vEcef, jd, eop);
            [r2, v2] = frames.eciToEcef(r, v, jd, eop);
            testCase.verifyEqual(r2, testCase.rEcef, 'AbsTol', 1e-9);
            testCase.verifyEqual(v2, testCase.vEcef, 'AbsTol', 1e-12);
        end

        function fk5MatricesAreRotations(testCase)
            T = frames.fk5(timesys.jd(testCase.DateUtc), FrameTest.valladoEop());
            for M = {T.P, T.N, T.R, T.W}
                testCase.verifyEqual(M{1}'*M{1}, eye(3), 'AbsTol', 1e-14);
                testCase.verifyEqual(det(M{1}), 1, 'AbsTol', 1e-14);
            end
        end

        function lvlhAxes(testCase)
            % z points to nadir, y to negative orbit normal, x ~ along velocity
            % for a circular orbit. Must match EGSE PVi2Qi2l.
            r = [7000; 0; 0]; v = [0; 7.5; 0];
            [A, q] = frames.eciToLvlh(r, v);
            testCase.verifyEqual(A*(-r/norm(r)), [0; 0; 1], 'AbsTol', 1e-14);
            testCase.verifyEqual(A*(v/norm(v)), [1; 0; 0], 'AbsTol', 1e-14);
            testCase.verifyEqual(att.toDCM(q), A, 'AbsTol', 1e-14);
        end

        function aerospaceToolboxFk5(testCase)
            % Independent implementation of the same model; skipped without toolbox.
            testCase.assumeTrue(exist('dcmeci2ecef', 'file') == 2, ...
                'Aerospace Toolbox not available; skipping interop test.');
            A2R = pi/180/3600;
            T = frames.fk5(timesys.jd(testCase.DateUtc), FrameTest.valladoEop());
            C = dcmeci2ecef('IAU-76/FK5', testCase.DateUtc, 32, -0.4399619, ...
                [-0.140682 0.333309]*A2R);
            testCase.verifyEqual(C, (T.P*T.N*T.R*T.W)', 'AbsTol', 1e-9);
        end
    end
end
