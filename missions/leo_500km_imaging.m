function m = leo_500km_imaging()
%LEO_500KM_IMAGING Mission configuration template: 500 km SSO imaging smallsat.
%
%   m = leo_500km_imaging() returns a struct consumed by the simulator.
%   Copy this file to create new scenarios (e.g. vleo_250km.m).
%
%   UNITS: SI throughout (m, s, kg, rad, T). Angles stored in rad.
%   SOURCES: All values are ILLUSTRATIVE, taken from textbooks or typical
%   public datasheet ranges. No proprietary or program-specific data.

%% Identification
m.name  = 'LEO 500 km SSO imaging (template)';
m.epoch = datetime(2026, 1, 1, 0, 0, 0, 'TimeZone', 'UTC');

%% Orbit (osculating Keplerian elements at epoch)
m.orbit.altitude    = 500e3;          % m, above mean equatorial radius
m.orbit.ecc         = 0.001;
m.orbit.inc         = deg2rad(97.4);  % approx. sun-synchronous at 500 km
m.orbit.ltan        = 10.5;           % h, local time of ascending node (sets RAAN)
m.orbit.raan        = NaN;            % rad, NaN = derive from LTAN and epoch
m.orbit.argp        = deg2rad(90);
m.orbit.trueAnomaly = 0;

%% Spacecraft physical properties
m.sc.mass           = 100;                 % kg
m.sc.inertia        = diag([12, 14, 10]);  % kg m^2, body frame at CoM
m.sc.dragArea       = 0.5;                 % m^2 (constant-area model)
m.sc.Cd             = 2.2;                 % drag coefficient
m.sc.cpOffset       = [0.05; 0; 0];        % m, centre of pressure - CoM (aero torque)
m.sc.srpArea        = 1.0;                 % m^2
m.sc.Cr             = 1.3;                 % SRP reflectivity coefficient
m.sc.residualDipole = [0.1; 0.1; 0.1];     % A m^2

%% Environment models (Aerospace Toolbox names where applicable)
m.env.gravity.model      = 'EGM2008';      % gravitysphericalharmonic
m.env.gravity.degree     = 20;
m.env.atmosphere.model   = 'nrlmsise00';   % atmosnrlmsise00
m.env.atmosphere.f107    = 150;            % solar flux (moderate activity)
m.env.atmosphere.f107a   = 150;
m.env.atmosphere.ap      = 4;
m.env.magfield.model     = 'igrf';         % igrfmagm
m.env.thirdBody          = {'Sun', 'Moon'};
m.env.srp                = true;
m.env.eclipse            = true;

%% Sensors
% Gyro: Farrenkopf model. ARW/RRW values from Crassidis & Junkins example.
m.sensors.gyro.rate        = 10;                       % Hz
m.sensors.gyro.sigmaV      = sqrt(10)*1e-7;            % rad/s^(1/2), angle random walk
m.sensors.gyro.sigmaU      = sqrt(10)*1e-10;           % rad/s^(3/2), rate random walk
m.sensors.gyro.initialBias = deg2rad(0.1/3600)*[1; 1; 1];  % rad/s (0.1 deg/h)
m.sensors.gyro.quantization = 0;                       % rad/s per LSB (0 = ideal)
m.sensors.gyro.delaySteps   = 0;                       % samples of latency (0 = ideal)

m.sensors.starTracker.rate       = 2;                  % Hz
m.sensors.starTracker.sigmaCross = deg2rad(6/3600);    % rad (6 arcsec, 1-sigma)
m.sensors.starTracker.sigmaRoll  = deg2rad(40/3600);   % rad (40 arcsec, 1-sigma)
m.sensors.starTracker.boresight  = [0; 1; 0];          % body frame
m.sensors.starTracker.sunExcl    = deg2rad(30);        % rad, sun exclusion half-angle
m.sensors.starTracker.earthExcl  = deg2rad(20);        % rad, Earth-limb exclusion
m.sensors.starTracker.delaySteps = 0;                  % samples of latency (0 = ideal)

m.sensors.magnetometer.rate  = 10;                     % Hz
m.sensors.magnetometer.sigma = 100e-9;                 % T
m.sensors.magnetometer.bias  = [0; 0; 0];              % T
m.sensors.magnetometer.mtqCoupling  = zeros(3);        % T/(A m^2), MTQ field at sensor (0 = ideal)
m.sensors.magnetometer.quantization = 0;               % T per LSB (0 = ideal)
m.sensors.magnetometer.delaySteps   = 0;               % samples of latency (0 = ideal)

m.sensors.sunSensor.rate  = 10;                        % Hz
m.sensors.sunSensor.sigma = deg2rad(0.5);              % rad, per axis normal to the Sun line
m.sensors.sunSensor.fovHalf   = deg2rad(60);           % rad, field-of-view half-angle
m.sensors.sunSensor.boresight = [0; 0; -1];            % body frame
m.sensors.sunSensor.delaySteps = 0;                    % samples of latency (0 = ideal)

m.sensors.gnss.rate     = 1;                           % Hz
m.sensors.gnss.sigmaPos = 5;                           % m
m.sensors.gnss.sigmaVel = 0.05;                        % m/s
m.sensors.gnss.delaySteps = 0;                         % samples of latency (0 = ideal)

%% Actuators
m.actuators.rw.count       = 4;
m.actuators.rw.config      = 'pyramid';
m.actuators.rw.maxTorque   = 0.02;                     % N m
m.actuators.rw.maxMomentum = 0.5;                      % N m s
m.actuators.rw.cantAngle   = deg2rad(54.7356);         % rad, spin axis from body +z (equal 3-axis capacity)
m.actuators.rw.wheelInertia = 1e-3;                    % kg m^2 (sets wheel speed for friction)
m.actuators.rw.coulomb     = 0;                        % N m, Coulomb friction (0 = ideal; typ. 1e-4)
m.actuators.rw.viscous     = 0;                        % N m s, viscous friction (0 = ideal; typ. 1e-6)

m.actuators.mtq.maxDipole  = 15;                       % A m^2 per axis
m.actuators.mtq.quantization = 0;                      % A m^2 per step (0 = ideal)

% Thrusters: 4 x 0.1 N on the -x face, canted 10 deg outward, firing toward +x
c = cosd(10); s10 = sind(10);
m.actuators.thruster.positions  = [-0.5 -0.5 -0.5 -0.5;
                                    0.3 -0.3  0.3 -0.3;
                                    0.3  0.3 -0.3 -0.3];   % m, from CoM
m.actuators.thruster.directions = [c c c c;
                                   -s10 s10 -s10 s10;
                                    0 0 0 0];              % unit force on the spacecraft
m.actuators.thruster.maxThrust  = 0.1;                 % N
m.actuators.thruster.minOnTime  = 0;                   % s, valve on-time step (0 = ideal)

%% Flight software
m.fsw.rate = 10;                                       % Hz, main control loop

%% Simulation
m.sim.dt       = 0.01;                                 % s, truth integration step
m.sim.duration = 5400;                                 % s (~1 orbit)
m.sim.seed     = 1;                                    % RNG seed for reproducibility
end
