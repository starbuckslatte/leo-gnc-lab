%DEMO_OPEN_LOOP One orbit with no control: tumbling body under all disturbances.
%   Runs sim.run with sim.nullFsw on the mission template and plots body
%   rates, disturbance torques and sensor validity.
%   Takes about 2-3 minutes for one orbit (dt = 0.01 s).
setupPaths();

m = leo_500km_imaging();
tic;
out = sim.run(m, sim.nullFsw());
fprintf('Simulated %.0f s in %.0f s CPU\n', m.sim.duration, toc);

t = out.t / 60;
figure('Name', 'Open loop: one orbit');
subplot(3, 1, 1);
plot(t, rad2deg(out.w)); grid on;
ylabel('\omega [deg/s]'); legend('x', 'y', 'z');
title('Body rate (no control)');

subplot(3, 1, 2);
semilogy(t, vecnorm(out.tau.gg), t, vecnorm(out.tau.aero), ...
         t, vecnorm(out.tau.srp), t, vecnorm(out.tau.mag)); grid on;
ylim([1e-9 1e-4]);
ylabel('|\tau| [N m]'); legend('gravity gradient', 'aero', 'SRP', 'magnetic');
title('Disturbance torques');

subplot(3, 1, 3);
plot(t, out.meas.strValid, t, 1.1*out.meas.sunValid); grid on;
ylim([-0.2 1.4]); yticks([0 1 1.1]); yticklabels({'invalid', 'STR valid', 'Sun valid'});
xlabel('time [min]');
title('Sensor validity');
