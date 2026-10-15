%STUDY_MEKF_MONTE_CARLO MEKF consistency over N closed-loop runs (sim.run).
%   Each run uses a different random seed (sensor noise, gyro bias walk) on
%   the mission template, with mekf.estimatorFsw (no control, tumbling).
%   The true initial gyro bias of each run is drawn from the filter's own
%   prior N(0, P0_bias): NEES is only meaningful when the truth is
%   distributed as the filter assumes (a fixed small bias with a wide P0
%   makes any filter look over-conservative).
%   Reports:
%     - average attitude NEES over runs vs the 95% chi-square band
%       (NEES uses the diagonal of P; P is logged as sqrt(diag) only)
%     - one run's per-axis attitude error with its +/-3 sigma bounds
%   A consistent filter keeps the average NEES inside the band.
setupPaths();

N_RUNS   = 20;
DURATION = 600;          % s per run
SETTLE   = 60;           % s, excluded from the statistics

m = leo_500km_imaging();
m.sim.duration = DURATION;
neesAll = [];
for i = 1:N_RUNS
    m.sim.seed = i;
    rng(1000 + i);
    m.sensors.gyro.initialBias = sqrt(diag(m.fsw.mekf.P0(4:6, 4:6))) .* randn(3, 1);
    out = sim.run(m, mekf.estimatorFsw());
    qHat = out.fsw(1:4, :);
    sig  = out.fsw(8:10, :);
    ok = ~isnan(qHat(1, :));
    e = nan(3, numel(out.t));
    for k = find(ok)
        dq = att.multiply(att.conj(qHat(:, k)), out.q(:, k));
        e(:, k) = 2*sign(dq(1))*dq(2:4);
    end
    if isempty(neesAll), neesAll = zeros(N_RUNS, numel(out.t)); end
    neesAll(i, :) = sum((e ./ sig).^2, 1);
    if i == 1, first = struct('t', out.t, 'e', e, 'sig', sig); end
    fprintf('run %2d/%d done\n', i, N_RUNS);
end

t = out.t;
use = t >= SETTLE;
neesMean = mean(neesAll, 1);
% 95% band for the mean of N chi-square(3) samples: chi2(3N)/N
band = 2*gammaincinv([0.025 0.975], 3*N_RUNS/2) / N_RUNS;
fracIn = mean(neesMean(use) >= band(1) & neesMean(use) <= band(2));
fprintf('\nAverage NEES (t >= %d s): %.2f (expected 3), 95%% band [%.2f, %.2f], %.0f%% of samples inside\n', ...
    SETTLE, mean(neesMean(use)), band(1), band(2), 100*fracIn);

figure('Name', 'MEKF Monte Carlo');
subplot(2, 1, 1);
plot(t, neesMean, t, band(1)*ones(size(t)), 'k--', t, band(2)*ones(size(t)), 'k--');
grid on; xlim([SETTLE t(end)]); ylabel('NEES');
title(sprintf('Average attitude NEES over %d runs (95%% band dashed)', N_RUNS));
subplot(2, 1, 2);
as = 3600*180/pi;
plot(first.t, as*first.e, '.', 'MarkerSize', 3); hold on;
set(gca, 'ColorOrderIndex', 1);
plot(first.t, 3*as*first.sig, '-');
set(gca, 'ColorOrderIndex', 1);
plot(first.t, -3*as*first.sig, '-');
grid on; xlim([SETTLE t(end)]); xlabel('time [s]'); ylabel('error [arcsec]');
legend('x', 'y', 'z', 'Location', 'eastoutside');
title('Run 1: attitude error and \pm3\sigma');
root = fileparts(fileparts(mfilename('fullpath')));
exportgraphics(gcf, fullfile(root, 'results', 'mekf_monte_carlo.png'), 'Resolution', 150);
