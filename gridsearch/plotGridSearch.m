function T = plotGridSearch(csvFile)
%PLOTGRIDSEARCH Plot the results of gridsearch/grid_search.
%   T = plotGridSearch('results/example_grid.csv')
%
%   One point per grid point that built and ran (status "ok"):
%     x = estimated latency of one controller() call [us] (BALANCED profile,
%         estCyclesBalanced * 10 ns; an estimate, not a synthesis result),
%     y = mean tracking RMSE over the runs.
%   Blue: no run violated a constraint. Red: at least one run left
%   [yMin, yMax] or exceeded deltaY. The black line is the Pareto front of
%   the violation-free points: lowest RMSE for a given latency budget.
%   Hover over a point (data tips) to see its hyperparameters.
%
%   Returns the table of all grid points, failed ones included.

    % status and message as text even when every message is empty
    % (readtable would otherwise read an all-empty column as NaN)
    opts = detectImportOptions(csvFile, 'Delimiter', ',', 'VariableNamingRule', 'preserve');
    opts = setvartype(opts, {'status', 'message'}, 'string');
    T = readtable(csvFile, opts);
    ok = T.status == "ok";
    fprintf('%d grid points, %d ok, %d failed\n', height(T), nnz(ok), nnz(~ok));
    for k = find(~ok)'
        fprintf('  point %d: %s %s\n', T.point(k), T.status(k), T.message(k));
    end
    if ~any(ok), return; end

    % Hyperparameter columns: between "point" and "status"
    names  = T.Properties.VariableNames;
    hpCols = names(2 : find(strcmp(names, 'status')) - 1);

    R = T(ok, :);
    clean = R.runsViolY == 0 & R.runsViolDy == 0;

    figure('Name', csvFile);
    hold on; grid on;
    h1 = scatter(R.estLatencyUs(clean),  R.meanRmse(clean),  50, 'b', 'filled');
    h2 = scatter(R.estLatencyUs(~clean), R.meanRmse(~clean), 50, 'r', 'filled');
    addTips(h1, R(clean, :),  hpCols);
    addTips(h2, R(~clean, :), hpCols);

    % Pareto front of the violation-free points: sorted by latency, keep a
    % point only if its RMSE is below every faster point's.
    C = sortrows(R(clean, :), {'estLatencyUs', 'meanRmse'});
    front = false(height(C), 1);
    best = inf;
    for k = 1:height(C)
        if C.meanRmse(k) < best
            front(k) = true;
            best = C.meanRmse(k);
        end
    end
    if any(front)
        plot(C.estLatencyUs(front), C.meanRmse(front), 'k-', 'LineWidth', 1);
    end

    % Latency limit: the sampling time (the controller must finish within Ts)
    Ts = R.Ts(1);
    if Ts > 0 && Ts * 1e6 < 2 * max(R.estLatencyUs)
        xline(Ts * 1e6, '--', 'T_s');
    end

    xlabel('estimated latency per call [\mus] (BALANCED)');
    ylabel('mean RMSE over the runs');
    legend([h1 h2], {'no violations', 'violations'}, 'Location', 'best');
    title(sprintf('%s: %d grid points', csvFile, height(R)), 'Interpreter', 'none');
end

function addTips(h, R, hpCols)
% Data tips with the hyperparameter values of each point.
    if isempty(R), return; end
    for c = 1:numel(hpCols)
        h.DataTipTemplate.DataTipRows(end + 1) = dataTipTextRow(hpCols{c}, R.(hpCols{c}));
    end
    h.DataTipTemplate.DataTipRows(end + 1) = dataTipTextRow('runs with violations (y)', R.runsViolY);
    h.DataTipTemplate.DataTipRows(end + 1) = dataTipTextRow('runs with violations (dy)', R.runsViolDy);
end
