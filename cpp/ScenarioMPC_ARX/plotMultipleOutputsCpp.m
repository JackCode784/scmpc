%% Plot multi-run results of testMain.cpp (N_RUNS > 1 in plant.h)
% Each experiment (a build of testMain.cpp) writes
%   runs_<tag>_<run>.txt      one per run, same columns as output.txt
%   runs_<tag>_summary.txt    one row per run: true plant parameters and
%                             constraint-violation metrics
% where <tag> = <ctrl>_<plant>, e.g. 'scen_nl' or 'noscen_nl'.
%
% opts.isVitisSim : true -> read from the Vitis csim build folder
% opts.tags       : (optional) cell array of tags to plot, e.g.
%                   {'scen_nl', 'noscen_nl'}; default: every summary found
%
% For every tag: all runs overlaid (output, Delta y, input); runs that
% violate a constraint are drawn in red. With more than one tag: a paired
% comparison - the test bench uses the same seed for every build, so run i
% is the SAME true plant in every tag.
% A single run in detail: plotOutputCpp with opts.fileName.
function plotMultipleOutputsCpp(opts)
clc;

if opts.isVitisSim
    d = "..\..\vitis\scmpc-arx\solution1\csim\build\";
else
    d = pwd;
end

if isfield(opts, 'tags') && ~isempty(opts.tags)
    tags = string(opts.tags);
else
    s = dir(fullfile(d, 'runs_*_summary.txt'));
    tags = erase(erase(string({s.name}), "runs_"), "_summary.txt");
end
if isempty(tags)
    error('No runs_*_summary.txt found in %s', d);
end

nTags = numel(tags);
summaries = cell(nTags, 1);
okColor  = [0 0.4470 0.7410 0.35];   % RGBA: semi-transparent blue
badColor = [0.8500 0.1 0.0980 0.8];  % red

for t = 1:nTags
    tag = tags(t);
    S = readtable(fullfile(d, "runs_" + tag + "_summary.txt"), VariableNamingRule="preserve");
    summaries{t} = S;
    violating = S.nViolY > 0 | S.nViolDy > 0;

    figure(Name="Runs: " + tag);
    for r = 1:height(S)
        % File names from the summary (not a directory listing), so stale
        % files from an earlier experiment with more runs are never read.
        f = fullfile(d, sprintf('runs_%s_%03d.txt', tag, S.run(r)));
        data = readtable(f, VariableNamingRule="preserve");
        c = okColor; if violating(r), c = badColor; end

        subplot(3,1,1); hold on;
        plot(data.ySim, Color=c, LineWidth=1);
        subplot(3,1,2); hold on;
        plot(diff(data.ySim), Color=c, LineWidth=1);
        subplot(3,1,3); hold on;
        plot(data.uSim, Color=c, LineWidth=1);
    end

    % Reference and bounds (identical in every run)
    subplot(3,1,1);
    plot(data.yref, 'k--', LineWidth=1.5);
    yline(data.yMin(1), 'k:', num2str(data.yMin(1)), LineWidth=1.5);
    yline(data.yMax(1), 'k:', num2str(data.yMax(1)), LineWidth=1.5);
    hold off; grid on; xlabel('k'); ylabel('y_k');
    title(sprintf('%s: %d runs, %d leave [y_{min}, y_{max}], %d exceed \\Deltay (red: any violation)', ...
        strrep(tag, '_', ' '), height(S), sum(S.nViolY > 0), sum(S.nViolDy > 0)), Interpreter="tex");

    subplot(3,1,2);
    yline(data.deltaY(1)*[-1, 1], 'k:', {num2str(-data.deltaY(1)), num2str(data.deltaY(1))}, LineWidth=1.5);
    hold off; grid on; xlabel('k'); ylabel('y_k - y_{k-1}'); title('\Deltay');

    subplot(3,1,3);
    plot(data.uMax, 'k:', LineWidth=1.5);
    plot(data.uMin, 'k:', LineWidth=1.5);
    hold off; grid on; xlabel('k'); ylabel('u^*_k'); title('Applied input');
end

% Paired comparison across experiments (same run index = same true plant)
if nTags > 1
    figure(Name="Comparison");
    counts = zeros(nTags, 2);
    for t = 1:nTags
        counts(t,:) = [sum(summaries{t}.nViolY > 0), sum(summaries{t}.nViolDy > 0)];
    end
    subplot(3,1,1);
    bar(categorical(strrep(tags, '_', ' ')), counts);
    legend('y outside [y_{m}, y_{M}]', '|\Deltay| > \Deltay_{M}', Location="best");
    ylabel('Runs with violations'); grid on;
    title(sprintf('%d runs per experiment', height(summaries{1})));

    metrics = ["maxViolY", "maxViolDy"];
    labels  = ["Largest excursion outside [y_{m}, y_{M}]", "Largest excess of |\Deltay| over \Deltay_{M}"];
    for m = 1:2
        subplot(3,1,m+1); hold on;
        for t = 1:nTags
            plot(summaries{t}.run, summaries{t}.(metrics(m)), 'o-', LineWidth=1.2, DisplayName=strrep(tags(t), '_', ' '));
        end
        hold off; grid on; legend(Location="best");
        xlabel('Run (same true plant in every experiment)'); ylabel(labels(m));
    end
end

end
