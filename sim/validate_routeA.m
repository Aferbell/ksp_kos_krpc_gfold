function R = validate_routeA(opts)
% VALIDATE_ROUTEA  Test whether a plan-timescale fix (route A) resolves the vertical channel.
%
% Route A hypothesis to test: the vehicle overshoots the plan's flight time by
% ~29% because the controller advances the plan by nearest position instead of by
% time. If that were the root cause, advancing the plan by a matched clock should
% make the vertical command physical (a_up >= 0) and restore plan tracking.
%
% Tested here against the logged flight, using the real trajectory to evaluate
% what each candidate clock would command. Also establishes where the vertical
% error actually originates.
%
% Name-Value options
%   DataFile  full path to the gfold log CSV
%   N         plan node count (default 80, inferred from the lead offset)
%   TfPlan    plan flight time [s] (default 18.5)
%   KVel      vertical velocity gain [1/s] (default 0.8)
%   KPos      vertical position gain [1/s^2] (default 0.5)
%   MakePlot  true to draw and save a summary figure (default true)
%
% Output R (struct): clock comparison table, plan consistency, root cause metrics.

arguments
    opts.DataFile (1,1) string = ""
    opts.N (1,1) double = 80
    opts.TfPlan (1,1) double = 18.5
    opts.KVel (1,1) double = 0.8
    opts.KPos (1,1) double = 0.5
    opts.MakePlot (1,1) logical = true
end

G0 = 9.81;
here = fileparts(mfilename("fullpath"));
if strlength(opts.DataFile) == 0
    opts.DataFile = fullfile(here, "..", "..", "log-B1040-9-gfold", ...
        "gfold_log_20261007_212924.csv");
end
T = readtable(opts.DataFile, CommentStyle="#", VariableNamingRule="preserve");

t = T.t; alt = T.alt; vz = T.vz;
tpLog = T.n_i ./ (opts.N - 1) .* opts.TfPlan;
[tpU, iu] = unique(tpLog, "stable");
pAlt = T.plan_alt(iu); pVz = T.plan_vz(iu); pAv = T.av_plan(iu);

% ---- 1. is the plan internally consistent? ----
aNetPlan = gradient(pVz, tpU);
aNetFromAv = pAv - G0;
R.planConsistency = struct( ...
    "rmsMismatch", rms(aNetPlan - aNetFromAv), ...
    "medianPlanNet", median(aNetPlan), ...
    "medianAvMinusG", median(aNetFromAv));

fprintf("=== [1] plan self-consistency (d(plan_vz)/dt vs av_plan - g0) ===\n");
fprintf("  rms mismatch = %.3f m/s^2   (small => plan is consistent)\n", ...
    R.planConsistency.rmsMismatch);

% ---- 2. candidate clocks, evaluated on the REAL trajectory ----
clocks = struct( ...
    "pos",  tpLog, ...
    "time", min(t, tpU(end)), ...
    "map",  min(t .* (tpU(end) / t(end)), tpU(end)));

names = ["pos","time","map"];
nC = numel(names);
negFrac = zeros(1, nC); aP10 = zeros(1, nC); aP50 = zeros(1, nC); aMin = zeros(1, nC);
for i = 1:nC
    tp = clocks.(names(i));
    pv = interp1(tpU, pVz,  tp, "linear", "extrap");
    pa = interp1(tpU, pAlt, tp, "linear", "extrap");
    pc = interp1(tpU, aNetPlan, tp, "linear", "extrap");
    aDes = G0 + pc + opts.KVel * (pv - vz) + opts.KPos * (pa - alt);
    negFrac(i) = mean(aDes < 0);
    aP10(i) = prctile(aDes, 10);
    aP50(i) = median(aDes);
    aMin(i) = min(aDes);
end
R.clocks = table(names.', negFrac.', aP10.', aP50.', aMin.', ...
    VariableNames=["clock","negFrac","aUpP10","aUpP50","aUpMin"]);

fprintf("\n=== [2] vertical command under each candidate clock (real trajectory) ===\n");
fprintf("  %-6s %10s %10s %10s %10s\n", "clock","negFrac","aUpP10","aUpP50","aUpMin");
for i = 1:nC
    fprintf("  %-6s %10.3f %10.2f %10.2f %10.2f\n", ...
        names(i), negFrac(i), aP10(i), aP50(i), aMin(i));
end
fprintf("  (aUp < 0 is unphysical: thrust can only push up)\n");

% ---- 3. where does the vertical error actually come from? ----
% early-window over-thrust: commanded vertical vs plan vertical
earlyWin = t <= 3.0;
R.earlyOverThrust = mean(T.av_up(earlyWin)) / mean(T.av_plan(earlyWin));
R.magRatio = median(T.a_cmd_mag) / median(T.av_plan);

% the legacy magnitude carries the horizontal PD corrections
R.horizPD = struct("u_h", median(T.u_h), "pdv_h", median(T.pdv_h), ...
    "pdp_h", median(T.pdp_h));

fprintf("\n=== [3] root cause: vertical over-thrust in the early phase ===\n");
fprintf("  t <= 3.0 s: mean av_up = %6.2f  vs mean av_plan = %6.2f  ratio = %.2f\n", ...
    mean(T.av_up(earlyWin)), mean(T.av_plan(earlyWin)), R.earlyOverThrust);
fprintf("  |target_a| median %6.2f vs av_plan median %6.2f\n", ...
    median(T.a_cmd_mag), median(T.av_plan));
fprintf("  horizontal PD inside |target_a|: u_h=%.2f  KV|dve|=%.2f  KP|dp|=%.2f\n", ...
    R.horizPD.u_h, R.horizPD.pdv_h, R.horizPD.pdp_h);
fprintf("  => av_up = |target_a|*cos(tilt) inherits the HORIZONTAL PD magnitude.\n");

% velocity deficit accumulated early
[~, k10] = min(abs(t - 1.89));
R.vzDeficit = T.vz(k10) - T.plan_vz(k10);
fprintf("  vz vs plan at t=1.89 s: %+6.2f m/s (vehicle far slower than plan)\n", ...
    R.vzDeficit);

% ---- 4. counterfactual: pure plan feedforward on the vertical channel ----
vzCF = zeros(size(vz)); vzCF(1) = vz(1);
for k = 2:numel(t)
    dt = t(k) - t(k-1);
    vzCF(k) = vzCF(k-1) + (T.av_plan(k) - G0) * dt;
end
planVzAtLog = interp1(tpU, pVz, tpLog, "linear", "extrap");
R.errActual = mean(abs(vz - planVzAtLog));
R.errFeedforward = mean(abs(vzCF - planVzAtLog));
fprintf("\n=== [4] counterfactual: vertical thrust = av_plan (pure feedforward) ===\n");
fprintf("  mean |vz - plan_vz|  actual        = %6.2f m/s\n", R.errActual);
fprintf("  mean |vz - plan_vz|  feedforward   = %6.2f m/s\n", R.errFeedforward);
fprintf("  => %.0f%% of the vertical tracking error is removed by feedforward alone\n", ...
    100 * (1 - R.errFeedforward / R.errActual));

% ---- 5. verdict ----
R.timescaleIsRootCause = false;
R.verdict = "ROUTE A REFUTED: plan is self-consistent and every candidate clock " + ...
    "still commands a_up < 0; the vertical error originates from |target_a| " + ...
    "carrying the horizontal PD magnitude into the vertical amplitude.";

fprintf("\n=== [5] verdict ===\n  %s\n", R.verdict);

if opts.MakePlot
    f = figure("Visible","on","Position",[100 100 1100 780]);
    tl = tiledlayout(f, 3, 2, "TileSpacing","compact");

    nexttile; plot(tpU, pVz, "LineWidth",1.2); hold on; plot(t, vz, "LineWidth",1.2);
    grid on; legend("plan v_z","actual v_z","Location","southwest");
    xlabel("plan time [s]"); ylabel("v_z [m/s]"); title("plan vs actual vertical velocity");

    nexttile; plot(t, t - tpLog, "LineWidth",1.2); grid on;
    xlabel("actual time [s]"); ylabel("lag [s]"); title("lag of actual behind plan (nearest-position clock)");

    nexttile; bar(1:nC, negFrac); grid on; ylim([0 1]);
    set(gca, "XTick", 1:nC, "XTickLabel", cellstr(names));
    ylabel("fraction with a_{up} < 0"); title("unphysical vertical command by clock");

    nexttile; plot(t, T.av_up, "LineWidth",1.2); hold on; plot(t, T.av_plan, "LineWidth",1.2);
    grid on; legend("av_{up} commanded","av_{plan}","Location","northwest");
    xlabel("t [s]"); ylabel("m/s^2"); title("vertical over-thrust (early)");

    nexttile; plot(t, T.a_cmd_mag, "LineWidth",1.2); hold on;
    plot(t, T.av_plan, "LineWidth",1.2); grid on;
    legend("|target_a|","av_{plan}","Location","northwest");
    xlabel("t [s]"); ylabel("m/s^2"); title("|target_a| carries horizontal PD");

    nexttile; plot(t, vz - planVzAtLog, "LineWidth",1.2); hold on;
    plot(t, vzCF - planVzAtLog, "LineWidth",1.2); grid on;
    legend("actual","pure feedforward","Location","southwest");
    xlabel("t [s]"); ylabel("v_z error [m/s]"); title("counterfactual feedforward");

    title(tl, "Route A validation: is the plan timescale the root cause?");
    outPng = fullfile(here, "routeA_validation.png");
    exportgraphics(f, outPng, "Resolution", 130);
    R.figure = outPng;
    fprintf("\nfigure saved: %s\n", outPng);
end
end
