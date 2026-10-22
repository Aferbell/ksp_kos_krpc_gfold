function R = validate_item1_closedloop(opts)
% VALIDATE_ITEM1_CLOSEDLOOP  Closed-loop vertical check for the item-1 crosstalk fix.
%
% The open-loop screen showed the clamped item-1 law commands a physical a_up and
% cuts the early over-thrust. This function closes the loop to test whether it
% actually TRACKS the plan's vertical profile, which is the point of the fix.
%
% Controllers flown from the logged initial state against the same plan:
%   "logged"   the logged av_up trace (what really flew) - plant-validity control
%   "item1"    av_plan + clamped PD, a_up in [lo,hi]*av_plan and >= 0
%   "feedfwd"  a_up = av_plan exactly (pure feedforward)
%
% Drag is carried as the residual measured from the flight, so the plant is not
% invented: aNet = a_up - g0 + resid. Validity is evidenced by the "logged" row
% reproducing the flown landing velocity.
%
% All three candidate plan clocks are exercised, because in 1D the
% nearest-position clock can stall - which is itself a finding.
%
% Name-Value options
%   DataFile   full path to the gfold log CSV
%   N, TfPlan  plan node count and plan flight time (default 80 / 18.5)
%   KVel, KPos vertical PD gains (default 0.8 / 0.5)
%   FloScl     [lo hi] multiples of av_plan for the item-1 clamp (default [0.5 1.5])
%   MakePlot   true to draw and save a figure (default true)
%
% Output R: tracking metrics per (clock, controller) and a verdict.

arguments
    opts.DataFile (1,1) string = ""
    opts.N (1,1) double = 80
    opts.TfPlan (1,1) double = 18.5
    opts.KVel (1,1) double = 0.8
    opts.KPos (1,1) double = 0.5
    opts.FloScl (1,2) double = [0.5 1.5]
    opts.MakePlot (1,1) logical = true
end

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

profile = struct( ...
    "tp", tpU, "alt", pAlt, "vz", pVz, "av", pAv, ...
    "t", t, "resid", gradient(vz, t) - (T.av_up - 9.81), ...
    "avUp", T.av_up);

altGrid = linspace(min(alt), max(alt), 4000);
tpGrid  = interp1(pAlt, tpU, altGrid, "linear", "extrap");

clocks = ["pos","time","map"];
laws   = ["logged","item1","feedfwd"];
pVzAtLog = interp1(tpU, pVz, tpLog, "linear", "extrap");

nC = numel(clocks); nL = numel(laws);
V = cell(nC, nL); A = cell(nC, nL);
for i = 1:nC
    for j = 1:nL
        [V{i,j}, A{i,j}] = flyVertical(profile, altGrid, tpGrid, alt, vz, ...
            clocks(i), laws(j), opts.KVel, opts.KPos, opts.FloScl);
    end
end

fprintf("=== plant validity (the logged row must reproduce the flight) ===\n");
fprintf("  real landing vz = %.2f m/s\n", vz(end));
for i = 1:nC
    fprintf("  clock %-5s logged land vz = %7.2f\n", clocks(i), V{i,1}(end));
end

R = struct("clocks", clocks, "laws", laws);
R.trackErr = zeros(nC, nL); R.earlyErr = zeros(nC, nL); R.landVz = zeros(nC, nL);
early = t <= 3.0;
for i = 1:nC
    for j = 1:nL
        vj = V{i,j}(:);
        nv = numel(vj);
        R.trackErr(i,j) = mean(abs(vj - pVzAtLog(1:nv)));
        ne = min(nv, sum(early));
        R.earlyErr(i,j) = mean(abs(vj(1:ne) - pVzAtLog(1:ne)));
        R.landVz(i,j) = vj(end);
    end
end

fprintf("\n=== closed-loop vertical tracking, mean|vz - plan_vz| (m/s) ===\n");
fprintf("  %-7s %10s %10s %10s\n", "clock","logged","item1","feedfwd");
for i = 1:nC
    fprintf("  %-7s %10.2f %10.2f %10.2f\n", clocks(i), ...
        R.trackErr(i,1), R.trackErr(i,2), R.trackErr(i,3));
end

fprintf("\n=== early phase (t <= 3 s), mean|vz - plan_vz| (m/s) ===\n");
fprintf("  %-7s %10s %10s %10s\n", "clock","logged","item1","feedfwd");
for i = 1:nC
    fprintf("  %-7s %10.2f %10.2f %10.2f\n", clocks(i), ...
        R.earlyErr(i,1), R.earlyErr(i,2), R.earlyErr(i,3));
end

fprintf("\n=== landing vertical velocity (m/s); safe is roughly -2 .. -6 ===\n");
fprintf("  %-7s %10s %10s %10s\n", "clock","logged","item1","feedfwd");
for i = 1:nC
    fprintf("  %-7s %10.2f %10.2f %10.2f\n", clocks(i), ...
        R.landVz(i,1), R.landVz(i,2), R.landVz(i,3));
end

R.earlyRatio = zeros(nC, nL);
for i = 1:nC
    for j = 1:nL
        aj = A{i,j}(:);
        ne = min(numel(aj), sum(early));
        R.earlyRatio(i,j) = mean(aj(1:ne)) / mean(pAv(early));
    end
end
fprintf("\n=== early av_up / av_plan ratio (1.0 is ideal) ===\n");
fprintf("  %-7s %10s %10s %10s\n", "clock","logged","item1","feedfwd");
for i = 1:nC
    fprintf("  %-7s %10.2f %10.2f %10.2f\n", clocks(i), ...
        R.earlyRatio(i,1), R.earlyRatio(i,2), R.earlyRatio(i,3));
end

iPos = find(clocks == "pos");
improves = R.trackErr(iPos,2) < R.trackErr(iPos,1);
lands = R.landVz(iPos,2) > -8 && R.landVz(iPos,2) < 0;
R.pass = improves && lands;
if R.pass
    R.verdict = "ITEM 1 PASSES: improves tracking and still lands descending.";
elseif improves
    R.verdict = "ITEM 1 PARTIAL: improves tracking but the landing vz is unsafe.";
else
    R.verdict = "ITEM 1 FAILS: does not improve vertical tracking.";
end
fprintf("\nVERDICT: %s\n", R.verdict);

if opts.MakePlot
    fg = figure("Visible","on","Position",[80 80 1200 560]);
    tl = tiledlayout(fg, 1, 2, "TileSpacing","compact");

    nexttile; hold on;
    plot(t, pVzAtLog, "k--", "LineWidth",1.5);
    plot(t, V{iPos,1}, "-", "LineWidth",1.2);
    plot(t, V{iPos,2}, "-", "LineWidth",1.2);
    plot(t, V{iPos,3}, "-", "LineWidth",1.2);
    grid on; legend(["plan","logged","item1","feedfwd"], "Location","southwest");
    xlabel("t [s]"); ylabel("v_z [m/s]"); title("vertical velocity (nearest-pos clock)");

    nexttile; hold on;
    plot(t, pAv, "k--", "LineWidth",1.5);
    plot(t, A{iPos,1}, "-", "LineWidth",1.2);
    plot(t, A{iPos,2}, "-", "LineWidth",1.2);
    plot(t, A{iPos,3}, "-", "LineWidth",1.2);
    grid on; legend(["plan","logged","item1","feedfwd"], "Location","northwest");
    xlabel("t [s]"); ylabel("a_{up} [m/s^2]"); title("vertical command");

    title(tl, "item 1 closed-loop validation");
    outPng = fullfile(here, "item1_closedloop.png");
    exportgraphics(fg, outPng, "Resolution", 130);
    R.figure = outPng;
    fprintf("figure saved: %s\n", outPng);
end
end

% -------------------------------------------------------------------------
function [V, A] = flyVertical(profile, altGrid, tpGrid, alt, vz, clockMode, ...
    law, KVel, KPos, floScl)
% FLYVERTICAL  Fly one vertical controller against the plan profile.
G0 = 9.81;
V = zeros(size(profile.t)); A = zeros(size(profile.t));
v = vz(1); h = alt(1);
n = numel(profile.t);
tpEnd = profile.tp(end);
tEnd = profile.t(end);

interpOn = @(xq, yq, q) interp1(xq, yq, q, "linear", "extrap");

for k = 1:n
    V(k) = v;
    if k == n
        break
    end
    dt = profile.t(k + 1) - profile.t(k);

    switch clockMode
        case "pos"
            tp = interpOn(altGrid, tpGrid, ...
                min(max(h, altGrid(1)), altGrid(end)));
            tp = min(max(tp, profile.tp(1)), tpEnd);
        case "time"
            tp = min(profile.t(k), tpEnd);
        otherwise   % "map"
            tp = min(profile.t(k) * (tpEnd / tEnd), tpEnd);
    end

    avPlan = interpOn(profile.tp, profile.av, tp);
    switch law
        case "logged"
            a = profile.avUp(k);
        case "item1"
            raw = avPlan ...
                + KVel * (interpOn(profile.tp, profile.vz, tp) - v) ...
                + KPos * (interpOn(profile.tp, profile.alt, tp) - h);
            a = min(max(raw, floScl(1) * avPlan), floScl(2) * avPlan);
            a = max(a, 0);
        otherwise   % "feedfwd"
            a = avPlan;
    end
    A(k) = a;

    aNet = a - G0 + interpOn(profile.t, profile.resid, profile.t(k));
    v = v + aNet * dt;
    h = h + v * dt;
    if h <= 0
        V = V(1:k); A = A(1:k);
        break
    end
end
end
