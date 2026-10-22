function R = crosscheck_feedfwd(opts)
% CROSSCHECK_FEEDFWD  Re-check the item-1 winner across several logged flights.
%
% The item-1 closed-loop test on the 212924 log found that capping the plan's
% vertical command by the physical requirement, a_up = min(av_plan, a_ver_need),
% tracked better than the flown law while still landing descending. A single log is
% weak evidence, so this function repeats the comparison on every available flight.
%
% The plant is the same residual-carrying model used before (drag taken from the
% logged force balance), and each flight's own plan profile is rebuilt from its
% logged index. The "logged" row doubles as the plant-validity control: it must
% reproduce that flight's recorded landing velocity.
%
% Name-Value options
%   LogDir    folder holding the gfold CSVs
%   Pattern   file glob for the logs
%   TfPlan    plan flight time [s] (default 18.5)
%   TargetAlt touchdown altitude [m] (default 17)
%   MakePlot  true to draw and save a figure (default true)
%
% Output R: a table of per-flight tracking error and landing velocity.

arguments
    opts.LogDir (1,1) string = ""
    opts.Pattern (1,1) string = "gfold_log_*.csv"
    opts.TfPlan (1,1) double = 18.5
    opts.TargetAlt (1,1) double = 17
    opts.MakePlot (1,1) logical = true
end

G0 = 9.81;
here = fileparts(mfilename("fullpath"));
if strlength(opts.LogDir) == 0
    opts.LogDir = fullfile(here, "..", "..", "log-B1040-9-gfold");
end
listing = dir(fullfile(opts.LogDir, opts.Pattern));
names = strings(0,1); Ns = []; eLog = []; eFF = []; lzLog = []; lzFF = [];
okFF = logical([]);

for i = 1:numel(listing)
    fp = fullfile(listing(i).folder, listing(i).name);
    try
        T = readtable(fp, CommentStyle="#", VariableNamingRule="preserve");
    catch
        continue
    end
    need = ["t","alt","vz","n_i","plan_alt","plan_vz","av_plan","av_up","tf_lead_n"];
    if ~all(ismember(need, string(T.Properties.VariableNames)))
        continue
    end
    [e1, e2, l1, l2, N] = compareOne(T, opts.TfPlan, opts.TargetAlt, G0);
    if isnan(e1)
        continue
    end
    names(end+1,1) = string(listing(i).name); %#ok<AGROW> small fixed list
    Ns(end+1,1) = N; %#ok<AGROW>
    eLog(end+1,1) = e1; %#ok<AGROW>
    eFF(end+1,1) = e2; %#ok<AGROW>
    lzLog(end+1,1) = l1; %#ok<AGROW>
    lzFF(end+1,1) = l2; %#ok<AGROW>
    okFF(end+1,1) = l2 < 0 && l2 > -12; %#ok<AGROW>
end

R = table(names, Ns, eLog, eFF, lzLog, lzFF, okFF, VariableNames= ...
    ["log","N","errLogged","errFeedfwdNeed","landVzLogged", ...
     "landVzFeedfwdNeed","safeLanding"]);

fprintf("=== cross-flight check of a_up = min(av_plan, a_ver_need) ===\n");
fprintf("  %-30s %5s %10s %10s %12s %14s %7s\n", ...
    "log","N","errLogged","errFFneed","lzLogged","lzFFneed","safe");
for i = 1:height(R)
    fprintf("  %-30s %5d %10.2f %10.2f %12.2f %14.2f %7d\n", ...
        R.log(i), R.N(i), R.errLogged(i), R.errFeedfwdNeed(i), ...
        R.landVzLogged(i), R.landVzFeedfwdNeed(i), R.safeLanding(i));
end

better = sum(R.errFeedfwdNeed < R.errLogged);
fprintf("\n  tracking better than flown in %d/%d flights\n", better, height(R));
fprintf("  safe descents: %d/%d\n", sum(R.safeLanding), height(R));
% plant validity: the logged row should match each flight's recorded landing vz
fprintf("  (plant validity: the logged row must equal each flight's recorded landing vz)\n");
% Expose the headline counts as custom properties so callers need not parse the table.
R = addprop(R, "betterCount", "table");
R.Properties.CustomProperties.betterCount = better;
R = addprop(R, "safeCount", "table");
R.Properties.CustomProperties.safeCount = sum(R.safeLanding);
R = addprop(R, "figurePath", "table");
R.Properties.CustomProperties.figurePath = "";

if opts.MakePlot && height(R) > 0
    fg = figure("Visible","on","Position",[80 80 1000 420]);
    tl = tiledlayout(fg, 1, 2, "TileSpacing","compact");
    nexttile; bar([R.errLogged, R.errFeedfwdNeed]); grid on;
    set(gca, "XTick", 1:height(R), "XTickLabel", cellstr(R.log));
    ylabel("mean |v_z - plan v_z| [m/s]");
    legend("flown","min(av_{plan}, a_{ver,need})", "Location","best");
    title("tracking error by flight");
    nexttile; bar([R.landVzLogged, R.landVzFeedfwdNeed]); grid on;
    yline(0,"k--");
    set(gca, "XTick", 1:height(R), "XTickLabel", cellstr(R.log));
    ylabel("landing v_z [m/s]");
    legend("flown","min(av_{plan}, a_{ver,need})", "Location","best");
    title("landing vertical velocity");
    title(tl, "cross-flight check of the item-1 candidate");
    outPng = fullfile(here, "crosscheck_feedfwd.png");
    exportgraphics(fg, outPng, "Resolution", 130);
    R.Properties.CustomProperties.figurePath = outPng;
    fprintf("figure saved: %s\n", outPng);
end
end

% -------------------------------------------------------------------------
function [e1, e2, l1, l2, N] = compareOne(T, tfPlan, tgtAlt, G0)
% COMPAREONE  Rebuild one flight's plan and compare the flown law with the candidate.
e1 = NaN; e2 = NaN; l1 = NaN; l2 = NaN; N = NaN;

t = T.t; alt = T.alt; vz = T.vz;
nl = T.tf_lead_n; nl = nl(isfinite(nl));
if isempty(nl) || ~any(isfinite(T.plan_vz))
    return
end
N = round(1 + max(nl) * tfPlan / 1.5);
if N < 10
    return
end

tpLog = T.n_i ./ (N - 1) .* tfPlan;
[tpU, iu] = unique(tpLog, "stable");
if numel(tpU) < 5
    return
end
pAlt = T.plan_alt(iu); pVz = T.plan_vz(iu); pAv = T.av_plan(iu);

aNetLog = gradient(vz, t);
resid = aNetLog - (T.av_up - G0);
pVzAtLog = interp1(tpU, pVz, tpLog, "linear", "extrap");
altGrid = linspace(min(alt), max(alt), 4000);
tpGrid  = interp1(pAlt, tpU, altGrid, "linear", "extrap");
iq = @(xq, yq, q) interp1(xq, yq, q, "linear", "extrap");

n = numel(t);
V1 = zeros(n,1); V2 = zeros(n,1);
v1 = vz(1); h1 = alt(1);
v2 = vz(1); h2 = alt(1);
for k = 1:n
    V1(k) = v1; V2(k) = v2;
    if k == n
        break
    end
    dt = t(k+1) - t(k);
    rk = iq(t, resid, t(k));
    a1 = T.av_up(k);
    v1 = v1 + (a1 - G0 + rk) * dt;  h1 = h1 + v1 * dt;

    tp = iq(altGrid, tpGrid, min(max(h2, altGrid(1)), altGrid(end)));
    tp = min(max(tp, tpU(1)), tpU(end));
    avp = iq(tpU, pAv, tp);
    need = G0 + (v2 * v2) / (2 * max(1.0, h2 - tgtAlt));
    a2 = min(avp, need);
    v2 = v2 + (a2 - G0 + rk) * dt;  h2 = h2 + v2 * dt;
end
e1 = mean(abs(V1 - pVzAtLog));
e2 = mean(abs(V2 - pVzAtLog));
l1 = V1(end); l2 = V2(end);
end
