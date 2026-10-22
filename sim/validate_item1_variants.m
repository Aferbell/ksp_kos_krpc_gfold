function R = validate_item1_variants(opts)
% VALIDATE_ITEM1_VARIANTS  Decide item 1 with the physical vertical clamp applied.
%
% Findings so far:
%   - The crosstalk is real: av_up = |target_a|*cos(tilt) carries the horizontal
%     PD magnitude, over-thrusting the early phase by 1.94x.
%   - A plain clamped PD (item1raw) removes the negative a_up but lands CLIMBING,
%     because the plan's terminal node spikes (av_plan = 43.8 where the profile
%     only needs about 23).
%
% This function tests each candidate WITH the physical clamp the flight code already
% has, a_ver_need = g0 + (vz^2 - V^2) / (2h), which is what keeps the vehicle
% descending. The question is whether item 1 then improves tracking without
% breaking the landing.
%
% Name-Value options
%   DataFile   full path to the gfold log CSV
%   N, TfPlan  plan node count and plan flight time (default 80 / 18.5)
%   TargetAlt  touchdown altitude [m] (default 17)
%   KVel, KPos vertical PD gains (default 0.8 / 0.5)
%   MakePlot   true to draw and save a figure (default true)
%
% Output R: per-law tracking, early error, landing vz, and a verdict.

arguments
    opts.DataFile (1,1) string = ""
    opts.N (1,1) double = 80
    opts.TfPlan (1,1) double = 18.5
    opts.TargetAlt (1,1) double = 17
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

P = struct("tp", tpU, "alt", pAlt, "vz", pVz, "av", pAv, ...
    "t", t, "avUp", T.av_up, ...
    "resid", gradient(vz, t) - (T.av_up - G0));

altGrid = linspace(min(alt), max(alt), 4000);
tpGrid  = interp1(pAlt, tpU, altGrid, "linear", "extrap");
pVzAtLog = interp1(tpU, pVz, tpLog, "linear", "extrap");

laws = ["logged","item1raw","item1need","feedfwd","feedfwdNeed"];
nL = numel(laws);
V = cell(1, nL); A = cell(1, nL);
for j = 1:nL
    [V{j}, A{j}] = flyLaw(P, altGrid, tpGrid, alt, vz, laws(j), ...
        opts.KVel, opts.KPos, opts.TargetAlt);
end

R = struct("laws", laws);
R.trackErr = zeros(1, nL); R.earlyErr = zeros(1, nL);
R.landVz = zeros(1, nL); R.minA = zeros(1, nL); R.earlyRatio = zeros(1, nL);
early = t <= 3.0;
for j = 1:nL
    vj = V{j}(:); aj = A{j}(:);
    nv = numel(vj);
    R.trackErr(j) = mean(abs(vj - pVzAtLog(1:nv)));
    ne = min(nv, sum(early));
    R.earlyErr(j) = mean(abs(vj(1:ne) - pVzAtLog(1:ne)));
    R.landVz(j) = vj(end);
    R.minA(j) = min(aj);
    R.earlyRatio(j) = mean(aj(1:ne)) / mean(pAv(early));
end

fprintf("=== plant validity ===\n");
fprintf("  real landing vz = %.2f m/s ; logged-model landing vz = %.2f\n", ...
    vz(end), R.landVz(1));

fprintf("\n=== closed-loop results (nearest-position clock) ===\n");
fprintf("  %-13s %12s %12s %10s %9s %11s\n", ...
    "law","mean|vz-plan|","early|err|","land_vz","min a_up","earlyRatio");
for j = 1:nL
    fprintf("  %-13s %12.2f %12.2f %10.2f %9.2f %11.2f\n", ...
        laws(j), R.trackErr(j), R.earlyErr(j), R.landVz(j), R.minA(j), ...
        R.earlyRatio(j));
end

% item 1 is accepted only if it beats the logged law on tracking AND lands descending
jI = find(laws == "item1need");
improves = R.trackErr(jI) < R.trackErr(1);
lands = R.landVz(jI) < 0 && R.landVz(jI) > -12;
R.pass = improves && lands;
if R.pass
    R.verdict = "ITEM 1 PASSES with the physical clamp: better tracking, still descending.";
elseif ~lands
    R.verdict = "ITEM 1 FAILS: lands while still moving upward (climbing).";
else
    R.verdict = "ITEM 1 FAILS: no tracking improvement.";
end
fprintf("\nVERDICT: %s\n", R.verdict);

if opts.MakePlot
    fg = figure("Visible","on","Position",[80 80 1200 560]);
    tl = tiledlayout(fg, 1, 2, "TileSpacing","compact");

    nexttile; hold on;
    plot(t, pVzAtLog, "k--", "LineWidth",1.6);
    leg = "plan";
    for j = 1:nL
        vj = V{j};
        plot(t(1:numel(vj)), vj, "LineWidth",1.1);
        leg(end+1) = laws(j); %#ok<AGROW> small fixed-size legend list
    end
    grid on; legend(leg, "Location","southwest");
    xlabel("t [s]"); ylabel("v_z [m/s]"); title("vertical velocity");

    nexttile; hold on;
    plot(P.tp, P.av, "k--", "LineWidth",1.6);
    leg2 = "plan";
    for j = 1:nL
        aj = A{j};
        plot(t(1:numel(aj)), aj, "LineWidth",1.1);
        leg2(end+1) = laws(j); %#ok<AGROW> small fixed-size legend list
    end
    grid on; legend(leg2, "Location","northwest");
    xlabel("t [s]"); ylabel("a_{up} [m/s^2]"); title("vertical command");

    title(tl, "item 1 variants with the physical vertical clamp");
    outPng = fullfile(here, "item1_variants.png");
    exportgraphics(fg, outPng, "Resolution", 130);
    R.figure = outPng;
    fprintf("figure saved: %s\n", outPng);
end
end

% -------------------------------------------------------------------------
function [V, A] = flyLaw(P, altGrid, tpGrid, alt, vz, law, KVel, KPos, tgtAlt)
% FLYLAW  Fly one vertical law against the plan, with the physical clamp available.
G0 = 9.81;
V = zeros(size(P.t)); A = zeros(size(P.t));
v = vz(1); h = alt(1);
n = numel(P.t);
tpEnd = P.tp(end);
% only the nearest-position clock is exercised here; the time-based clocks were
% already tested separately and were strictly worse (see the route A report).
iq = @(xq, yq, q) interp1(xq, yq, q, "linear", "extrap");

for k = 1:n
    V(k) = v;
    if k == n
        break
    end
    dt = P.t(k + 1) - P.t(k);

    tp = iq(altGrid, tpGrid, min(max(h, altGrid(1)), altGrid(end)));
    tp = min(max(tp, P.tp(1)), tpEnd);

    avPlan = iq(P.tp, P.av, tp);
    hRel = max(1.0, h - tgtAlt);
    need = G0 + (v * v) / (2 * hRel);      % a_ver_need, physical upper bound

    switch law
        case "logged"
            a = P.avUp(k);
        case "item1raw"
            a = avPlan + KVel * (iq(P.tp, P.vz, tp) - v) ...
                + KPos * (iq(P.tp, P.alt, tp) - h);
            a = max(a, 0);
        case "item1need"
            a = avPlan + KVel * (iq(P.tp, P.vz, tp) - v) ...
                + KPos * (iq(P.tp, P.alt, tp) - h);
            a = max(a, 0);
            a = min(a, need);
        case "feedfwd"
            a = avPlan;
        otherwise   % "feedfwdNeed"
            a = min(avPlan, need);
    end
    A(k) = a;

    v = v + (a - G0 + iq(P.t, P.resid, P.t(k))) * dt;
    h = h + v * dt;
    if h <= 0
        V = V(1:k); A = A(1:k);
        break
    end
end
end
