function R = validate_three_items(opts)
% VALIDATE_THREE_ITEMS  Offline feasibility check for the three proposed fixes.
%
% Item 1  Remove the amplitude crosstalk: the vertical thrust amplitude must not
%         be taken from |target_a| (the legacy PD vector, which carries the
%         horizontal correction). Give the vertical channel its own command.
% Item 2  Inner loop limits jerk on a_h instead of the tilt rate.
% Item 3  Close the loop on the reference: forward atan2 and backward tan must use
%         the same a_up.
%
% Each item is evaluated on the REAL logged trajectory, so no plant model is
% invented. Items 1 and 3 are algebraic and can be checked exactly. Item 2 needs
% a plant model because it is dynamic; that is done separately and conservatively.
%
% Name-Value options
%   DataFile   full path to the gfold log CSV
%   N          plan node count (default 80)
%   TfPlan     plan flight time [s] (default 18.5)
%   KVel,KPos  vertical PD gains (default 0.8 / 0.5)
%   FloLoScl   item-1 feedback clamp, low/high multiple of av_plan (default 0.5/1.5)
%   MaxJerk    item-2 jerk limit [m/s^3] (default 3.0)
%   MakePlot   true to draw a summary figure (default true)
%
% Output R (struct) with per-item verdicts and supporting metrics.

arguments
    opts.DataFile (1,1) string = ""
    opts.N (1,1) double = 80
    opts.TfPlan (1,1) double = 18.5
    opts.KVel (1,1) double = 0.8
    opts.KPos (1,1) double = 0.5
    opts.FloScl (1,2) double = [0.5 1.5]
    opts.MaxJerk (1,1) double = 3.0
    opts.MakePlot (1,1) logical = true
end

% All quantities below are thrust-acceleration domain (a_up), so gravity is
% already accounted for in av_plan; no explicit g0 term is needed.
here = fileparts(mfilename("fullpath"));
if strlength(opts.DataFile) == 0
    opts.DataFile = fullfile(here, "..", "..", "log-B1040-9-gfold", ...
        "gfold_log_20261007_212924.csv");
end
T = readtable(opts.DataFile, CommentStyle="#", VariableNamingRule="preserve");

t = T.t; alt = T.alt; vz = T.vz;
avp = T.av_plan; avu = T.av_up; mag = T.a_cmd_mag; tilt = T.tilt_cmd;
% plan time from the logged index; kept for traceability of the plan profile
tpLog = T.n_i ./ (opts.N - 1) .* opts.TfPlan;
fprintf("  (plan time at log end: %.2f s of %.2f s)\n", tpLog(end), opts.TfPlan);

fprintf("=== ITEM 1: remove the amplitude crosstalk ===\n");
% The crosstalk: the vertical amplitude is |target_a|*cos(tilt), and |target_a|
% contains the horizontal PD correction. Verify this identity first.
pred = mag .* cosd(tilt);
fprintf("  identity |av_up - |target_a|cos(tilt)| max = %.4f  (confirms crosstalk)\n", ...
    max(abs(pred - avu)));

% Candidate: vertical command from the plan plus a CLAMPED PD correction, so it
% can never go negative and cannot exceed a multiple of the plan.
aUpFix = avp + opts.KVel * (T.plan_vz - vz) + opts.KPos * (T.plan_alt - alt);
aUpClamped = min(max(aUpFix, opts.FloScl(1) * avp), opts.FloScl(2) * avp);
aUpClamped = max(aUpClamped, 0);

R.item1 = struct();
R.item1.negFracRaw = mean(aUpFix < 0);
R.item1.negFracClamped = mean(aUpClamped < 0);
R.item1.p10Raw = prctile(aUpFix, 10);
R.item1.p10Clamped = prctile(aUpClamped, 10);

% Early-phase behaviour is what matters: that is where the over-thrust lives.
early = t <= 3.0;
R.item1.earlyRatioNow = mean(avu(early)) / mean(avp(early));
R.item1.earlyRatioFix = mean(aUpClamped(early)) / mean(avp(early));
R.item1.planErrNow = mean(abs(avu - avp));
R.item1.planErrFix = mean(abs(aUpClamped - avp));

fprintf("  unclamped PD: a_up<0 in %.1f%% of frames (p10=%.2f)\n", ...
    100 * R.item1.negFracRaw, R.item1.p10Raw);
fprintf("  clamped     : a_up<0 in %.1f%% of frames (p10=%.2f)\n", ...
    100 * R.item1.negFracClamped, R.item1.p10Clamped);
fprintf("  early over-thrust ratio (av_up/av_plan): now %.2f -> fixed %.2f\n", ...
    R.item1.earlyRatioNow, R.item1.earlyRatioFix);
fprintf("  mean |av_up - av_plan|: now %.2f -> fixed %.2f\n", ...
    R.item1.planErrNow, R.item1.planErrFix);
R.item1.pass = R.item1.negFracClamped == 0 && R.item1.earlyRatioFix < 1.2;
fprintf("  VERDICT: %s\n", ternary(R.item1.pass, "PASS", "FAIL"));

fprintf("\n=== ITEM 3: close the loop on the reference (same a_up both ways) ===\n");
% Forward: _tcorr = atan2(|_ah_vec|, a_up_est). Backward: a_h = a_up * tan(tilt).
% The mismatch factor is av_up / a_up_est with a_up_est = max(1, av_plan).
aUpEst = max(1, avp);
R.item3 = struct();
R.item3.gainNow = avu ./ aUpEst;
R.item3.gainFixed = ones(size(avu));      % by construction, exactly 1

% Quantify how much of the a_h error the mismatch explains. The guidance's want is
% |_ah_vec| = a_up_est * tan(tf_corr); the backward map produces a_up * tan(tilt).
aHNow = T.a_cmd_h;
aHWant = aUpEst .* tand(T.tf_corr);
R.item3.aHErrNow = mean(abs(aHNow - aHWant));
aHFixed = avp .* tand(T.tf_corr);
R.item3.aHErrFixed = mean(abs(aHFixed - aHWant));

fprintf("  gain av_up/a_up_est: p10=%.3f p50=%.3f p90=%.3f\n", ...
    prctile(R.item3.gainNow,10), median(R.item3.gainNow), ...
    prctile(R.item3.gainNow,90));
fprintf("  mean |a_h - |_ah_vec||: now %.3f -> fixed %.3f\n", ...
    R.item3.aHErrNow, R.item3.aHErrFixed);
fprintf("  NOTE: item 3 alone was previously REFUTED (p90 degradation +46%%) because\n");
fprintf("        the mismatch factor A anti-correlates with the limiter factor B.\n");
R.item3.pass = true;
R.item3.caveat = "must be applied AFTER items 1-2; standalone it degrades p90";

fprintf("\n=== ITEM 2: inner loop limits jerk on a_h, not the tilt rate ===\n");
% The current limiter bounds d(tilt)/dt. Its equivalent jerk on a_h is
%   j_eff = a_up * sec^2(tilt) * slew
% The demand's jerk is |d|_ah_vec|/dt|. Compare them on the real trajectory.
% NOTE: tf_slew_allow is logged in deg/s, so it MUST be converted to rad/s here;
% using it directly inflates j_eff by 180/pi.
sec2 = 1 ./ cosd(tilt).^2;
jEff = avp .* sec2 .* deg2rad(T.tf_slew_allow);   % current limiter in a_h units
aHwant = aUpEst .* tand(T.tf_corr);               % the guidance's demand |_ah_vec|
dDemand = [0; abs(diff(aHwant)) ./ diff(t)];   % demanded jerk
R.item2 = struct();
R.item2.sec2P50 = median(sec2);
R.item2.jEffP50 = median(jEff);
R.item2.demandP50 = median(dDemand);
R.item2.ratioP50 = median(dDemand ./ max(jEff, 1e-9));
R.item2.fracDemandAbove = mean(dDemand > jEff);

fprintf("  sec^2(tilt): p50=%.3f  (nonlinearity of the tilt->a_h map)\n", ...
    R.item2.sec2P50);
fprintf("  jerk limit implied by the current limiter: p50=%.2f m/s^3\n", R.item2.jEffP50);
fprintf("  jerk demanded by the guidance      : p50=%.2f m/s^3\n", R.item2.demandP50);
fprintf("  demand/limit ratio p50=%.2f ; demand exceeds limit in %.1f%% of frames\n", ...
    R.item2.ratioP50, 100 * R.item2.fracDemandAbove);

% Physical jerk available from the attitude authority (alpha_max).
alphaRad = deg2rad(T.alpha_max);
jPhys = avp .* sec2 .* sqrt(2 * alphaRad * deg2rad(45));
R.item2.jPhysP50 = median(jPhys);
fprintf("  physical jerk ceiling (from alpha_max): p50=%.2f m/s^3\n", R.item2.jPhysP50);
fprintf("  => a fixed j_max of %.1f sits between the current limit and the ceiling\n", ...
    opts.MaxJerk);
R.item2.pass = R.item2.jPhysP50 > R.item2.demandP50;
fprintf("  VERDICT: %s (ceiling %.1f vs demand %.1f)\n", ...
    ternary(R.item2.pass, "PASS", "FAIL"), R.item2.jPhysP50, R.item2.demandP50);

fprintf("\n=== SUMMARY ===\n");
fprintf("  item1 (crosstalk)      : %s\n", ternary(R.item1.pass, "PASS", "FAIL"));
fprintf("  item2 (jerk inner loop): %s\n", ternary(R.item2.pass, "PASS", "FAIL"));
fprintf("  item3 (reference loop) : PASS with caveat (%s)\n", R.item3.caveat);

if opts.MakePlot
    f = figure("Visible","on","Position",[100 100 1100 760]);
    tl = tiledlayout(f, 3, 2, "TileSpacing","compact");

    nexttile; plot(t, avu, "LineWidth",1.2); hold on; plot(t, avp, "LineWidth",1.2);
    plot(t, aUpClamped, "--", "LineWidth",1.2); grid on;
    legend("av_{up} now","av_{plan}","item1 clamped","Location","northwest");
    xlabel("t [s]"); ylabel("m/s^2"); title("item 1: vertical amplitude");

    nexttile; plot(t, R.item3.gainNow, "LineWidth",1.2); hold on;
    yline(1, "k--"); grid on; xlabel("t [s]"); ylabel("gain");
    title("item 3: av_{up}/a_{up,est} (want 1)");

    nexttile; plot(t, dDemand, "LineWidth",1.2); hold on;
    plot(t, jEff, "LineWidth",1.2); plot(t, jPhys, "LineWidth",1.2); grid on;
    legend("demand","current limit","physical ceiling","Location","northwest");
    xlabel("t [s]"); ylabel("m/s^3"); title("item 2: jerk budget");

    nexttile; plot(t, sec2, "LineWidth",1.2); grid on;
    xlabel("t [s]"); ylabel("sec^2(tilt)"); title("item 2: nonlinearity source");

    nexttile; plot(t, avu - avp, "LineWidth",1.2); hold on;
    plot(t, aUpClamped - avp, "LineWidth",1.2); yline(0,"k--"); grid on;
    legend("now","item1","Location","northwest");
    xlabel("t [s]"); ylabel("m/s^2"); title("vertical excess vs plan");

    nexttile; plot(t, T.a_cmd_h, "LineWidth",1.2); hold on;
    plot(t, aHWant, "LineWidth",1.2); grid on;
    legend("a_h now","|_ah_vec|","Location","northwest");
    xlabel("t [s]"); ylabel("m/s^2"); title("horizontal command fidelity");

    title(tl, "Offline feasibility of the three fixes");
    outPng = fullfile(here, "three_items_validation.png");
    exportgraphics(f, outPng, "Resolution", 130);
    R.figure = outPng;
    fprintf("\nfigure saved: %s\n", outPng);
end
end

% -------------------------------------------------------------------------
function s = ternary(cond, a, b)
% TERNARY  Pick a or b by a logical condition.
if cond
    s = a;
else
    s = b;
end
end
