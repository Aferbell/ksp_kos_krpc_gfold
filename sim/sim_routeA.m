function R = sim_routeA(mode, opts)
% SIM_ROUTEA  Validate route A: plan-timescale consistency of the vertical channel.
%
% Route A hypothesis: the vehicle takes 29% longer than the plan's tf because the
% controller advances the plan by *nearest position* instead of by *time*. As a
% consequence the plan's velocity profile is never demanded, the vertical PD sees
% a persistent (plan_vz - vz) < 0 error, and its correction drives a_up below the
% physical floor (a_up must be >= 0: thrust pushes up only).
%
% This function replays the logged plan profile under different timescale modes
% and reports whether the vertical command stays physical.
%
% Modes
%   "pos"  plan time from the logged index (nearest-position) -- current behaviour
%   "time" plan time advances with actual time (scale = tf_plan/tf_actual)
%   "map"  plan time advances with actual time, stretched to reach the plan end
%
% Name-Value options
%   DataFile  full path to the gfold log CSV
%   N         plan node count (default 80)
%   TfPlan    plan flight time [s] (default 18.5)
%   KVel      vertical velocity gain [1/s] (default 0.8)
%   KPos      vertical position gain [1/s^2] (default 0.5)
%   TfSim     simulated flight time [s] (default: actual log duration)
%   Dt        integration step [s] (default 0.02)
%
% Output R (struct): t, h, vz, tp, aUp, aDes, negFrac, landVz, landVh, msg

arguments
    mode (1,1) string {mustBeMember(mode, ["pos","time","map"])}
    opts.DataFile (1,1) string = ""
    opts.N (1,1) double = 80
    opts.TfPlan (1,1) double = 18.5
    opts.KVel (1,1) double = 0.8
    opts.KPos (1,1) double = 0.5
    opts.TfSim (1,1) double = NaN
    opts.Dt (1,1) double = 0.02
end

G0 = 9.81;
if strlength(opts.DataFile) == 0
    here = fileparts(mfilename("fullpath"));
    opts.DataFile = fullfile(here, "..", "..", "log-B1040-9-gfold", ...
        "gfold_log_20261007_212924.csv");
end

% ---- load log ----
L = loadGfoldLog(opts.DataFile);

% ---- build plan profile as a function of PLAN time ----
% The log records the plan at the visited index, so ni is monotonic and
% tp = ni/(N-1)*TfPlan is monotonic. That yields the plan profile directly.
tpLog = L.n_i ./ (opts.N - 1) .* opts.TfPlan;
[tpU, iu] = unique(tpLog, "stable");
pAlt = L.plan_alt(iu);
pVz  = L.plan_vz(iu);
pAcc = gradient(pVz, tpU);            % plan vertical acceleration [m/s^2]

profAlt = @(tp) interp1(tpU, pAlt, min(max(tp, tpU(1)), tpU(end)), "linear");
profVz  = @(tp) interp1(tpU, pVz,  min(max(tp, tpU(1)), tpU(end)), "linear");
profAcc = @(tp) interp1(tpU, pAcc, min(max(tp, tpU(1)), tpU(end)), "linear");

% ---- available thrust acceleration ----
% a_cap = |target_a| / thr_cmd  (from the log, median over the flight)
okc = L.thr_cmd > 1e-6 & isfinite(L.a_cmd_mag);
aCap = median(L.a_cmd_mag(okc) ./ L.thr_cmd(okc));

% ---- simulate ----
dt = opts.Dt;
if isnan(opts.TfSim)
    tfSim = L.t(end);
else
    tfSim = opts.TfSim;
end
n = round(tfSim / dt);

t   = zeros(1, n);
h   = zeros(1, n);
vz  = zeros(1, n);
tps = zeros(1, n);
aUp = zeros(1, n);
aDs = zeros(1, n);

h(1)  = L.alt(1);
vz(1) = L.vz(1);
tps(1) = 0;

% Position-based index lookup for mode "pos": nearest plan altitude.
for k = 1:n
    tk = (k - 1) * dt;
    t(k) = tk;

    switch mode
        case "pos"
            % nearest-position lookup, exactly what the flight controller does
            [~, j] = min(abs(pAlt - h(k)));
            tpNow = tpU(j);
        case "time"
            % plan time advances with real time, same rate as designed
            tpNow = min(tk, tpU(end));
        case "map"
            % stretched so the plan end is reached exactly at tfSim
            tpNow = min(tk * (tpU(end) / tfSim), tpU(end));
    end
    tps(k) = tpNow;

    % vertical PD tracking of the plan profile (thrust-acceleration form)
    aDes = G0 + profAcc(tpNow) ...
        + opts.KVel * (profVz(tpNow) - vz(k)) ...
        + opts.KPos * (profAlt(tpNow) - h(k));

    % physical limits: thrust acceleration in [0, aCap]; j-limit is stepwise
    aCmd = min(max(aDes, 0), aCap);
    aUp(k) = aCmd;
    aDs(k) = aDes;

    % integrate
    aNet = aCmd - G0;
    vz(k + 1) = vz(k) + aNet * dt;
    h(k + 1)  = h(k) + vz(k + 1) * dt;

    if h(k + 1) <= 0
        t = t(1:k); h = h(1:k+1); vz = vz(1:k+1);
        tps = tps(1:k); aUp = aUp(1:k); aDs = aDs(1:k);
        break
    end
end

R = struct();
R.t = t; R.h = h(1:numel(t)); R.vz = vz(1:numel(t)); R.tp = tps;
R.aUp = aUp; R.aDes = aDs;
R.aCap = aCap;
R.negFrac = mean(aDs < 0);
R.landVz = vz(numel(t));
R.landVh = 0;
R.mode = mode;
R.msg = sprintf("mode=%-4s  negFrac=%.3f  landVz=%.2f  dur=%.2f s", ...
    mode, R.negFrac, R.landVz, t(end));
end

% -------------------------------------------------------------------------
function L = loadGfoldLog(f)
% LOADGFOLDLOG  Read a gfold CSV, skipping the leading '#' comment lines.
T = readtable(f, CommentStyle="#", VariableNamingRule="preserve");
L = struct();
L.t = T.t; L.alt = T.alt; L.vz = T.vz;
L.plan_alt = T.plan_alt; L.plan_vz = T.plan_vz;
L.n_i = T.n_i;
L.thr_cmd = T.thr_cmd; L.a_cmd_mag = T.a_cmd_mag;
end
