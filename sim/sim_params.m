function P = sim_params
% sim_params — 制导/载具参数单一出处（与 boot/B1040-9.ks 对齐）
% --- 制导（与脚本 §参数 一致）---
P.tg_min = 4.0; P.gamma_gs = 60;
P.tilt_hi = 55; P.tilt_lo = 25; P.tilt_taper = 400;
P.term_h = 400; P.term_k = 2; P.term_v = 2.0; P.term_kp = 0.8; P.term_kd = 1.8;
P.term_amax = 20; P.term_tilt = 20; P.tilt_min = 6; P.tilt_span = 120;
% --- v7（47822691 后修订：温和减速段）---
P.term_v_slow = 40;      % 减速段→精降段速度门 [m/s]
P.term_h_margin = 1.3;   % 减速段刹车高度余量
P.term_h_reserve = 8;    % 竖直给横向预留的推力配额 [m/s^2]
% --- v7（47821217 后修订）---
P.ad_frac = 0.62;        % 竖直剖面减速度系数（净 a_net 的比例）
P.adl_frac = 0.5;        % 横向剖面减速度系数
P.lat_timescale = 2;     % 1=横向自有时标, 2=与竖直共用（末段另由 lat_pd_gate 接管）
P.lat_pd_gate = 400;     % 横向 PD 交接距离 [m]
P.lat_pd_kp = 0.8; P.lat_pd_kd = 1.8;
P.tan_cap = 15;          % 切向阻尼限幅 [m/s^2]
P.cone_gain = 0.5;       % 回锥增益 [1/s^2]
P.cone_gain_max = 8;     % 回锥偏置上限 [m/s^2]
P.coast_floor_g = 0.75;  % 主段滑行推力底座（g 的倍数）：保横向锥内余额
% --- 载具（第六趟实测）---
P.veh.Tmax = 12.8e6; P.veh.Isp = 350; P.veh.g0 = 9.81;
P.veh.ref_wmax = 12; P.veh.att_tau = 0.8;
% --- 目标 ---
P.tgt.alt_true = 36.8; P.tgt.aim_alt = 15;
end
