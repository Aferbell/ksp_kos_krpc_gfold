function [a_cmd, dbg] = guide_v7(st, P, veh)
% guide_v7 — 回收段重构制导律（第七版）单一实现，供 sim_landing / sim_sweep 共用。
%
% 输入 st（状态）：
%   .r (3x1) 位置（当地平直系，目标在地表原点） [m]
%   .v (3x1) 速度 [m/s]
%   .a_net   净推力加速度能力 = Tmax/m - g [m/s^2]
% 输入 P：制导参数结构体（字段见 sim_landing）
% 输入 veh：载具（只用 .g0）
% 输出 a_cmd (3x1)：净加速度指令（E,N,U 系，重力未含）
% 输出 dbg：诊断结构体（t_go 等）
%
% 结构（与 a_note.md §21-25、REFACTOR_PLAN 一致）：
%   主段：分轴二次型（Klumpp/Apollo ZEM/ZEV），时标由自杀点火剖面给；
%         约束以"推力锥预算分配"投影（竖直优先、横向用锥内余额），无弧切换；
%         滑翔角锥只作有界回锥偏置。
%   终端段（h_aim < term_h）：竖直 P 律 + 高速刹车前馈；横向 PD + SAFE 降级。
r = st.r; v = st.v; a_net = st.a_net; g = veh.g0;

r_tgt = [-r(1); -r(2); (P.tgt_alt + P.aim_alt) - r(3)];
h_aim = r(3) - (P.tgt_alt + P.aim_alt);
h_gnd = r(3) - P.tgt_alt;
dist_hz = norm(r_tgt(1:2));
vz = v(3); v_h = v(1:2);

% 分轴时标（剖面给，不由当前速度给）
v_f = -P.term_v;
r_hat = [0;0]; if dist_hz > 1, r_hat = r_tgt(1:2)/dist_hz; end
v_rad = dot(v_h, r_hat);
v_tan = v_h - r_hat*v_rad;
a_d  = P.ad_frac*a_net;
vz_star = sqrt(P.term_v^2 + 2*a_d*max(1,h_aim));
T_v   = max(P.tg_min, 2*max(1,h_aim)/max(vz_star, -vz + P.term_v));
t_go = T_v;

% 竖直二次型（上正）
a_v = -6*h_aim/T_v^2 - 2*(v_f + 2*vz)/T_v;

% 横向：径向/切向分拆。默认与竖直共用时标 T_v（天然同步到）；
% 末段（dist<lat_pd_gate）改用横向自己的时标 T_lat=dist/|v_rad| 主动收敛
% （治 47821217：共用 T_v 时末段竖直掉得比横向快，横向停在 480 m 收不动）。
if dist_hz > P.lat_pd_gate
    T_lat = T_v;
    a_rad = 6*dist_hz/T_lat^2 - 2*(0 + 2*v_rad)/T_lat;
else
    T_lat = max(P.tg_min, dist_hz/max(0.5,abs(v_rad)));
    a_rad = 6*dist_hz/T_lat^2 - 2*(0 + 2*v_rad)/T_lat;
    if dist_hz < P.lat_pd_gate * 0.25   % 很近（<100 m）才换纯 PD，防增益爆炸
        a_rad = P.lat_pd_kp*dist_hz - P.lat_pd_kd*v_rad;
    end
end
a_tan = -v_tan*P.term_kd;
if norm(a_tan) > P.tan_cap, a_tan = a_tan/norm(a_tan)*P.tan_cap; end
a_lat = r_hat*a_rad + a_tan;

% 主段竖直铁律：绝不爬升；下限 -0.5g 给横向留推力底座
if vz > 0, a_v = min(a_v, -g); end
a_v = max(a_v, -0.5*g);

if h_aim < P.term_h
    % ---------- 终端段：温和减速段 + P 律 ----------
    % 竖直不急刹：|vz|>term_v_slow 时用"够用就好"的刹车率 vz²/(2h·margin)，
    % 并给横向预留 term_h_reserve 配额（治 47822691：急刹偷走横向推力）。
    a_vp = P.term_k*(-P.term_v - vz);
    a_vff = vz^2/(2*max(1,h_gnd-1));
    if vz < -P.term_v_slow
        a_upcmd = a_vff / P.term_h_margin;
        a_upcmd = min(a_upcmd, a_vp);
        a_upcmd = max(a_upcmd, P.term_k*(-P.term_v_slow - vz));
    elseif vz < -6
        a_upcmd = max(a_vp, a_vff);
    else
        a_upcmd = a_vp;
    end
    a_upcmd = max(-0.5*g, a_upcmd);
    a_upcmd = min(a_upcmd, a_net - g - P.term_h_reserve);
    t_h_lat2 = min(5, max(0.1, h_gnd/max(0.5,-vz)));
    r_hz = r_tgt(1:2);
    r_max = max(5, norm(v_h)*t_h_lat2);
    if norm(r_hz) > r_max, r_hz = r_hz/norm(r_hz)*r_max; end
    a_hcmd = r_hz*P.term_kp - v_h*P.term_kd;
    tilt_cap = P.tilt_min + (P.term_tilt-P.tilt_min)*max(0,min(1,h_gnd/P.tilt_span));
    a_cap = min(P.term_amax, tand(tilt_cap)*max(0.5, a_upcmd+g));
    if norm(v_h) > a_cap*max(1,t_h_lat2)*1.2
        a_hcmd = -v_h*P.term_kd;   % SAFE：只杀速度，不追位置
    end
    if norm(a_hcmd) > a_cap, a_hcmd = a_hcmd/norm(a_hcmd)*a_cap; end
    a_cmd = [a_hcmd; a_upcmd];
else
    % ---------- 主段：推力锥预算分配 ----------
    tilt_lim = P.tilt_lo + (P.tilt_hi-P.tilt_lo)*max(0,h_gnd)/(max(0,h_gnd)+P.tilt_taper);
    A_max = a_net + g;
    % 竖直底座（竖直优先；不做横向反拉——仿真实测会反馈成爬升悬停烧油）
    T_v_cmd = min(max(0, a_v + g), A_max);
    % 滑行段推力底座：a_v 压到 -0.5g 时竖直底座太小、锥内横向余额枯竭，
    % 横向修正爬行（仿真实测：±10° 偏离时 vh 拖到 200+ 无法收敛）。
    % 给一个最小推力底座（默认 0.75g）——代价是下降率比"最优"略缓，换来
    % 滑行段横向机动的实际执行力。底座不到 A_max 的 1/3，不危及竖直安全。
    T_v_cmd = max(T_v_cmd, min(P.coast_floor_g*g, A_max));
    T_h_cap = min(T_v_cmd*tand(tilt_lim), sqrt(max(0, A_max^2 - T_v_cmd^2)));
    a_h2 = a_lat;
    if norm(a_h2) > T_h_cap, a_h2 = a_h2/norm(a_h2)*T_h_cap; end
    a_cmd = [a_h2; T_v_cmd - g];
    % 滑翔角锥：有界回锥偏置
    cone_rng = max(1,h_aim)*tand(P.gamma_gs);
    if dist_hz > cone_rng && dist_hz > 1
        a_cmd(1:2) = a_cmd(1:2) + r_hat*min(P.cone_gain_max, (dist_hz-cone_rng)*P.cone_gain);
    end
end

% 贴地关停 + 幅值钳位
if h_gnd < 2 && vz > -P.term_v
    a_cmd = [0;0;0];
elseif norm(a_cmd) > a_net
    a_cmd = a_cmd/norm(a_cmd)*a_net;
end

dbg.t_go = t_go; dbg.h_aim = h_aim; dbg.h_gnd = h_gnd; dbg.dist_hz = dist_hz;
dbg.v_rad = v_rad; dbg.phase = 1 + (h_aim < P.term_h);
end
