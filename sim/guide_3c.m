function [a_cmd, dbg] = guide_3c(st, P, veh)
% guide_3c — Apollo 三段式 E-guidance（速度+位置+姿态），与 boot/B1040-9.ks §12 一致
% 段1 制动段：四次多项式 E-guidance（a_f=up 嵌入末端姿态），牛顿法解时标
% 段2 消速度：一阶消速度（P65）+ 推力渐竖直
% 段3 触地段：恒下降率 + 落点PD + 推力方向≤5°
r = st.r; v = st.v; a_net = st.a_net; g = veh.g0;
r_tgt = [-r(1); -r(2); (P.tgt_alt + P.aim_alt) - r(3)];
h_aim = r(3) - (P.tgt_alt + P.aim_alt);
h_gnd = r(3) - P.tgt_alt;
dist_hz = norm(r_tgt(1:2));
vz = v(3); v_h = v(1:2);

% 段选择（高度门）
if h_gnd < P.p66_h, eg_phase = 3;
elseif h_gnd < P.gate_h, eg_phase = 2;
else, eg_phase = 1; end

% 时标（自杀点火剖面，v7 验证过；不用四次式牛顿解——vz 符号陷阱会坠毁）
vz_s = sqrt(P.term_v^2 + 2*P.eg_ad*a_net*max(1,h_aim));
T = max(P.eg_tmin, 2*max(1,h_aim)/max(vz_s, -vz + P.term_v));
t_go = T;

r_hat = [0;0]; if dist_hz > 1, r_hat = r_tgt(1:2)/dist_hz; end
v_rad = dot(v_h, r_hat); v_tan = v_h - r_hat*v_rad;

if eg_phase == 1
    % ---- 段1：四次多项式 E-guidance ----
    a_v = -6*h_aim/max(0.01,T^2) - 2*(-P.term_v + 2*vz)/max(0.01,T);
    a_lat = [0;0];
    if dist_hz > 1
        % 横向分两段：
        %  · 接近段（dist > lat_hold）：匀减速时标二次型，主动刹到目标附近；
        %  · 保持段（dist ≤ lat_hold）：位置保持 PD（不依赖时标），把箭钉住——
        %    治"到位后被竖直段剩余时间吹走"（仿真实测 dist 351→2949 反弹）。
        % 保持段门要比交接目标门宽（500 vs 150），否则没进保持段就反弹。
        if dist_hz > P.lat_hold
            % 【T_lat 上限 = 竖直时标 T】v_rad→0 时 T_lat 爆炸 ⇒ a_rad→0 ⇒ 反弹
            T_lat = max(P.eg_tmin, 2*dist_hz/max(0.5,abs(v_rad)));
            T_lat = min(T_lat, T);
            a_rad = 6*dist_hz/max(0.01,T_lat^2) - 4*v_rad/max(0.01,T_lat);
        else
            a_rad = P.lat_pd_kp*dist_hz - P.lat_pd_kd*v_rad;
        end
        a_tan = -v_tan*P.term_kd;
        if norm(a_tan) > P.tan_cap, a_tan = a_tan/norm(a_tan)*P.tan_cap; end
        a_lat = r_hat*a_rad + a_tan;
        if norm(a_lat) > P.eg_lat_max, a_lat = a_lat/norm(a_lat)*P.eg_lat_max; end
    end
    if vz > 0, a_v = min(a_v, -g); end
    a_v = max(a_v, -0.5*g);
    % 主段分配：横向与竖直都要够。横向不能被竖直滑行压死（v7/v8 老坑：
    % 竖直滑行时 T_v_cmd 小 ⇒ 锥内横向余额枯竭 ⇒ 横向漂走）。
    % 本机 gimbal ±10.5°、推力大，主段允许大倾斜。先合成指令，再统一按 A_max 限幅。
    a_cmd = [a_lat; a_v];
    % 推力幅值限幅（竖直+横向的合成）
    if norm(a_cmd) > a_net
        % 超限时优先保竖直安全（不爬升），横向按比例缩
        excess = norm(a_cmd) - a_net;
        a_lat2 = a_cmd(1:2);
        if norm(a_lat2) > 0.1
            a_lat2 = a_lat2 * max(0, (norm(a_lat2) - excess)) / norm(a_lat2);
        end
        a_cmd = [a_lat2; a_cmd(3)];
        if norm(a_cmd) > a_net, a_cmd = a_cmd/norm(a_cmd)*a_net; end
    end
    cone_rng = max(1,h_aim)*tand(P.gamma_gs);
    cone_marg = dist_hz - cone_rng;
    if cone_marg > 0 && dist_hz > 1
        a_cmd(1:2) = a_cmd(1:2) + r_hat*min(P.cone_gain_max, cone_marg*P.cone_gain);
    end
elseif eg_phase == 2
    % ---- 段2：消速度 + 推力渐竖直 ----
    a_upcmd = P.term_k*(-P.term_v_slow - vz);
    if vz < -P.term_v_slow
        a_vff = vz^2/(2*max(1,h_gnd-1));
        a_upcmd = max(a_upcmd, a_vff / P.term_h_margin);
    end
    a_upcmd = max(-0.5*g, a_upcmd);
    a_upcmd = min(a_upcmd, a_net - g - P.term_h_reserve);
    t_h_lat = min(5, max(0.1, h_gnd/max(0.5,-vz)));
    r_hz = r_tgt(1:2); r_max = max(5, norm(v_h)*t_h_lat);
    if norm(r_hz) > r_max, r_hz = r_hz/norm(r_hz)*r_max; end
    a_hcmd = r_hz*P.term_kp - v_h*P.term_kd;
    if norm(v_h) > P.term_amax*max(1,t_h_lat)*1.2
        a_hcmd = -v_h*P.term_kd;
    end
    if norm(a_hcmd) > P.term_amax, a_hcmd = a_hcmd/norm(a_hcmd)*P.term_amax; end
    if h_gnd < P.att_h
        a_hcmd = a_hcmd * max(0.15, (h_gnd/P.att_h)^2);
    end
    a_cmd = [a_hcmd; a_upcmd];
else
    % ---- 段3：恒下降率 + 落点PD + 推力≤5° ----
    a_upcmd = P.term_k*(-P.term_v - vz);
    a_upcmd = max(-0.5*g, a_upcmd);
    a_upcmd = min(a_upcmd, a_net - g - P.term_h_reserve);
    a_hcmd = r_tgt(1:2)*P.eg_kp - v_h*P.eg_kd;
    if norm(a_hcmd) > P.eg_amax, a_hcmd = a_hcmd/norm(a_hcmd)*P.eg_amax; end
    a_cmd = [a_hcmd; a_upcmd];
    a_cap = tand(P.att_tilt5)*max(0.5, a_upcmd+g);
    if norm(a_cmd(1:2)) > a_cap
        a_cmd = [a_cmd(1:2)/norm(a_cmd(1:2))*a_cap; a_cmd(3)];
    end
end

if h_gnd < 2 && vz > -P.term_v
    a_cmd = [0;0;0];
elseif norm(a_cmd) > a_net
    a_cmd = a_cmd/norm(a_cmd)*a_net;
end
dbg.t_go = t_go; dbg.phase = eg_phase;
dbg.tilt_cmd = atan2d(norm(a_cmd(1:2)), a_cmd(3)+g);
end
