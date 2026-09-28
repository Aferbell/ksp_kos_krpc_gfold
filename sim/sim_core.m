function R = sim_core(s0, veh, tgt, P, mode, dbg)
% sim_core — 回收段闭环仿真积分核心。制导律由 mode 选择：
%   'v6' -> 内嵌第六版（基线，用于对照）
%   'v7' -> guide_v7（单一实现）
% s0: .r .v .m；veh: .Tmax .Isp .g0 .ref_wmax .att_tau；tgt: .alt_true .aim_alt
% P: 制导参数；dbg: 1 时每 2 s 打印一行。
if nargin < 6, dbg = 0; end
dt = 0.02; tmax = 150;
nmax = round(tmax/dt);
r = s0.r; v = s0.v; m = s0.m;
pl_dir = [0;0;1]; att_dir = [0;0;1];
R.t = zeros(1,nmax); R.r = zeros(3,nmax); R.v = zeros(3,nmax);
R.thr = zeros(1,nmax); R.tilt = zeros(1,nmax); R.m = zeros(1,nmax);
R.phase = zeros(1,nmax);
P.tgt_alt = tgt.alt_true; P.aim_alt = tgt.aim_alt;
g = veh.g0;
phase = 1;

for k = 1:nmax
    tk = (k-1)*dt;
    upv = [0;0;1];
    a_net = veh.Tmax/m - g;
    h_aim = r(3) - (tgt.alt_true + tgt.aim_alt);
    h_gnd = r(3) - tgt.alt_true;
    if h_aim < P.term_h, phase = 2; end

    if strcmp(mode,'v7')
        st.r = r; st.v = v; st.a_net = a_net;
        [a_cmd, d] = guide_v7(st, P, veh);
        t_go = d.t_go;
    else
        % ---- v6 基线（第六版原样，仅供对照）----
        r_tgt = [-r(1); -r(2); (tgt.alt_true + tgt.aim_alt) - r(3)];
        dist_hz = norm(r_tgt(1:2)); vz = v(3); v_up = -vz; v_h = v(1:2);
        t_go = max(P.tg_min, 2*h_aim/max(sqrt(max(1,h_aim)), -vz));
        cone_rng = max(1,h_aim)*tand(P.gamma_gs);
        cone_on = (dist_hz - cone_rng) > 0;
        tilt_lim = P.tilt_lo + (P.tilt_hi-P.tilt_lo)*max(0,h_gnd)/(max(0,h_gnd)+P.tilt_taper);
        u_r_ax = -6*h_aim/max(0.01,t_go^2) - 4*v_up/max(0.01,t_go);
        if dist_hz > 0.5
            a_cone = [-(r_tgt(1:2)*(6/t_go^2) - v_h*(4/t_go)); max(0,u_r_ax+g)];
        else
            a_cone = [0;0;max(0,u_r_ax+g)];
        end
        a_nc = r_tgt*(6/t_go^2) - v*(4/t_go) - upv*g;
        if dist_hz > 1
            r_hat = r_tgt(1:2)/dist_hz; v_rad = dot(v_h, r_hat); v_tan = v_h - r_hat*v_rad;
            t_h_lat = min(60, max(0.5, dist_hz/max(0.5,abs(v_rad))));
            a_pd2 = r_hat*(6*dist_hz/t_h_lat^2 - 4*v_rad/t_h_lat) - v_tan*P.term_kd;
            t_fall = max(0.2, h_gnd/max(0.5,-vz));
            if dist_hz/t_fall > max(1,abs(v_rad)) && dot(a_pd2,r_hat) < 0
                a_pd2 = a_pd2 - r_hat*dot(a_pd2,r_hat);
            end
            if norm(a_pd2) > norm(a_nc(1:2)), a_nc = [a_pd2; a_nc(3)]; end
        end
        if phase == 2
            a_vp = P.term_k*(-P.term_v - vz);
            a_vff = vz^2/(2*max(1,h_gnd-1));
            a_upcmd = max(0, max(a_vp, a_vff));
            if a_upcmd + g > a_net, a_upcmd = a_net - g; end
            t_h_lat2 = min(5, max(0.1, h_gnd/max(0.5,-vz)));
            ds_cap = max(0, norm(v_h)*t_h_lat2 - 0.5*P.term_amax*t_h_lat2^2);
            r_hz = r_tgt(1:2);
            if norm(r_hz) > ds_cap + 0.001, r_hz = r_hz/norm(r_hz)*ds_cap; end
            a_hcmd = r_hz*P.term_kp - v_h*P.term_kd;
            tilt_cap = P.tilt_min + (P.term_tilt-P.tilt_min)*max(0,min(1,h_gnd/P.tilt_span));
            a_cap = min(P.term_amax, tand(tilt_cap)*max(0.5, a_upcmd+g));
            if norm(a_hcmd) > a_cap, a_hcmd = a_hcmd/norm(a_hcmd)*a_cap; end
            a_cmd = [a_hcmd; a_upcmd];
        elseif cone_on
            a_cmd = a_cone;
        elseif norm(a_nc) <= a_net
            a_cmd = a_nc;
        else
            a_cmd = a_cone;
        end
        tilt_gdn = atan2d(norm(a_cmd(1:2)), a_cmd(3));
        if phase == 1 && tilt_gdn > tilt_lim && tilt_gdn > 0.01
            a_cmd = [a_cmd(1:2)/norm(a_cmd(1:2))*sind(tilt_lim); cosd(tilt_lim)]*norm(a_cmd);
        end
        if h_gnd < 2 && vz > -P.term_v
            a_cmd = [0;0;0];
        elseif norm(a_cmd) > a_net
            a_cmd = a_cmd/norm(a_cmd)*a_net;
        end
    end

    % ---- 执行：推力分配 + 姿态滞后 ----
    a_thr = a_cmd + upv*g;
    t_up = min(max(0, a_thr(3)), veh.Tmax/m);
    t_h2 = a_thr(1:2);
    t_h_max = sqrt(max(0, (veh.Tmax/m)^2 - t_up^2));
    if norm(t_h2) > t_h_max, t_h2 = t_h2/norm(t_h2)*t_h_max; end
    a_thr_act = [t_h2; t_up];
    if norm(a_thr_act) > 0.001
        pl_want = a_thr_act/norm(a_thr_act);
        ang = acosd(max(-1,min(1,dot(pl_dir,pl_want))));
        if ang <= veh.ref_wmax*dt
            pl_dir = pl_want;
        else
            pl_dir = pl_dir + (pl_want-pl_dir)*(veh.ref_wmax*dt/ang);
            pl_dir = pl_dir/norm(pl_dir);
        end
    end
    att_dir = att_dir + (pl_dir - att_dir)*(dt/veh.att_tau);
    att_dir = att_dir/norm(att_dir);
    a_real = att_dir*norm(a_thr_act) - upv*g;
    v = v + a_real*dt; r = r + v*dt;
    m = m - (norm(a_thr_act)*m)/(veh.Isp*veh.g0)*dt;

    R.t(k) = tk; R.r(:,k) = r; R.v(:,k) = v; R.m(k) = m;
    R.thr(k) = norm(a_thr_act)/(veh.Tmax/m);
    R.tilt(k) = atan2d(norm(a_thr_act(1:2)), a_thr_act(3));
    R.phase(k) = phase;
    if dbg && mod(k, round(2/dt)) == 1
        fprintf('  t=%5.1f h=%7.1f vz=%7.1f vh=%6.1f dist=%7.1f tgo=%5.1f thr=%.2f tilt=%5.1f ph=%d |acmd|=%6.1f anet=%5.1f\n', ...
            tk, r(3), v(3), norm(v(1:2)), norm(r(1:2)), t_go, R.thr(k), R.tilt(k), phase, norm(a_cmd), a_net);
    end
    if r(3) <= tgt.alt_true
        R.t = R.t(1:k); R.r = R.r(:,1:k); R.v = R.v(:,1:k); R.m = R.m(1:k);
        R.thr = R.thr(1:k); R.tilt = R.tilt(1:k); R.phase = R.phase(1:k);
        if R.v(3,end) > -5 && norm(R.v(1:2,end)) < 5 && R.tilt(end) < 10
            R.verdict = '软着陆达标';
        elseif R.v(3,end) > -15
            R.verdict = '接地偏重';
        else
            R.verdict = '坠毁';
        end
        return;
    end
end
R.t = R.t(1:nmax); R.r = R.r(:,1:nmax); R.v = R.v(:,1:nmax); R.m = R.m(1:nmax);
R.thr = R.thr(1:nmax); R.tilt = R.tilt(1:nmax); R.phase = R.phase(1:nmax);
R.verdict = '超时未落地';
end
