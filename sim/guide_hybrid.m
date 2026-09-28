function [a_cmd, dbg] = guide_hybrid(st, P, veh)
% guide_hybrid -- hybrid guidance. Phase 1 = v8 quartic E-guidance (SAFE BASELINE).
%
% History: several phase-1 laws were tried and ALL failed (see
% dev/reports/FLIGHT_DIAG_47827395.md). The v8 quartic form is restored because
% it is the best historical result (11.8 m miss) and, critically, it never
% commands a downward dive nor a climb.
r = st.r; v = st.v; a_net = st.a_net; g = veh.g0;
r_tgt = [-r(1); -r(2); (P.tgt_alt + P.aim_alt) - r(3)];
h_aim = r(3) - (P.tgt_alt + P.aim_alt);
h_gnd = r(3) - P.tgt_alt;
dist_hz = norm(r_tgt(1:2));
vz = v(3); v_h = v(1:2);

cone_rng  = max(1, h_aim) / tand(P.gamma_gs);
cone_marg = dist_hz - cone_rng;
in_cone   = cone_marg <= 0;

if h_gnd < P.p66_h
    eg_phase = 3;
elseif h_gnd < P.gate_h && in_cone
    eg_phase = 2;
else
    eg_phase = 1;
end

r_hat = [0;0]; if dist_hz > 1, r_hat = r_tgt(1:2)/dist_hz; end
vz_s = sqrt(P.term_v^2 + 2*P.eg_ad*a_net*max(1, h_aim));
T = max(P.eg_tmin, 2*max(1, h_aim) / max(vz_s, -vz + P.term_v));
t_go = T;

if eg_phase == 1
    a_v = -6*h_aim/max(0.01,T^2) - 2*(-P.term_v + 2*vz)/max(0.01,T);
    a_lat = [0;0];
    if dist_hz > 1
        v_rad = dot(v_h, r_hat); v_tan = v_h - r_hat*v_rad;
        if dist_hz > P.lat_hold
            T_lat = max(P.lat_tmin, 2*dist_hz/max(0.5, abs(v_rad)));
            a_rad = 6*dist_hz/max(0.01,T_lat^2) - 4*v_rad/max(0.01,T_lat);
        else
            a_rad = P.lat_pd_kp*dist_hz - P.lat_pd_kd*v_rad;
        end
        a_tan = -v_tan*P.term_kd;
        if norm(a_tan) > P.tan_cap, a_tan = a_tan/norm(a_tan)*P.tan_cap; end
        a_lat = r_hat*a_rad + a_tan;
        lat_budget = min(P.eg_lat_max, sqrt(max(0, a_net^2 - max(0,a_v)^2)));
        if norm(a_lat) > lat_budget, a_lat = a_lat/norm(a_lat)*lat_budget; end
    end
    if vz > 0, a_v = min(a_v, -g); end
    if a_v < -0.5*g, a_v = -0.5*g; end
    a_cmd = [a_lat; a_v];
    if norm(a_cmd) > a_net
        excess = norm(a_cmd) - a_net;
        ah = a_cmd(1:2);
        if norm(ah) > 0.1, ah = ah * max(0, norm(ah)-excess)/norm(ah); end
        a_cmd = [ah; a_cmd(3)];
        if norm(a_cmd) > a_net, a_cmd = a_cmd/norm(a_cmd)*a_net; end
    end
    if cone_marg > 0 && dist_hz > 1
        a_cmd(1:2) = a_cmd(1:2) + r_hat*min(P.cone_gain_max, cone_marg*P.cone_gain);
    end

elseif eg_phase == 2
    a_upcmd = P.term_k * (-P.term_v_slow - vz);
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
    a_upcmd = P.term_k * (-P.term_v - vz);
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
dbg.cone_marg = cone_marg; dbg.in_cone = in_cone;
dbg.tilt_cmd = atan2d(norm(a_cmd(1:2)), a_cmd(3)+g);
end
