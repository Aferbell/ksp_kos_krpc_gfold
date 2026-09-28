% replay47819000.m — 用第六趟日志逐帧复算回收段制导律，找出"指令与公式不符"的帧
% 方法：geo_lat/geo_lng 差分重建速度矢量方向，r_tgt 由目标坐标+高度重建，
% 然后严格按 boot/B1040-9.ks L1167-1404 的公式重放，与日志的 acmd_up/acmd_h/thr 对比。
clear; clc;
p = 'D:\SteamLibrary\steamapps\common\Kerbal Space Program\Ships\Script\log-B1040-9\land_log_47819000.csv';
T = readtable(p);
n = height(T);

% --- 常量（与脚本配置一致） ---
tgt_lat = -0.0972060951675948; tgt_lon = -74.5576822740041;
target_alt = 36.8; aim_alt = 15;
tg_min = 4.0; gamma_gs = 60; tilt_hi = 55; tilt_lo = 25; tilt_taper = 400;
term_h = 80; term_k = 2; term_v = 2.0; term_kp = 0.8; term_kd = 1.8;
term_amax = 12; term_tilt = 20; tilt_min = 6; tilt_span = 120;
Re = 600000; g0 = 9.81;

deg2rad = pi/180;
% ENU 重建（原点在目标点）
lat = T.geo_lat*deg2rad; lon = T.geo_lng*deg2rad;
lat0 = tgt_lat*deg2rad; lon0 = tgt_lon*deg2rad;
E = (lon-lon0).*cos(lat0)*Re;   % 东 [m]
N = (lat-lat0)*Re;              % 北 [m]
H = T.alt;                       % alt:radar
t = T.t;

% 速度：位置中心差分
vE = gradient(E,t); vN = gradient(N,t); vU = gradient(H,t);

% 比较结果存放
cmp = zeros(n,9); % idx, tgo_code, up_nc, up_log, h_nc, h_log, branch, tilt_after, thr_pred
for i = 1:n
    vz_log = T.vz(i); vh_log = T.vh(i);
    h_aim = H(i) - (target_alt + aim_alt);
    h_gnd = H(i) - target_alt;
    % 只在点火后主段复算（phase>=1 且 h_aim>term_h）
    if T.phase(i) < 1 || h_aim < term_h, continue; end
    vz = vU(i);                    % 重建的垂速（应与 vz_log 接近）
    % r_tgt：水平指向目标，垂距 h_aim（向下为负的 up 分量）
    r_h = [-E(i); -N(i)];          % 水平：从船指向目标
    dist_hz = norm(r_h);
    v_h = [vE(i); vN(i)];
    v_now_up = vz;
    t_go = max(tg_min, 2*h_aim/max(sqrt(max(1,h_aim)), -vz));
    % a_net 用日志 avail 列（净加速度）
    a_net = T.avail(i) - g0;  % avail 列是 avail_thrust/mass（总推力加速度），净 = 减 g
    % --- a_nc = r*6/T² - v*4/T - g*up ---
    a_nc_h  = r_h*(6/t_go^2) - v_h*(4/t_go);
    a_nc_up = -6*h_aim/t_go^2 - 4*vz/t_go - g0;   % vdot(r,up)=-h_aim, vdot(v,up)=vz
    % --- ④b 横向分解 ---
    if dist_hz > 1
        r_hat = r_h/dist_hz;
        v_rad = dot(v_h, r_hat);
        v_tan = v_h - r_hat*v_rad;
        t_h_lat = min(60, max(0.5, dist_hz/max(0.5, abs(v_rad))));
        a_pd_h = r_hat*(6*dist_hz/t_h_lat^2 - 4*v_rad/t_h_lat) - v_tan*term_kd;
        t_fall = max(0.2, h_gnd/max(0.5, -vz));
        if dist_hz/t_fall > max(1, abs(v_rad))
            if dot(a_pd_h, r_hat) < 0
                a_pd_h = a_pd_h - r_hat*dot(a_pd_h, r_hat);
            end
        end
        if norm(a_pd_h) > norm(a_nc_h)
            a_nc_h2 = a_pd_h;   % 替换横向，保留竖直
        else
            a_nc_h2 = a_nc_h;
        end
    else
        a_nc_h2 = a_nc_h;
    end
    % --- 约束弧 a_cone ---
    u_r_ax = -6*h_aim/t_go^2 - 4*(-vz)/t_go;   % 注意代码里 v_up=-vz
    cone_up = max(0, u_r_ax + g0);
    if dist_hz > 0.5
        cone_h = -(r_h*(6/t_go^2) - v_h*(4/t_go));
    else
        cone_h = [0;0];
    end
    % --- 选弧 ---
    cone_rng = max(1,h_aim)*tand(gamma_gs);
    cone_on = (dist_hz - cone_rng) > 0;
    if cone_on
        a_up = cone_up; a_h = cone_h; branch = 2;
    elseif norm([a_nc_h2; a_nc_up]) <= a_net
        a_up = a_nc_up; a_h = a_nc_h2; branch = 1;
    else
        a_up = cone_up; a_h = cone_h; branch = 2;
    end
    % --- 推力指向锥（仅主段） ---
    tilt_lim = tilt_lo + (tilt_hi-tilt_lo)*max(0,h_gnd)/(max(0,h_gnd)+tilt_taper);
    amag = norm([a_h; a_up]);
    tilt_gdn = atan2d(norm(a_h), a_up);
    if tilt_gdn > tilt_lim && tilt_gdn > 0.01
        a_up2 = cosd(tilt_lim)*amag;
        a_h2 = (a_h/norm(a_h))*sind(tilt_lim)*amag;
        a_up = a_up2; a_h = a_h2; branch = branch + 10;
    end
    cmp(i,:) = [i, t_go, a_up, T.acmd_up(i), norm(a_h), T.acmd_h(i), branch, tilt_gdn, norm([a_h;a_up+g0])/T.avail(i)];
end
cmp = cmp(cmp(:,1)>0,:);
fprintf('idx | tgo | up_code up_log | h_code h_log | branch tiltG | thr_pred thr_log\n');
for k = 1:3:size(cmp,1)
    fprintf('%4d | %5.1f | %7.2f %6.2f | %7.2f %6.2f | %2d %5.1f | %5.2f %5.2f\n', ...
        cmp(k,1), cmp(k,2), cmp(k,3), cmp(k,4), cmp(k,5), cmp(k,6), cmp(k,7), cmp(k,8), cmp(k,9), T.thr(cmp(k,1)));
end
% 找差异最大的帧
d = abs(cmp(:,3)-cmp(:,4)) + abs(cmp(:,5)-cmp(:,6));
[dm,mi] = max(d);
fprintf('\n最大差异帧 idx=%d: code(up=%.2f,h=%.2f) log(up=%.2f,h=%.2f)\n', ...
    cmp(mi,1), cmp(mi,3), cmp(mi,5), cmp(mi,4), cmp(mi,6));
