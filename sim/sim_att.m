% sim_att.m — 回收段 3DoF 质点 + 姿态二阶动力学闭环仿真
% 目的：同时验证"速度+位置+姿态（倾角）"三约束的制导律。
% 与 sim_landing 的差别：姿态不再是"瞬时指向 + 一阶滞后"，而是
%   有转动惯量、有姿态环带宽、有气动干扰力矩的二阶系统。
% 用法：sim_att            （从 47822691 真实点火帧起飞）
%       sim_att(P_override)（覆盖参数）
function R = sim_att(P_over)
if nargin < 1, P_over = struct(); end
P = sim_att_params;
f = fieldnames(P_over); for k=1:numel(f), P.(f{k}) = P_over.(f{k}); end

% ---------- 载具（47822691 实测 + VEHICLE_B1040-9_ANALYSIS.md 真实数据）----------
veh.Tmax = 12.8e6;      % 点火时可用推力 [N]（thr_avail≈12800 kN，9台SSME@150%）
veh.Isp  = 315; veh.g0 = 9.81;   % SSME 海平面 Isp（真实值，原误用 350）
veh.m0   = 225.03e3;    % 点火质量 [kg]（日志实测；着陆末段会降到 ~150 t）
% 姿态动力学（真实载具：gimbal ±10.5°/臂11.8m/3.2MN·m，I≈1.8e4 kg·m²）
veh.Iyy  = 1.8e4;       % 绕俯仰/偏航轴转动惯量 [kg·m^2]（craft 报告估算）
veh.gimbal = 10.5;      % 引擎矢量偏转上限 [deg]（真实值，原误用 6）
veh.att_bw = 1.2;       % 姿态环带宽 [rad/s]（小惯量+gimbal 3.2MN·m，响应快）
veh.att_zeta = 0.9;     % 姿态环阻尼比
veh.aero_q = 0.35;      % 气动干扰力矩增益 [N·m/(m/s·deg)]（FAR 近似）
veh.ref_wmax = 12;      % 姿态参考限速 [deg/s]

% ---------- 目标 ----------
tgt.alt_true = 36.8; tgt.aim_alt = 15;
P.tgt_alt = tgt.alt_true; P.aim_alt = tgt.aim_alt;

% ---------- 初始条件（land_log_47822691 真实点火帧）----------
tgt_lat = -0.0972060951675948; tgt_lon = -74.5576822740041;
Re = 600000; d2r = pi/180;
T = readtable('D:\SteamLibrary\steamapps\common\Kerbal Space Program\Ships\Script\log-B1040-9\land_log_47822691.csv');
ii = find([double(string(T.phase))] >= 1, 1, 'first');
if isempty(ii), ii = 129; end
E0 = (T.geo_lng(ii)-tgt_lon)*d2r*cos(tgt_lat*d2r)*Re;
N0 = (T.geo_lat(ii)-tgt_lat)*d2r*Re;
dE = (T.geo_lng(min(ii+12,height(T)))-T.geo_lng(ii))*d2r*cos(tgt_lat*d2r)*Re;
dN = (T.geo_lat(min(ii+12,height(T)))-T.geo_lat(ii))*d2r*Re;
hd = [dE; dN]; hd = hd/norm(hd);
r = [E0; N0; T.alt(ii)];
v = [hd*T.vh(ii); T.vz(ii)];
m = T.mass(ii)*1000;

% ---------- 状态：质点 + 姿态 ----------
% att(1:2)=推力轴在 ENU 水平面内的偏角分量 [rad]（小角近似，att(3)≈1）
% om(1:2)=姿态角速度 [rad/s]
att = [0;0]; om = [0;0];
pl_dir = [0;0;1];

dt = 0.02; tmax = 150; nmax = round(tmax/dt);
R.t = zeros(1,nmax); R.r = zeros(3,nmax); R.v = zeros(3,nmax); R.m = zeros(1,nmax);
R.thr = zeros(1,nmax); R.tilt = zeros(1,nmax); R.att_err = zeros(1,nmax);
R.om = zeros(2,nmax); R.phase = zeros(1,nmax);
phase = 1;

for k = 1:nmax
    tk = (k-1)*dt;
    upv = [0;0;1]; g = veh.g0;
    a_net = veh.Tmax/m - g;
    r_tgt = [-r(1); -r(2); (tgt.alt_true + tgt.aim_alt) - r(3)];
    h_aim = r(3) - (tgt.alt_true + tgt.aim_alt);
    h_gnd = r(3) - tgt.alt_true;
    dist_hz = norm(r_tgt(1:2));
    vz = v(3); v_h = v(1:2);

    % ============ 制导律（三约束：速度+位置+姿态）============
    st.r = r; st.v = v; st.a_net = a_net;
    st.att = att; st.om = om;                 % 姿态状态进制导（末段姿态回正用）
    [a_cmd, d] = guide_3c(st, P, veh);
    t_go = d.t_go; phase = d.phase;

    % ============ 执行：推力分配（竖直优先）============
    a_thr = a_cmd + upv*g;
    t_up = min(max(0, a_thr(3)), veh.Tmax/m);
    t_h2 = a_thr(1:2);
    t_h_max = sqrt(max(0, (veh.Tmax/m)^2 - t_up^2));
    if norm(t_h2) > t_h_max, t_h2 = t_h2/norm(t_h2)*t_h_max; end
    a_thr_act = [t_h2; t_up];
    thr_mag = norm(a_thr_act);

    % 期望推力方向（小角近似转回 3D）
    if thr_mag > 0.001
        pl_want = a_thr_act / thr_mag;
    else
        pl_want = pl_dir;
    end
    % 姿态参考限速
    ang = acosd(max(-1,min(1,dot(pl_dir,pl_want))));
    if ang <= veh.ref_wmax*dt
        pl_dir = pl_want;
    else
        pl_dir = pl_dir + (pl_want-pl_dir)*(veh.ref_wmax*dt/ang);
        pl_dir = pl_dir/norm(pl_dir);
    end
    % 期望姿态小角（pl_dir 相对 up）
    att_des = pl_dir(1:2);   % 小角：水平分量 ≈ 偏角 [rad]

    % ============ 姿态二阶动力学（转动惯量 + 带宽 + 气动干扰）============
    % 姿态环：二阶跟踪 att_des，带宽 att_bw、阻尼 att_zeta
    % 气动干扰力矩 ∝ 空速 × 攻角（这里用 |v| × 当前偏角近似）
    vspd = norm(v);
    tau_aero = veh.aero_q * vspd * att;       % 气动回正/发散力矩（简化）
    acc_om = veh.att_bw^2*(att_des - att) - 2*veh.att_zeta*veh.att_bw*om + tau_aero/veh.Iyy;
    om = om + acc_om*dt;
    att = att + om*dt;
    % 实际推力方向 = up 偏 att（小角）
    att_dir = [att(1); att(2); 1]; att_dir = att_dir/norm(att_dir);

    % 实际加速度 = 推力沿实际方向 + 重力
    a_real = att_dir*thr_mag - upv*g;
    v = v + a_real*dt; r = r + v*dt;
    m = m - thr_mag*m/(veh.Isp*veh.g0)*dt;

    % 记录
    R.t(k)=tk; R.r(:,k)=r; R.v(:,k)=v; R.m(k)=m;
    R.thr(k)=thr_mag/(veh.Tmax/m);
    R.tilt(k)=atan2d(norm(a_thr_act(1:2)), a_thr_act(3));
    R.att_err(k)=norm(att_des - att)*57.2958;
    R.om(:,k)=om; R.phase(k)=phase;
    if mod(k, round(2/dt))==1
        fprintf('  t=%5.1f h=%7.1f vz=%7.1f vh=%6.1f d=%7.1f tgo=%4.1f thr=%.2f tilt=%4.1f ph=%d att_err=%4.2f\n', ...
            tk, r(3), vz, norm(v_h), dist_hz, t_go, R.thr(k), R.tilt(k), phase, R.att_err(k));
    end
    if r(3) <= tgt.alt_true
        R.t=R.t(1:k); R.r=R.r(:,1:k); R.v=R.v(:,1:k); R.m=R.m(1:k);
        R.thr=R.thr(1:k); R.tilt=R.tilt(1:k); R.att_err=R.att_err(1:k); R.phase=R.phase(1:k);
        tl = norm(att)*57.2958;
        if R.v(3,end) > -5 && norm(R.v(1:2,end)) < 5 && tl < 10
            R.verdict = sprintf('软着陆达标(倾角%.1f°)', tl);
        elseif R.v(3,end) > -15
            R.verdict = sprintf('接地偏重(倾角%.1f°)', tl);
        else
            R.verdict = '坠毁';
        end
        fprintf('【结果】dist=%.1f vz=%.2f vh=%.2f 倾角=%.1f° 余油=%.1f t  %s\n', ...
            norm(r(1:2)), v(3), norm(v(1:2)), tl, m/1000, R.verdict);
        return;
    end
end
R.verdict = '超时未落地';
fprintf('【结果】%s\n', R.verdict);
end
