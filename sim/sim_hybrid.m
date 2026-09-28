function R = sim_hybrid(P_over)
% sim_hybrid -- eguidance+gfold hybrid closed-loop simulation (ASCII comments only)
% Log source auto-selected (newest in log-B1040-9).
if nargin < 1, P_over = struct(); end
P = sim_att_params;
f = fieldnames(P_over); for k=1:numel(f), P.(f{k}) = P_over.(f{k}); end

veh.Tmax = 12.8e6; veh.Isp = 315; veh.g0 = 9.81;
veh.Iyy = 1.8e4; veh.gimbal = 10.5;
veh.att_bw = 1.2; veh.att_zeta = 0.9;
veh.aero_q = 0.35; veh.ref_wmax = 12;

tgt.alt_true = 36.8; tgt.aim_alt = 15;
P.tgt_alt = tgt.alt_true; P.aim_alt = tgt.aim_alt;

tgt_lat = -0.0972060951675948; tgt_lon = -74.5576822740041;
Re = 600000; d2r = pi/180;

logdir = 'D:\SteamLibrary\steamapps\common\Kerbal Space Program\Ships\Script\log-B1040-9';
d = dir(fullfile(logdir, 'land_log_*.csv'));
if isempty(d), error('no land_log_*.csv in %s', logdir); end
[~, ix] = max([d.datenum]);
T = readtable(fullfile(logdir, d(ix).name));

ii = find([double(string(T.phase))] >= 1, 1, 'first');
if isempty(ii), ii = 1; end
E0 = (T.geo_lng(ii)-tgt_lon)*d2r*cos(tgt_lat*d2r)*Re;
N0 = (T.geo_lat(ii)-tgt_lat)*d2r*Re;
j2 = min(ii+12, height(T));
dE = (T.geo_lng(j2)-T.geo_lng(ii))*d2r*cos(tgt_lat*d2r)*Re;
dN = (T.geo_lat(j2)-T.geo_lat(ii))*d2r*Re;
hd = [dE; dN];
if norm(hd) < 1e-6, hd = [1;0]; end
hd = hd/norm(hd);
r = [E0; N0; T.alt(ii)];
v = [hd*T.vh(ii); T.vz(ii)];
m = T.mass(ii)*1000;

att = [0;0]; om = [0;0]; pl_dir = [0;0;1];
dt = 0.02; tmax = 300; nmax = round(tmax/dt);
R.t=zeros(1,nmax); R.r=zeros(3,nmax); R.v=zeros(3,nmax); R.m=zeros(1,nmax);
R.thr=zeros(1,nmax); R.tilt=zeros(1,nmax); R.att_err=zeros(1,nmax);
R.om=zeros(2,nmax); R.phase=zeros(1,nmax); R.cone=zeros(1,nmax);
phase = 1;

for k = 1:nmax
    tk = (k-1)*dt;
    upv = [0;0;1]; g = veh.g0;
    a_net = veh.Tmax/m - g;
    st.r = r; st.v = v; st.a_net = a_net; st.att = att; st.om = om;
    [a_cmd, d] = guide_hybrid(st, P, veh);
    t_go = d.t_go; phase = d.phase;

    a_thr = a_cmd + upv*g;
    t_up = min(max(0, a_thr(3)), veh.Tmax/m);
    t_h2 = a_thr(1:2);
    t_h_max = sqrt(max(0, (veh.Tmax/m)^2 - t_up^2));
    if norm(t_h2) > t_h_max, t_h2 = t_h2/norm(t_h2)*t_h_max; end
    a_thr_act = [t_h2; t_up];
    thr_mag = norm(a_thr_act);

    if thr_mag > 0.001, pl_want = a_thr_act/thr_mag; else, pl_want = pl_dir; end
    ang = acosd(max(-1,min(1,dot(pl_dir,pl_want))));
    if ang <= veh.ref_wmax*dt
        pl_dir = pl_want;
    else
        pl_dir = pl_dir + (pl_want-pl_dir)*(veh.ref_wmax*dt/ang);
        pl_dir = pl_dir/norm(pl_dir);
    end
    att_des = pl_dir(1:2);

    vspd = norm(v);
    tau_aero = veh.aero_q * vspd * att;
    acc_om = veh.att_bw^2*(att_des - att) - 2*veh.att_zeta*veh.att_bw*om + tau_aero/veh.Iyy;
    om = om + acc_om*dt;
    att = att + om*dt;
    att_dir = [att(1); att(2); 1]; att_dir = att_dir/norm(att_dir);

    a_real = att_dir*thr_mag - upv*g;
    v = v + a_real*dt; r = r + v*dt;
    m = m - thr_mag*m/(veh.Isp*veh.g0)*dt;

    R.t(k)=tk; R.r(:,k)=r; R.v(:,k)=v; R.m(k)=m;
    R.thr(k)=thr_mag/(veh.Tmax/m);
    R.tilt(k)=atan2d(norm(a_thr_act(1:2)), a_thr_act(3));
    R.att_err(k)=norm(att_des-att)*57.2958;
    R.om(:,k)=om; R.phase(k)=phase; R.cone(k)=d.cone_marg;

    if mod(k, round(2/dt))==1
        fprintf('  t=%5.1f h=%7.1f vz=%7.1f vh=%6.1f d=%7.1f tgo=%5.1f thr=%.2f tilt=%5.1f ph=%d cone=%7.1f\n', ...
            tk, r(3), v(3), norm(v(1:2)), norm(r(1:2)), t_go, R.thr(k), R.tilt(k), phase, d.cone_marg);
    end
    if r(3) <= tgt.alt_true
        R.t=R.t(1:k); R.r=R.r(:,1:k); R.v=R.v(:,1:k); R.m=R.m(1:k);
        R.thr=R.thr(1:k); R.tilt=R.tilt(1:k); R.att_err=R.att_err(1:k);
        R.phase=R.phase(1:k); R.cone=R.cone(1:k);
        tl = norm(att)*57.2958;
        dist = norm(r(1:2));
        if R.v(3,end) > -5 && norm(R.v(1:2,end)) < 5 && tl < 5
            R.verdict = sprintf('PASS(dist%.2f tilt%.1f)', dist, tl);
        elseif R.v(3,end) > -15
            R.verdict = 'hard-touchdown';
        else
            R.verdict = 'CRASH';
        end
        fprintf('[result] dist=%.1f m  vz=%.2f  vh=%.2f  tilt=%.1f deg  fuel=%.1f t  %s\n', ...
            dist, v(3), norm(v(1:2)), tl, m/1000, R.verdict);
        return;
    end
end
R.verdict = 'timeout';
fprintf('[result] %s\n', R.verdict);
end
