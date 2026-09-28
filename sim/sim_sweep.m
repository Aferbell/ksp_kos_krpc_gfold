% sim_sweep.m — v7 制导律多初始条件鲁棒性扫描（单一实现 guide_v7）
% 用法：sim_sweep
function sim_sweep
P = sim_params;
tgt_lat = -0.0972060951675948; tgt_lon = -74.5576822740041;
Re = 600000; deg2rad = pi/180;
T6 = readtable('D:\SteamLibrary\steamapps\common\Kerbal Space Program\Ships\Script\log-B1040-9\land_log_47819000.csv');
i0 = 129;
E0 = (T6.geo_lng(i0)-tgt_lon)*deg2rad*cos(tgt_lat*deg2rad)*Re;
N0 = (T6.geo_lat(i0)-tgt_lat)*deg2rad*Re;
dE = (T6.geo_lng(141)-T6.geo_lng(129))*deg2rad*cos(tgt_lat*deg2rad)*Re;
dN = (T6.geo_lat(141)-T6.geo_lat(129))*deg2rad*Re;
hd = [dE; dN]; hd = hd/norm(hd);

cases = struct('name', {}, 'r', {}, 'v', {}, 'm', {});
add = @(nm,r,vv,m) deal(0); % placeholder (avoid anonymous struct grow complexity)
cases(1).name = '基准(第六趟点火)';
cases(1).r = [E0; N0; T6.alt(i0)];
cases(1).v = [hd*T6.vh(i0); T6.vz(i0)];
cases(1).m = T6.mass(i0)*1000;
for sgn = [1,-1]
    ang10 = sgn*10*pi/180;
    hd2 = [cos(ang10)*hd(1)-sin(ang10)*hd(2); sin(ang10)*hd(1)+cos(ang10)*hd(2)];
    cases(end+1).name = sprintf('速度方向偏 %+d°', sgn*10);
    cases(end).r = [E0; N0; T6.alt(i0)];
    cases(end).v = [hd2*T6.vh(i0); T6.vz(i0)];
    cases(end).m = T6.mass(i0)*1000;
end
for s = [1.2, 0.8]
    cases(end+1).name = sprintf('dist x%.1f', s);
    cases(end).r = [E0*s; N0*s; T6.alt(i0)];
    cases(end).v = [hd*T6.vh(i0); T6.vz(i0)];
    cases(end).m = T6.mass(i0)*1000;
end
cases(end+1).name = '|v| +5%';
cases(end).r = [E0; N0; T6.alt(i0)];
cases(end).v = [hd*T6.vh(i0)*1.05; T6.vz(i0)*1.05];
cases(end).m = T6.mass(i0)*1000;
cases(end+1).name = '横向15km(域外)';
cases(end).r = [E0*1.33; N0*1.33; T6.alt(i0)];
cases(end).v = [hd*T6.vh(i0); T6.vz(i0)];
cases(end).m = T6.mass(i0)*1000;

fprintf('算例 | 触地 dist [m] | vz | vh | tilt | 余油 [t] | 判定\n');
for k = 1:numel(cases)
    s0.r = cases(k).r; s0.v = cases(k).v; s0.m = cases(k).m;
    R = sim_core(s0, P.veh, P.tgt, P, 'v7', 0);
    fprintf('%-18s | %8.1f | %7.2f | %6.2f | %5.1f | %6.1f | %s\n', ...
        cases(k).name, norm(R.r(1:2,end)), R.v(3,end), norm(R.v(1:2,end)), ...
        R.tilt(end), R.m(end)/1000, R.verdict);
end
end
