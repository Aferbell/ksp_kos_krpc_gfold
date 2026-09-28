% sim_landing.m — 第六趟点火状态下的单点仿真（v6 对照 / v7 重构）
% 用法： sim_landing('v7') | sim_landing('v6') | sim_landing('all')
function sim_landing(mode)
if nargin < 1, mode = 'v7'; end
P = sim_params;
% 初始条件：land_log_47819000 idx=128（点火帧）
tgt_lat = -0.0972060951675948; tgt_lon = -74.5576822740041;
Re = 600000; deg2rad = pi/180;
T6 = readtable('D:\SteamLibrary\steamapps\common\Kerbal Space Program\Ships\Script\log-B1040-9\land_log_47819000.csv');
i0 = 129;
E0 = (T6.geo_lng(i0)-tgt_lon)*deg2rad*cos(tgt_lat*deg2rad)*Re;
N0 = (T6.geo_lat(i0)-tgt_lat)*deg2rad*Re;
dE = (T6.geo_lng(141)-T6.geo_lng(129))*deg2rad*cos(tgt_lat*deg2rad)*Re;
dN = (T6.geo_lat(141)-T6.geo_lat(129))*deg2rad*Re;
hd = [dE; dN]; hd = hd/norm(hd);
s0.r = [E0; N0; T6.alt(i0)];
s0.v = [hd*T6.vh(i0); T6.vz(i0)];
s0.m = T6.mass(i0)*1000;

if strcmp(mode,'all'), modes = {'v6','v7'}; else, modes = {mode}; end
for k = 1:numel(modes)
    R = sim_core(s0, P.veh, P.tgt, P, modes{k}, 1);
    fprintf('【%s】触地: dist=%.1f m  vz=%.2f  vh=%.2f  tilt=%.1f°  燃料余量=%.1f t  %s\n', ...
        modes{k}, norm(R.r(1:2,end)), R.v(3,end), norm(R.v(1:2,end)), ...
        R.tilt(end), R.m(end)/1000, R.verdict);
end
end
