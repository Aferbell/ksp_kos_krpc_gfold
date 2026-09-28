function P = sim_att_params
% sim_att_params �� Apollo ����ʽ�����Ƶ��������� boot/B1040-9.ks ����������룩
% --- ʱ��/���� ---
P.eg_tmin = 4.0;           % ʱ������ [s]
P.eg_ad = 0.55;            % ��ֱ�������ϵ��
P.eg_lat_max = 60;         % ���κ���ָ������ [m/s2]
P.gamma_gs = 48;
P.tilt_hi = 55; P.tilt_lo = 25; P.tilt_taper = 400;
% --- ����ʽ�ţ���ʵ�ؾ߱궨��---
P.gate_h = 180;            % ��1����2 �� [m]
P.vh_gate = 35; P.dist_gate = 150; P.vz_gate = 120;
P.p66_h = 30;              % ��2����3 �� [m]
% --- ���κ��� ---
P.tan_cap = 15;
P.cone_gain = 0.5; P.cone_gain_max = 8;
P.lat_pd_kp = 0.8; P.lat_pd_kd = 1.8;   % ���ֶ�λ�� PD��dist��lat_hold ʱ��ס��
P.lat_hold = 1200;
P.lat_tmin = 2.0;                        % ����ʱ������ [s]�������� eg_T �����ޣ�v8 ��������
P.eg_a_v = 18.6;            % ��1 ���ֱ�����ʣ���ʵ��־��ϣ�[m/s2]
P.eg_a_h = 23.8;            % ��1 ���������ʣ���ʵ��־��ϣ�[m/s2]
P.eg_kv = 0.35;             % ��1 ��������������� [1/s]
P.gfold_h = 5000;           % kRPC/G-FOLD ����߶��� [m]
P.eg_t_ref = 6.0;            % ��1 ����ο�ʱ�� [s]��dist/t ���ޣ������Ŀ�꣩
P.eg_ad_a = 0.55;
P.eg_k_steer = 0.9;           % cross-range steering gain (x a_net)           % ��1 �ƶ�������ٱȣ��� a_net���������ã�
P.eg_lat_min = 12;                       % ��1 ��ֱָ�������Ԥ������С��� [m/s2]
P.eg_ad_suicide = 0.45;                  % ��1 �ƶ�������ٱȣ��� a_net��ʹ���ʱ vz_want��ʵ�ʣ�
P.eg_tau_v = 0.8;                        % ��1 ��ֱһ�׸���ʱ�䳣�� [s]
% --- ��2/��3 ---
P.term_v = 2.0; P.term_k = 2; P.term_kp = 0.8; P.term_kd = 1.8;
P.term_amax = 20; P.term_v_slow = 40; P.term_h_margin = 1.3; P.term_h_reserve = 8;
P.eg_kp = 0.8; P.eg_kd = 1.8; P.eg_amax = 20;
P.att_tilt5 = 5.0;         % ��3 ���������޷� [deg]
P.att_h = 60;              % ��̬������ʼ�߶� [m]
P.term_tilt = 20; P.tilt_min = 6; P.tilt_span = 120;
% --- �ؾ�/Ŀ���� sim_att �ṩ ---
end
