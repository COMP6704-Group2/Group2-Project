%% ------------------------------------------------------------
%  EMS with PV, Wind, BESS solved by linprog (interior-point / dual-simplex)
%  Variables order (NO PV/Wind in x now):
%     x = [Pbuy(1:T); Psell(1:T); Pch(1:T); Pdis(1:T); E(1:T)]
%  Objective: min sum_t (pi_buy*Pbuy - pi_sell*Psell + c_sto*(Pch+Pdis))
%  Constraints: Power balance (PV/Wind must-take), BESS dynamics, terminal SoC, bounds
%  + timing
% ------------------------------------------------------------
clear; clc;

%% ---------------- Switch: choose algorithm ----------------
% 'interior-point'  or  'dual-simplex'
LP_ALG = 'interior-point';  % <== 改这里可切换

%% ---------------- Time and data ----------------
T  = 24;   h = 1;

PV_avail = [0 0 0 0 0 0 1.28 4.92 6.26 9.08 11.14 11.34 9.44 6.70 4.34 2.12 0.22 0 0 0 0 0 0 0]'.*1.2;
W_avail  = [13 13.5 14 13.5 12.5 11 10 9 8 7.5 7 7.5 9 8.5 8 7 7.4 8.5 10 10.5 11 12 12.5 12.7]'-2;

Load = [18 17 16 16 16 17 19 22 24 26 28 29 30 30 29 28 27 26 25 24 23 22 21 20]'*0.5;

pi_buy  = 60;
pi_sell = 0.8*pi_buy;

E_max   = 60;  E_min = 6;
P_ch_max  = 20; P_dis_max = 20;
eta_ch  = 0.95; eta_dis = 0.95;
E0      = 0.50*E_max;
c_sto   = 2;

%% ---------------- Variable indexing (NO PV/Wind) ----------------
% x = [Pbuy; Psell; Pch; Pdis; E]
n = 5*T;
ixPbuy  = 1:T;
ixPsell = T+(1:T);
ixPch   = 2*T+(1:T);
ixPdis  = 3*T+(1:T);
ixE     = 4*T+(1:T);

%% ---------------- Objective f ----------------
f = zeros(n,1);
f(ixPbuy)  = pi_buy;
f(ixPsell) = -pi_sell;
f(ixPch)   = c_sto;
f(ixPdis)  = c_sto;

%% ---------------- Bounds ----------------
lb = zeros(n,1);
ub = inf(n,1);
ub(ixPbuy)  = 80;
ub(ixPsell) = 80;
ub(ixPch)   = P_ch_max;
ub(ixPdis)  = P_dis_max;
lb(ixE)     = E_min;  ub(ixE) = E_max;

%% ---------------- Equality constraints Aeq x = beq ----------------
% Rows: T (power balance) + T (dynamics) + 1 (terminal) = 2T+1
Aeq = spalloc(2*T+1, n, 10*T);
beq = zeros(2*T+1,1);
row = 0;

% (1) Power balance  ——  Pbuy - Psell + Pdis - Pch + (PV+W) = Load
%  =>  Pbuy - Psell + Pdis - Pch = Load - PV - W
for t = 1:T
    row = row + 1;
    Aeq(row, ixPbuy(t))  =  1;
    Aeq(row, ixPsell(t)) = -1;
    Aeq(row, ixPdis(t))  =  1;
    Aeq(row, ixPch(t))   = -1;
    beq(row) = Load(t) - PV_avail(t) - W_avail(t);
end

% (2) BESS dynamics
% E1 = E0 + h*(eta_ch*Pch1 - (1/eta_dis)*Pdis1)
row = row + 1;
Aeq(row, ixE(1))    =  1;
Aeq(row, ixPch(1))  = -h*eta_ch;
Aeq(row, ixPdis(1)) =  h/eta_dis;
beq(row) = E0;

% Et - Et-1 = h*(eta_ch*Pch_t - (1/eta_dis)*Pdis_t),  t=2..T
for t = 2:T
    row = row + 1;
    Aeq(row, ixE(t))    =  1;
    Aeq(row, ixE(t-1))  = -1;
    Aeq(row, ixPch(t))  = -h*eta_ch;
    Aeq(row, ixPdis(t)) =  h/eta_dis;
    beq(row) = 0;
end

% (3) Terminal SoC: E_T = E0
row = row + 1;
Aeq(row, ixE(T)) = 1;
beq(row) = E0;

A = []; b = [];

%% ---------------- linprog options ----------------
opts = optimoptions('linprog', ...
    'Algorithm', LP_ALG, ...
    'Display','iter');

t_total = tic;
[x, fval, exitflag, output] = linprog(f, A, b, Aeq, beq, lb, ub, opts);
total_time = toc(t_total);

assert(exitflag > 0, 'linprog failed: %s', output.message);

%% ---------------- Unpack solution ----------------
Pbuy_v  = x(ixPbuy);
Psell_v = x(ixPsell);
Pch_v   = x(ixPch);
Pdis_v  = x(ixPdis);
E_v     = x(ixE);

% PV / Wind 不再是变量，直接用可用值（必须全额消纳）
Ppv_v   = PV_avail;
Pwt_v   = W_avail;

cost_v  = fval;
fprintf('Optimal operating cost: %.2f $\n', cost_v);
fprintf('Total  time      : %.6f s\n', total_time);

%% ---------------- Plots (保持你原来的风格) ----------------
t = 1:T;

% ========== Figure 1: PV, Wind, and Load ==========
figure('Color','w'); hold on; box on;
p1 = plot(t, Ppv_v, '-^', 'LineWidth',2, 'MarkerSize',5, ...
          'Color',[1.00 0.6 0.20], 'MarkerFaceColor',[1.00 0.75 0.45]);
p2 = plot(t, Pwt_v, '-s', 'LineWidth',2, 'MarkerSize',5, ...
          'Color',[0.35 0.70 0.95], 'MarkerFaceColor',[0.65 0.85 1]);
p3 = plot(t, Load,  '-v', 'LineWidth',2, 'MarkerSize',6, ...
          'Color',[0.70 0.55 0.80], 'MarkerFaceColor',[0.82 0.70 0.88]);
xlabel('Hour'); ylabel('Power (MW)'); title('PV, Wind, and Load');
legend([p1 p2 p3], {'PV','Wind','Load'}, 'Location','best');
grid on; box on; set(gca,'FontName','Times New Roman','FontSize',12);
ylim([0 18]);

% ========== Figure 2: Power Balance (bidirectional stacked bars) ==========
pos_stack = [Ppv_v, Pwt_v, Pdis_v, Pbuy_v];
neg_stack = [Pch_v, Psell_v];
neg_plot  = -neg_stack;

figure('Color','w'); hold on; box on;
b1 = bar(t, pos_stack, 'stacked', 'BarWidth', 0.8);
b2 = bar(t, neg_plot,  'stacked', 'BarWidth', 0.8);
set(b1(1),'FaceColor',[1.00 0.6 0.20]);
set(b1(2),'FaceColor',[0.35 0.70 0.95]);
set(b1(3),'FaceColor',[0.45 0.80 0.35]);
set(b1(4),'FaceColor',[0.93 0.40 0.10]);
set(b2(1),'FaceColor',[0.30 0.80 0.85]);
set(b2(2),'FaceColor',[0.70 0.55 0.80]);
plot(t, Load, '-.^','Color','k', 'LineWidth', 1);
yline(0,'k','LineWidth',1.2);
xlabel('Hour'); ylabel('Power (MW)'); title('Power Balance');
legend({'PV','Wind','BESS Discharge','Grid Import','BESS Charge','Grid Export','Load'}, ...
       'Location','northoutside','Orientation','horizontal');
grid on;

% ========== Figure 3: BESS SoC (brown line + markers) + Charge/Discharge (bars) ==========
figure('Color','w'); hold on; box on;
yyaxis right
bDis = bar(t,  -Pdis_v, 0.55, 'FaceColor',[0.45 0.80 0.35], 'EdgeColor','none'); hold on;
bCha = bar(t,   Pch_v,  0.55, 'FaceColor',[0.30 0.80 0.85], 'EdgeColor','none');
set([bDis bCha],'FaceAlpha',0.70);
yline(0,'k','LineWidth',1.2);
ylabel('Power (MW)');
Pmax = 1.2*max([max(Pdis_v), max(Pch_v), 1e-6]); ylim([-Pmax, Pmax]);
yyaxis left
span_left = 2*max(E_max - E0, E0 - E_min);
ylim([E0 - span_left/2, E0 + span_left/2]);
pSoC = plot(t, E_v, '-o', ...
    'Color',[0.65 0.33 0.10], 'LineWidth',2, ...
    'MarkerSize',4.5, 'MarkerFaceColor',[1 1 1], 'MarkerEdgeColor',[0.65 0.33 0.10]);
ylabel('Energy (MWh)'); xlabel('Hour');
title('BESS SoC (brown line) and Charge/Discharge (bars) ');
legend([pSoC  bDis bCha ], ...
       {'SoC','Discharge','Charge'}, ...
       'Location','northoutside','Orientation','horizontal');
grid on; box on; set(gca,'FontName','Times New Roman','FontSize',12);
