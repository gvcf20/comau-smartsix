% =========================================================
%  circulo.m
%  ELE041/EEE935 — Manipuladores Roboticos — UFMG
%
%  Itens xi a xii:
%      Regulacao:  P0 -> P6
%      Seguimento: Circulo completo anti-horario a partir de P6
%      Regulacao:  P6 -> P0
% =========================================================
clear; clc; close all;
addpath('utils');

%% ============================================================
%  MODELO
%% ============================================================
[Hbase, CS6p_raw] = CS6bot();
theta_off = CS6p_raw(1:6);
d_vec     = CS6p_raw(7:12);
a_vec     = CS6p_raw(13:18);
alpha_vec = CS6p_raw(19:24);
for i = 1:6
    L(i) = Link('d', d_vec(i), 'a', a_vec(i), ...
                'alpha', alpha_vec(i), 'offset', theta_off(i), 'standard');
end
CS6 = SerialLink(L, 'name', 'COMAU SmartSix');
CS6.base = Hbase;

%% ============================================================
%  PARAMETROS
%% ============================================================
dt    = 0.01;
K     = 2.0;    % ganho unico (posicao e orientacao)
lam   = 0.01;   % amortecimento para pseudo-inversa
N_reg = 1000;   % iteracoes regulacao  (10s)
N_seg = 2000;   % iteracoes seguimento (20s) para o circulo completo
mm    = 1e-3;

%% ============================================================
%  ORIENTACAO DESEJADA (item iii)
%% ============================================================
Rd = [ 0,  0,  1;
       0,  1,  0;
      -1,  0,  0];

%% ============================================================
%  PONTOS E GEOMETRIA DO CIRCULO
%% ============================================================
P0 = [700; 0; 650] * mm;
C  = [1000; 0; 650] * mm; % Centro geométrico do círculo

% Cálculo do raio do círculo tangente ao losango
a = 600; % semi-eixo em Y
b = 350; % semi-eixo em Z
R = (a * b) / sqrt(a^2 + b^2); % Raio em milímetros (~302.32 mm)
R_m = R * mm;                  % Raio em metros

% Ponto inicial P6 (Parte inferior do círculo)
P6 = C + [0; 0; -R_m]; 

%% ============================================================
%  ERRO DE ORIENTACAO
%% ============================================================
function eo = orient_err(Rd, Re)
    nphi = rotm2axang2(Rd * Re');   
    eo   = (nphi(1:3) * nphi(4))'; 
end

%% ============================================================
%  REGULACAO
%% ============================================================
function [q_out, Q, P] = regulacao(CS6, q0, Rd, pd, N, K, dt, lam)
    Q = zeros(N, 6);  P = zeros(N, 3);  q = q0(:);
    for k = 1:N
        T  = CS6.fkine(q'); if isobject(T), T = T.T; end
        p  = T(1:3, 4);
        R_mat = T(1:3, 1:3);

        p_err   = pd - p;
        eo      = orient_err(Rd, R_mat);
        e       = [p_err; eo];

        J = CS6.jacob0(q');
        u = (J' / (J*J' + lam^2*eye(6))) * (K * e);

        q = q + dt * u;
        Q(k,:) = q';
        P(k,:) = p';
    end
    q_out = q';
end

%% ============================================================
%  SEGUIMENTO CIRCULAR COM FEEDFORWARD
%% ============================================================
function [q_out, Q, P] = seguimento_circular(CS6, q0, Rd, C, R, N, K, dt, lam)
    Q = zeros(N, 6);  P = zeros(N, 3);  q = q0(:);
    omega = (2*pi) / (N * dt); % Velocidade angular constante necessária
    
    for k = 1:N
        T  = CS6.fkine(q'); if isobject(T), T = T.T; end
        p  = T(1:3, 4);
        R_mat = T(1:3, 1:3);

        s   = (k-1)/(N-1);
        ang = 2 * pi * s; % Ângulo varia de 0 a 2*pi
        
        % Posição desejada na circunferência (sentido anti-horário começando debaixo)
        % Y(t) = R * sen(ang) | Z(t) = -R * cos(ang)
        pd = C + [0; R*sin(ang); -R*cos(ang)];
        
        % Velocidade feedforward cartesiana tangencial
        v_ff = [0; R*omega*cos(ang); R*omega*sin(ang); 0; 0; 0];
        
        p_err   = pd - p;
        eo      = orient_err(Rd, R_mat);
        e       = [p_err; eo];

        J = CS6.jacob0(q');
        u = (J' / (J*J' + lam^2*eye(6))) * (v_ff + K * e);

        q = q + dt * u;
        Q(k,:) = q';
        P(k,:) = p';
    end
    q_out = q';
end

%% ============================================================
%  HELPER: imprime erro ao final de cada movimento
%% ============================================================
function report(label, CS6, q, pd, Rd)
    T  = CS6.fkine(q); if isobject(T), T = T.T; end
    ep = norm(T(1:3,4) - pd) * 1e3;
    nphi = rotm2axang2(Rd * T(1:3,1:3)');
    eo   = norm(nphi(1:3) * nphi(4));
    fprintf('%-8s | pos: [%6.1f %6.1f %6.1f] mm | err_p: %.3f mm | err_o: %.6f\n', ...
        label, T(1:3,4)'*1e3, ep, eo);
end

%% ============================================================
%  SEQUENCIA DE MOVIMENTOS
%% ============================================================
q     = deg2rad([0, 0, -90, 0, -90, 0]);
Q_all = q;
P_all = [];

% q0 -> P0  (regulacao)
fprintf('[REG] q0 -> P0 ... ');
[q, Qh, Ph] = regulacao(CS6, q, Rd, P0, N_reg, K, dt, lam);
Q_all = [Q_all; Qh];  P_all = [P_all; Ph];
report('P0', CS6, q, P0, Rd);

% P0 -> P6  (regulacao)
fprintf('[REG] P0 -> P6 ... ');
[q, Qh, Ph] = regulacao(CS6, q, Rd, P6, N_reg, K, dt, lam);
Q_all = [Q_all; Qh];  P_all = [P_all; Ph];
report('P6', CS6, q, P6, Rd);

% Circulo completo (seguimento com feedforward)
fprintf('[TRAJ] Circulo Anti-Horario ... ');
[q, Qh, Ph] = seguimento_circular(CS6, q, Rd, C, R_m, N_seg, K, dt, lam);
Q_all = [Q_all; Qh];  P_all = [P_all; Ph];
report('P6_fim', CS6, q, P6, Rd); % Volta para P6 ao final dos 360 graus

% P6 -> P0  (regulacao)
fprintf('[REG] P6 -> P0 ... ');
[q, Qh, Ph] = regulacao(CS6, q, Rd, P0, N_reg, K, dt, lam);
Q_all = [Q_all; Qh];  P_all = [P_all; Ph];
report('P0', CS6, q, P0, Rd);

%% ============================================================
%  q_seq para CoppeliaSim (6 x n, graus)
%% ============================================================
q_seq = rad2deg(Q_all)';
fprintf('\nq_seq: %d juntas x %d amostras\n', size(q_seq,1), size(q_seq,2));
if ~exist('results','dir'), mkdir('results'); end
save('results/q_seq_circulo.mat', 'q_seq');

%% ============================================================
%  FIGURA 1 — Animacao 3D
%% ============================================================
figure('Color','white','Position',[100 100 900 700]);
CS6.plot(Q_all(1:5:end,:), ...
    'workspace', [-1.5 1.5 -1.5 1.5 -1.2 2.0], ...
    'jointdiam', 1, 'fps', 25, 'trail', 'r-', 'noname');
ax = gca;
ax.Color = 'white';  ax.XColor = 'black';
ax.YColor = 'black'; ax.ZColor = 'black';
set(gcf, 'Color', 'white');
hold(ax, 'on');
L_ax = 0.3;
quiver3(ax,0,0,0,L_ax,0,0,'r','LineWidth',2,'AutoScale','off','MaxHeadSize',0.5);
quiver3(ax,0,0,0,0,L_ax,0,'g','LineWidth',2,'AutoScale','off','MaxHeadSize',0.5);
quiver3(ax,0,0,0,0,0,L_ax,'b','LineWidth',2,'AutoScale','off','MaxHeadSize',0.5);
text(ax,L_ax+0.03,0,0,      'X_b','Color','r','FontWeight','bold','FontSize',11);
text(ax,0,L_ax+0.03,0,      'Y_b','Color','g','FontWeight','bold','FontSize',11);
text(ax,0,0,L_ax+0.03,      'Z_b','Color','b','FontWeight','bold','FontSize',11);
title(ax,'COMAU Smart SiX — Circulo','FontSize',13);

%% ============================================================
%  FIGURA 2 — Caminho no plano YZ
%% ============================================================
x_pl = 1000*mm;
tol  = 30*mm;
idx  = abs(P_all(:,1) - x_pl) < tol;

figure('Color','white','Position',[100 80 800 650]);
hold on; grid on; axis equal; box on;
plot(P_all(:,2)*1e3, P_all(:,3)*1e3, '-','Color',[0.7 0.7 0.7],'LineWidth',1);
if any(idx)
    plot(P_all(idx,2)*1e3, P_all(idx,3)*1e3, 'b-','LineWidth',2.5);
end

% Plotando o circulo ideal para referencia
theta_plot = linspace(0, 2*pi, 100);
circ_y = C(2)*1e3 + R * sin(theta_plot);
circ_z = C(3)*1e3 - R * cos(theta_plot);
plot(circ_y, circ_z, 'g--','LineWidth',1.5);

pts  = {P0, P6};
nms  = {'P0','P6'};
mkrs = {'ko','rs'};
for i = 1:2
    p = pts{i};
    plot(p(2)*1e3,p(3)*1e3,mkrs{i},'MarkerSize',9,'LineWidth',2);
    text(p(2)*1e3+20,p(3)*1e3+15,nms{i},'FontSize',10,'FontWeight','bold');
end
xlabel('Y (mm)','FontSize',12); ylabel('Z (mm)','FontSize',12);
title('Circulo — Caminho do Efetuador no Plano YZ','FontSize',13);
legend({'Trajetoria completa','No plano x=1000mm','Circulo ideal'},'Location','best');
xlim([-750 750]); ylim([100 1150]);
saveas(gcf,'results/circulo_YZ.png');

fprintf('\n=== Concluido ===\n');