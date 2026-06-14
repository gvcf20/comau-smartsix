% =========================================================
%  losango.m
%  ELE041/EEE935 — Manipuladores Roboticos — UFMG
%  Prof. Gustavo Medeiros Freitas
%
%  Itens ix a x:
%      Regulacao:  P0 -> P5
%      Seguimento: P5->P5inf->P5dir->P5sup->P5 (anti-horário com feedforward)
%      Regulacao:  P5 -> P0
%
%  Controle cinematico via Jacobiana Geometrica e erro de
%  orientacao eixo-angulo (rotm2axang2), conforme control.m
%  disponibilizado no Moodle.
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
dt      = 0.015;   % passo de integracao 
K       = 2.0;     % ganho proporcional
N_reg   = 1000;    % iteracoes regulacao  
N_seg   = 500;     % iteracoes seguimento
epsilon = 1e-3;    % criterio de parada regulacao
mm      = 1e-3;

%% ============================================================
%  ORIENTACAO DESEJADA (item iii)
%  Ze_e (approach) = Xb,  Xe_e (normal) = -Zb
%% ============================================================
Rd = [ 0,  0,  1;
       0,  1,  0;
      -1,  0,  0];

%% ============================================================
%  PONTOS DO LOSANGO
%% ============================================================
P0    = [700;    0;  650] * mm;
P5    = [1000; -600;  650] * mm; % Vertice Esquerdo
P5inf = [1000;    0;  300] * mm; % Vertice Inferior
P5dir = [1000;  600;  650] * mm; % Vertice Direito
P5sup = [1000;    0; 1000] * mm; % Vertice Superior

%% ============================================================
%  FUNCOES LOCAIS
%% ============================================================
function T = getT(Tin)
% Padroniza a saida da fkine para matriz homogenea 4x4.
    if isobject(Tin)
        T = Tin.T;
    else
        T = Tin;
    end
end

function [erro_ori, erro_rpy] = erroOrientacao(Rd, R_atual)
% Calcula erro de orientacao em eixo-angulo (para controle)
% e em RPY (para analise grafica)
    R_erro = Rd * R_atual';
    axang  = rotm2axang2(R_erro);
    eixo   = axang(1:3)';
    phi    = axang(4);
    erro_ori = eixo * phi;
    erro_rpy = tr2rpy(R_erro);
    erro_rpy = erro_rpy(:)';
end

%% ============================================================
%  REGULACAO
%  Lei de controle: u = pinv(J) * K * e   
%  Criterio de parada: norm(e) < epsilon
%% ============================================================
function [q_out, Q, P] = regulacao(CS6, q0, Rd, pd, N, K, dt, epsilon)
    Q = zeros(N, 6);  P = zeros(N, 3);  q = q0(:);
    k_final = N;
    for k = 1:N
        T = getT(CS6.fkine(q'));
        p = T(1:3, 4);
        R = T(1:3, 1:3);

        erro_pos = pd - p;
        [erro_ori, ~] = erroOrientacao(Rd, R);
        e = [erro_pos; erro_ori];

        if norm(e) < epsilon
            k_final = k;
            Q(k,:) = q';
            P(k,:) = p';
            break;
        end

        J = CS6.jacob0(q');
        u = pinv(J) * (K * e);

        q = q + dt * u;
        Q(k,:) = q';
        P(k,:) = p';
    end
    Q     = Q(1:k_final, :);
    P     = P(1:k_final, :);
    q_out = q';
end

%% ============================================================
%  SEGUIMENTO LINEAR COM FEEDFORWARD
%  Lei de controle: u = pinv(J) * (v_ff + K * e)
%% ============================================================
function [q_out, Q, P] = seguimento(CS6, q0, Rd, Pa, Pb, N, K, dt)
    Q    = zeros(N, 6);  P = zeros(N, 3);  q = q0(:);
    v_ff = [(Pb - Pa) / (N * dt); zeros(3,1)];
    for k = 1:N
        T = getT(CS6.fkine(q'));
        p = T(1:3, 4);
        R = T(1:3, 1:3);

        s   = (k-1)/(N-1);
        pd  = Pa + s*(Pb - Pa);
        erro_pos = pd - p;
        [erro_ori, ~] = erroOrientacao(Rd, R);
        e = [erro_pos; erro_ori];

        J = CS6.jacob0(q');
        u = pinv(J) * (v_ff + K * e);

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
    T  = getT(CS6.fkine(q));
    ep = norm(T(1:3,4) - pd) * 1e3;
    [eo, ~] = erroOrientacao(Rd, T(1:3,1:3));
    fprintf('%-8s | pos: [%6.1f %6.1f %6.1f] mm | err_p: %.3f mm | err_o: %.6f\n', ...
        label, T(1:3,4)'*1e3, ep, norm(eo));
end

%% ============================================================
%  SEQUENCIA DE MOVIMENTOS
%% ============================================================
q     = deg2rad([0, 0, -90, 0, -90, 0]);
Q_all = q;
P_all = [];

% q0 -> P0  (regulacao)
fprintf('[REG] q0 -> P0 ... ');
[q, Qh, Ph] = regulacao(CS6, q, Rd, P0, N_reg, K, dt, epsilon);
Q_all = [Q_all; Qh];  P_all = [P_all; Ph];
report('P0', CS6, q, P0, Rd);

% P0 -> P5  (regulacao)
fprintf('[REG] P0 -> P5 ... ');
[q, Qh, Ph] = regulacao(CS6, q, Rd, P5, N_reg, K, dt, epsilon);
Q_all = [Q_all; Qh];  P_all = [P_all; Ph];
report('P5', CS6, q, P5, Rd);

% P5->P5inf->P5dir->P5sup->P5  (seguimento com feedforward)
wpts = {P5, P5inf, P5dir, P5sup, P5};
lbls = {'P5->P5inf','P5inf->P5dir','P5dir->P5sup','P5sup->P5'};
fprintf('[TRAJ] Losango\n');
for s = 1:4
    fprintf('   %s ... ', lbls{s});
    [q, Qh, Ph] = seguimento(CS6, q, Rd, wpts{s}, wpts{s+1}, N_seg, K, dt);
    Q_all = [Q_all; Qh];  P_all = [P_all; Ph];
    report(lbls{s}(4:5), CS6, q, wpts{s+1}, Rd);
end

% P5 -> P0  (regulacao)
fprintf('[REG] P5 -> P0 ... ');
[q, Qh, Ph] = regulacao(CS6, q, Rd, P0, N_reg, K, dt, epsilon);
Q_all = [Q_all; Qh];  P_all = [P_all; Ph];
report('P0', CS6, q, P0, Rd);

%% ============================================================
%  q_seq para CoppeliaSim (6 x n, graus)
%% ============================================================
q_seq = rad2deg(Q_all)';
fprintf('\nq_seq: %d juntas x %d amostras\n', size(q_seq,1), size(q_seq,2));
if ~exist('results','dir'), mkdir('results'); end
save('results/q_seq_losango.mat', 'q_seq');

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
title(ax,'COMAU Smart SiX — Losango','FontSize',13);

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
los_y = [-600  0 600 0  -600];
los_z = [650 300  650 1000 650];
plot(los_y, los_z, 'g--','LineWidth',1.5);
pts  = {P0,P5,P5inf,P5dir,P5sup};
nms  = {'P0','P5','P5_{inf}','P5_{dir}','P5_{sup}'};
mkrs = {'ko','rs','rs','rs','rs'};
for i = 1:5
    p = pts{i};
    plot(p(2)*1e3,p(3)*1e3,mkrs{i},'MarkerSize',9,'LineWidth',2);
    text(p(2)*1e3+20,p(3)*1e3+15,nms{i},'FontSize',10,'FontWeight','bold');
end
xlabel('Y (mm)','FontSize',12); ylabel('Z (mm)','FontSize',12);
title('Losango — Caminho do Efetuador no Plano YZ','FontSize',13);
legend({'Trajetoria completa','No plano x=1000mm','Losango ideal'},'Location','best');
xlim([-750 750]); ylim([100 1150]);
saveas(gcf,'results/losango_YZ.png');

fprintf('\n=== Concluido ===\n');
