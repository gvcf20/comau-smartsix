% =========================================================
%  bandeira_completa.m
%  ELE041/EEE935 — Manipuladores Robóticos — UFMG
%
%  Execução sequencial da Bandeira do Brasil adaptada:
%  1. Regulação: q0 -> P0 -> P1 (Não desenha)
%  2. Seguimento: Retângulo (P1 -> P2 -> P3 -> P4 -> P1) -> DESENHA VERDE
%  3. Regulação: P1 -> P0 -> P5 (Não desenha)
%  4. Seguimento: Losango (P5 -> P5inf -> P5dir -> P5sup -> P5) -> DESENHA AMARELO
%  5. Regulação: P5 -> P0 -> P6 (Não desenha)
%  6. Seguimento: Círculo completo (a partir de P6) -> DESENHA AZUL
%  7. Regulação: P6 -> P0 (Não desenha)
% =========================================================
clear; clc; close all;
addpath('utils');

%% ============================================================
%  1. INICIALIZAÇÃO DO MODELO
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
%  2. PARÂMETROS E DIRETÓRIOS
%% ============================================================
dt    = 0.01;
K     = 2.0;    
lam   = 0.01;   
N_reg = 1000;   
N_seg = 500;    
mm    = 1e-3;

if ~exist('results','dir'), mkdir('results'); end

%% ============================================================
%  3. ORIENTAÇÃO E COORDENADAS DOS PONTOS
%% ============================================================
Rd = [ 0,  0,  1;
       0,  1,  0;
      -1,  0,  0];

P0 = [700;    0;   650] * mm;
P1 = [1000;  600; 1000] * mm;
P2 = [1000;  600;  300] * mm;
P3 = [1000; -600;  300] * mm;
P4 = [1000; -600; 1000] * mm;

P5    = [1000; -600;  650] * mm; 
P5inf = [1000;    0;  300] * mm; 
P5dir = [1000;  600;  650] * mm; 
P5sup = [1000;    0; 1000] * mm; 

C   = [1000; 0; 650] * mm; 
R_m = ((600 * 350) / sqrt(600^2 + 350^2)) * mm; 
P6  = C + [0; 0; -R_m]; 

%% ============================================================
%  4. FUNÇÕES DE SUPORTE (CONTROLE E ERRO)
%% ============================================================
function eo = orient_err(Rd, Re)
    nphi = rotm2axang2(Rd * Re');   
    eo   = (nphi(1:3) * nphi(4))'; 
end

function [q_out, Q, P] = regulacao(CS6, q0, Rd, pd, N, K, dt, lam)
    Q = zeros(N, 6);  P = zeros(N, 3);  q = q0(:);
    for k = 1:N
        T  = CS6.fkine(q'); if isobject(T), T = T.T; end
        p  = T(1:3, 4); R_m = T(1:3, 1:3);
        e  = [pd - p; orient_err(Rd, R_m)];
        J  = CS6.jacob0(q');
        u  = (J' / (J*J' + lam^2*eye(6))) * (K * e);
        q  = q + dt * u;
        Q(k,:) = q'; P(k,:) = p';
    end
    q_out = q';
end

function [q_out, Q, P] = seguimento(CS6, q0, Rd, Pa, Pb, N, K, dt, lam)
    Q    = zeros(N, 6);  P = zeros(N, 3);  q = q0(:);
    v_ff = [(Pb - Pa) / (N * dt); zeros(3,1)];  
    for k = 1:N
        T  = CS6.fkine(q'); if isobject(T), T = T.T; end
        p  = T(1:3, 4); R_m = T(1:3, 1:3);
        s  = (k-1)/(N-1); pd = Pa + s*(Pb - Pa);
        e  = [pd - p; orient_err(Rd, R_m)];
        J  = CS6.jacob0(q');
        u  = (J' / (J*J' + lam^2*eye(6))) * (v_ff + K * e);
        q  = q + dt * u;
        Q(k,:) = q'; P(k,:) = p';
    end
    q_out = q';
end

function [q_out, Q, P] = seguimento_circular(CS6, q0, Rd, C, R, N, K, dt, lam)
    Q = zeros(N, 6);  P = zeros(N, 3);  q = q0(:);
    omega = (2*pi) / (N * dt); 
    for k = 1:N
        T  = CS6.fkine(q'); if isobject(T), T = T.T; end
        p  = T(1:3, 4); R_m = T(1:3, 1:3);
        s  = (k-1)/(N-1); ang = 2 * pi * s;
        pd = C + [0; R*sin(ang); -R*cos(ang)];
        v_ff = [0; R*omega*cos(ang); R*omega*sin(ang); 0; 0; 0];
        e  = [pd - p; orient_err(Rd, R_m)];
        J  = CS6.jacob0(q');
        u  = (J' / (J*J' + lam^2*eye(6))) * (v_ff + K * e);
        q  = q + dt * u;
        Q(k,:) = q'; P(k,:) = p';
    end
    q_out = q';
end

%% ============================================================
%  5. CÁLCULO E CONCATENAÇÃO DAS TRAJETÓRIAS
%% ============================================================
q           = deg2rad([0, 0, -90, 0, -90, 0]); 
Q_all       = q;
P_all       = [];
Desenha_all = false; % Vetor lógico para controle do rastro dinâmico 3D

% Inicialização das matrizes isoladas de desenho
P_retangulo = [];
P_losango   = [];
P_circulo   = [];

disp('Calculando malhas cinematica da bandeira...');

% FASE 1: O Retângulo
[q, Qh, Ph] = regulacao(CS6, q, Rd, P0, N_reg, K, dt, lam); Q_all = [Q_all; Qh]; P_all = [P_all; Ph]; Desenha_all = [Desenha_all; false(N_reg, 1)];
[q, Qh, Ph] = regulacao(CS6, q, Rd, P1, N_reg, K, dt, lam); Q_all = [Q_all; Qh]; P_all = [P_all; Ph]; Desenha_all = [Desenha_all; false(N_reg, 1)];

wpts_rec = {P1, P2, P3, P4, P1};
for s = 1:4
    [q, Qh, Ph] = seguimento(CS6, q, Rd, wpts_rec{s}, wpts_rec{s+1}, N_seg, K, dt, lam);
    Q_all = [Q_all; Qh]; P_all = [P_all; Ph]; Desenha_all = [Desenha_all; true(N_seg, 1)];
    P_retangulo = [P_retangulo; Ph];
end
[q, Qh, Ph] = regulacao(CS6, q, Rd, P0, N_reg, K, dt, lam); Q_all = [Q_all; Qh]; P_all = [P_all; Ph]; Desenha_all = [Desenha_all; false(N_reg, 1)];

% FASE 2: O Losango
[q, Qh, Ph] = regulacao(CS6, q, Rd, P5, N_reg, K, dt, lam); Q_all = [Q_all; Qh]; P_all = [P_all; Ph]; Desenha_all = [Desenha_all; false(N_reg, 1)];
wpts_los = {P5, P5inf, P5dir, P5sup, P5};
for s = 1:4
    [q, Qh, Ph] = seguimento(CS6, q, Rd, wpts_los{s}, wpts_los{s+1}, N_seg, K, dt, lam);
    Q_all = [Q_all; Qh]; P_all = [P_all; Ph]; Desenha_all = [Desenha_all; true(N_seg, 1)];
    P_losango = [P_losango; Ph];
end
[q, Qh, Ph] = regulacao(CS6, q, Rd, P0, N_reg, K, dt, lam); Q_all = [Q_all; Qh]; P_all = [P_all; Ph]; Desenha_all = [Desenha_all; false(N_reg, 1)];

% FASE 3: O Círculo
[q, Qh, Ph] = regulacao(CS6, q, Rd, P6, N_reg, K, dt, lam); Q_all = [Q_all; Qh]; P_all = [P_all; Ph]; Desenha_all = [Desenha_all; false(N_reg, 1)];
[q, Qh, Ph] = seguimento_circular(CS6, q, Rd, C, R_m, N_seg*2, K, dt, lam); Q_all = [Q_all; Qh]; P_all = [P_all; Ph]; Desenha_all = [Desenha_all; true(N_seg*2, 1)];
P_circulo = [P_circulo; Ph];

[q, Qh, Ph] = regulacao(CS6, q, Rd, P0, N_reg, K, dt, lam); Q_all = [Q_all; Qh]; P_all = [P_all; Ph]; Desenha_all = [Desenha_all; false(N_reg, 1)];

%% ============================================================
%  6. SALVAMENTO DOS DADOS PARA O COPPELIASIM
%% ============================================================
q_seq = rad2deg(Q_all)';
save('results/q_seq_bandeira.mat', 'q_seq');

%% ============================================================
%  7. ANIMAÇÃO DO ROBÔ COM TRAIL SELETIVO
%% ============================================================
figure('Name','Animação 3D — COMAU SmartSix','Color','white','Position',[100 100 800 600]);
% Inicializa o ambiente gráfico com a pose de partida
CS6.plot(Q_all(1,:), 'workspace', [-1.5 1.5 -1.5 1.5 -1.2 2.0], 'jointdiam', 1, 'noname');
hold on;

% Plotagem geométrica dos eixos da base
L_ax = 0.3;
quiver3(0,0,0,L_ax,0,0,'r','LineWidth',2,'AutoScale','off','MaxHeadSize',0.5);
quiver3(0,0,0,0,L_ax,0,'g','LineWidth',2,'AutoScale','off','MaxHeadSize',0.5);
quiver3(0,0,0,0,0,L_ax,'b','LineWidth',2,'AutoScale','off','MaxHeadSize',0.5);

% Loop de animação contínua (subamostrado por passo_f para maior fluidez)
passo_f = 10;
for k = 1:passo_f:size(Q_all,1)
    CS6.animate(Q_all(k,:));
    
    % Desenha o rastro em tempo real APENAS se o flag for verdadeiro
    if Desenha_all(k)
        plot3(P_all(k,1), P_all(k,2), P_all(k,3), 'b.', 'MarkerSize', 5);
    end
    drawnow;
end
CS6.animate(Q_all(end,:)); drawnow; % Garante a plotagem da pose final de retorno

%% ============================================================
%  8. FIGURA FINAL ISOLADA NO PLANO YZ (SEM POLUIÇÃO DE TRANSIÇÃO)
%% ============================================================
figure('Name','Resultado Final — Plano YZ','Color','white','Position',[150 150 750 600]);
hold on; grid on; axis equal; box on;

% Plota as formas separadamente utilizando as cores da bandeira nacional
plot(P_retangulo(:,2)*1e3, P_retangulo(:,3)*1e3, 'Color', [0.0 0.5 0.0], 'LineWidth', 2.5); % Verde
plot(P_losango(:,2)*1e3,   P_losango(:,3)*1e3,   'Color', [0.9 0.7 0.0], 'LineWidth', 2.5); % Amarelo
plot(P_circulo(:,2)*1e3,   P_circulo(:,3)*1e3,   'b-', 'LineWidth', 2.5);                   % Azul

% Marcação pontual de descanso P0
plot(P0(2)*1e3, P0(3)*1e3, 'ko', 'MarkerSize', 7, 'MarkerFaceColor', 'k');
text(P0(2)*1e3+20, P0(3)*1e3+15, 'P0', 'FontSize', 10, 'FontWeight', 'bold');

xlabel('Y (mm)', 'FontSize', 12); ylabel('Z (mm)', 'FontSize', 12);
title('Trajetória Resultante do Efetuador no Plano YZ', 'FontSize', 13);
legend({'Retângulo', 'Losango', 'Círculo', 'Home (P0)'}, 'Location', 'best');
xlim([-750 750]); ylim([100 1150]);

saveas(gcf, 'results/bandeira_completa_YZ.png');
disp('Simulação concluída. Arquivos salvos na pasta /results.');