% =========================================================
%  controle.m
%
%  ELE041/EEE935 — Manipuladores Robóticos — UFMG
%
%  Regulação: q0 -> P0 = [700; 0; 650] mm
%
%  - Lei de controle: u = pinv(J) * K * e
%  - Erro de orientação: representação eixo-ângulo
%  - Integração: q(k+1) = q(k) + dt*u
%  - Critério de parada: norm(e) < epsilon ou N iterações
% =========================================================

clear; clc; close all;
addpath('utils');

%% Modelo do robo

[Hbase, CS6p_raw] = CS6bot();

theta_off = CS6p_raw(1:6);
d_vec     = CS6p_raw(7:12);
a_vec     = CS6p_raw(13:18);
alpha_vec = CS6p_raw(19:24);

for i = 1:6
    L(i) = Link('d', d_vec(i), ...
                'a', a_vec(i), ...
                'alpha', alpha_vec(i), ...
                'offset', theta_off(i), ...
                'standard');
end

robot = SerialLink(L, 'name', 'COMAU SmartSix');
robot.base = Hbase;

%% Orientacao desejada e ponto 0
Rd = [ 0,  0,  1;
       0,  1,  0;
      -1,  0,  0];

mm = 1e-3;
P0 = [700; 0; 650] * mm;


%% Parametros

q_inicial = deg2rad([0, 0, -90, 0, -90, 0]);
q         = q_inicial(:);

dt      = 0.015;   % passo de integração 
K       = 2.0;     % ganho proporcional
N       = 1000;    % máximo de iterações de regulação
epsilon = 1e-3;    % critério de parada: norma do erro em m/rad

Q_hist        = zeros(N,6);
P_hist        = zeros(N,3);
U_hist        = zeros(N,6);
E_hist        = zeros(N,1);
Erro_rpy_hist = zeros(N,3);

convergiu = false;
k_final   = N;

%% Laço de controle regulação

for k = 1:N
    T = getT(robot.fkine(q'));

    p_atual = T(1:3,4);
    R_atual = T(1:3,1:3);

    % Erro de posição
    erro_pos = P0 - p_atual;

    % Erro de orientação em eixo-ângulo
    [erro_ori, erro_rpy] = erroOrientacao(Rd, R_atual);

    % Vetor de erro completo: posição + orientação
    e = [erro_pos; erro_ori];

    % Critério de parada
    if norm(e) < epsilon
        convergiu = true;
        k_final = k;

        Q_hist(k,:)        = q';
        P_hist(k,:)        = p_atual';
        U_hist(k,:)        = zeros(1,6);
        E_hist(k)          = norm(e);
        Erro_rpy_hist(k,:) = erro_rpy;
        break;
    end

    % Jacobiana geométrica e lei de controle cinemático
    J = robot.jacob0(q');
    u = pinv(J) * (K * e);

    % Integração de Euler
    q = q + dt * u;

    % Histórico para análise e animação
    Q_hist(k,:)        = q';
    P_hist(k,:)        = p_atual';
    U_hist(k,:)        = u';
    E_hist(k)          = norm(e);
    Erro_rpy_hist(k,:) = erro_rpy;
end

% Trunca os vetores ao número real de iterações executadas
Q_hist        = Q_hist(1:k_final,:);
P_hist        = P_hist(1:k_final,:);
U_hist        = U_hist(1:k_final,:);
E_hist        = E_hist(1:k_final,:);
Erro_rpy_hist = Erro_rpy_hist(1:k_final,:);


%% Resultados

T_f = getT(robot.fkine(q'));
erro_pos_final = norm(T_f(1:3,4) - P0) * 1e3;
erro_ori_final = norm(T_f(1:3,1:3) - Rd);

fprintf('\n===== Resultado da regulação q0 -> P0 =====\n');
fprintf('Iterações executadas: %d de %d\n', k_final, N);

if convergiu
    fprintf('Status: convergiu com norm(e) = %.6f\n', E_hist(end));
else
    fprintf('Status: aviso - não convergiu até o limite de iterações. norm(e) = %.6f\n', E_hist(end));
end

fprintf('Posição final  (mm): x=%.3f  y=%.3f  z=%.3f\n', T_f(1:3,4)'*1e3);
fprintf('Erro posição   (mm): %.4f\n', erro_pos_final);
fprintf('Erro orientação:     %.6f\n', erro_ori_final);


%% Animacao

figure('Name','Regulação q0 -> P0', ...
       'Color','white', ...
       'Position',[100 100 900 700]);

passo_animacao = 5;
Q_anim = [q_inicial; Q_hist(1:passo_animacao:end,:)];

if ~isequal(Q_anim(end,:), Q_hist(end,:))
    Q_anim = [Q_anim; Q_hist(end,:)];
end

robot.plot(Q_anim, ...
           'workspace',[-1.5 1.5 -1.5 1.5 -1.2 2.0], ...
           'jointdiam',1, ...
           'fps',25, ...
           'noname');

ax = gca;
ax.Color = 'white';
ax.XColor = 'black';
ax.YColor = 'black';
ax.ZColor = 'black';
ax.GridColor = 'black';
ax.GridAlpha = 0.3;
hold(ax,'on');

% Eixos da base
L_ax = 0.3;
quiver3(ax,0,0,0,L_ax,0,0,'r','LineWidth',2,'AutoScale','off','MaxHeadSize',0.5);
quiver3(ax,0,0,0,0,L_ax,0,'g','LineWidth',2,'AutoScale','off','MaxHeadSize',0.5);
quiver3(ax,0,0,0,0,0,L_ax,'b','LineWidth',2,'AutoScale','off','MaxHeadSize',0.5);
text(ax,L_ax+0.03,0,0,'X_b','Color','r','FontWeight','bold','FontSize',11);
text(ax,0,L_ax+0.03,0,'Y_b','Color','g','FontWeight','bold','FontSize',11);
text(ax,0,0,L_ax+0.03,'Z_b','Color','b','FontWeight','bold','FontSize',11);

% Trajetória do efetuador e ponto desejado
plot3(ax,P_hist(:,1),P_hist(:,2),P_hist(:,3),'r-','LineWidth',1.8);
plot3(ax,P0(1),P0(2),P0(3),'ko','MarkerSize',8,'MarkerFaceColor','k');
text(ax,P0(1)+0.03,P0(2),P0(3),'P0','FontWeight','bold');

title(ax,'Regulação: q_0 \rightarrow P_0');



%%  Graficos da analise

tempo = (0:length(E_hist)-1) * dt;

figure('Name','Erro de Regulação', 'Color','white');
plot(tempo, E_hist, 'LineWidth', 1.8);
grid on;
xlabel('Tempo (s)');
ylabel('Norma do erro ||e||');
title('Convergência do erro na regulação q_0 \rightarrow P_0');

figure('Name','Ação de Controle', 'Color','white');
for i = 1:6
    subplot(3,2,i);
    plot(tempo, rad2deg(U_hist(:,i)), 'LineWidth', 1.5);
    grid on;
    xlabel('Tempo (s)');
    ylabel(['u_' num2str(i) ' (°/s)']);
    title(['Velocidade da Junta ' num2str(i)]);
end
sgtitle('Ação de controle — velocidades articulares');



%% Funcoes locais

function T = getT(Tin)
% Padroniza a saída da fkine para matriz homogênea 4x4.

    if isobject(Tin)
        T = Tin.T;
    else
        T = Tin;
    end
end

function [erro_ori, erro_rpy] = erroOrientacao(Rd, R_atual)
% Calcula erro de orientação.
% erro_ori é usado no controle, em representação eixo-ângulo.
% erro_rpy é usado apenas para análise gráfica.

    R_erro = Rd * R_atual';

    axang = rotm2axang2(R_erro);
    eixo  = axang(1:3)';
    phi   = axang(4);

    erro_ori = eixo * phi;

    erro_rpy = tr2rpy(R_erro);
    erro_rpy = erro_rpy(:)';
end
