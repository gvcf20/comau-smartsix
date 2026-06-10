% =========================================================
%  Regulacao: q0 -> P0 = [700; 0; 650] mm
% =========================================================
clear; clc; close all;

%% Modelo
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

%% Orientacao desejada
Rd = [ 0,  0,  1;
       0,  1,  0;
      -1,  0,  0];

%% Parametros
q   = deg2rad([0, 0, -90, 0, -90, 0]);
P0  = [700; 0; 650] * 1e-3;
Td  = [Rd, P0; 0 0 0 1]; %% Pose desejada
dt  = 0.01;
Kp  = 1.0;
Ko  = 1.0;
lam = 0.01;
N   = 1000;

Q_hist = zeros(N, 6);
P_hist = zeros(N, 3);

%% Laco de controle
for k = 1:N
    T  = CS6.fkine(q); if isobject(T), T = T.T; end

    % Erro de posicao
    ep = Td(1:3,4) - T(1:3,4);

    % Erro de orientacao (eixo-angulo)
    Rerr = Rd * T(1:3,1:3)';
    th   = acos(max(-1, min(1, (trace(Rerr)-1)/2)));
    if abs(th) < 1e-10
        eo = [0;0;0];
    else
        eo = (th/(2*sin(th))) * ...
             [Rerr(3,2)-Rerr(2,3); Rerr(1,3)-Rerr(3,1); Rerr(2,1)-Rerr(1,2)];
    end

    v = [Kp*ep; Ko*eo];

    J = CS6.jacob0(q);
    q = q + (J'/(J*J' + lam^2*eye(6)) * v)' * dt;

    Q_hist(k,:) = q;
    P_hist(k,:) = T(1:3,4)';
end

%% Resultado
T_f = CS6.fkine(q); if isobject(T_f), T_f = T_f.T; end
fprintf('Posicao final  (mm): x=%.3f  y=%.3f  z=%.3f\n', T_f(1:3,4)'*1e3);
fprintf('Erro posicao   (mm): %.4f\n', norm(T_f(1:3,4) - P0)*1e3);
fprintf('Erro orientacao:     %.6f\n', norm(T_f(1:3,1:3) - Rd));

%% Animacao
figure('Color','white','Position',[100 100 900 700]);

Q_anim = [deg2rad([0,0,-90,0,-90,0]); Q_hist(1:5:end,:)];

CS6.plot(Q_anim, ...
    'workspace', [-1.5 1.5 -1.5 1.5 -1.2 2.0], ...
    'jointdiam', 1, 'fps', 25, 'noname');

ax = gca;
ax.Color = 'white';
ax.XColor = 'black';
ax.YColor = 'black';
ax.ZColor = 'black';
set(gcf, 'Color', 'white');
hold(ax, 'on');

L_ax = 0.3;
quiver3(ax,0,0,0,L_ax,0,0,'r','LineWidth',2,'AutoScale','off','MaxHeadSize',0.5);
quiver3(ax,0,0,0,0,L_ax,0,'g','LineWidth',2,'AutoScale','off','MaxHeadSize',0.5);
quiver3(ax,0,0,0,0,0,L_ax,'b','LineWidth',2,'AutoScale','off','MaxHeadSize',0.5);
text(ax,L_ax+0.03,0,0,      'X_b','Color','r','FontWeight','bold','FontSize',11);
text(ax,0,L_ax+0.03,0,      'Y_b','Color','g','FontWeight','bold','FontSize',11);
text(ax,0,0,L_ax+0.03,      'Z_b','Color','b','FontWeight','bold','FontSize',11);
title(ax,'Regulacao: q_0 \rightarrow P_0','FontSize',13);