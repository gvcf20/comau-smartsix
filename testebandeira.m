% =========================================================
%  bandeira.m
%  
%  ELE041/EEE935 — Manipuladores Robóticos — UFMG
%
%  Controle usado:
%  - Regulação:  u = pinv(J) * K * e
%  - Seguimento: u = pinv(J) * (xdot_ref + K * e)
%  - Erro de orientação: representação eixo-ângulo
% =========================================================

clear; clc; close all;
addpath('utils');

%% ============================================================
%  1. MODELO DO ROBÔ
%% ============================================================

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

%% ============================================================
%  2. PARÂMETROS
%% ============================================================

dt      = 0.015;   % passo de integração
K       = 2.0;     % ganho proporcional
N_reg   = 1000;     % máximo de iterações para regulação
N_seg   = 400;     % pontos para cada segmento de desenho
epsilon = 1e-3;    % critério de parada da regulação (norma do erro em m/rad)
mm      = 1e-3;    % conversão mm -> m

if ~exist('results','dir'), mkdir('results'); end
if ~exist('figs','dir'), mkdir('figs'); end

%% ============================================================
%  3. ORIENTAÇÃO DESEJADA E PONTOS DA BANDEIRA
%% ============================================================

% Orientação desejada constante do efetuador
% Ze do efetuador na direção de Xb
% Xe do efetuador na direção contrária de Zb
Rd = [ 0,  0,  1;
       0,  1,  0;
      -1,  0,  0];

P0 = [700;    0;   650] * mm;

% Retângulo
P1 = [1000;  600; 1000] * mm;
P2 = [1000;  600;  300] * mm;
P3 = [1000; -600;  300] * mm;
P4 = [1000; -600; 1000] * mm;

% Losango
P5    = [1000; -600;  650] * mm;
P5inf = [1000;    0;  300] * mm;
P5dir = [1000;  600;  650] * mm;
P5sup = [1000;    0; 1000] * mm;

% Círculo
centro_circulo = [1000; 0; 650] * mm;
raio_circulo   = ((600 * 350) / sqrt(600^2 + 350^2)) * mm;
P6 = centro_circulo + [0; 0; -raio_circulo];

%% ============================================================
%  4. INICIALIZAÇÃO DAS TRAJETÓRIAS
%% ============================================================

q_atual = deg2rad([0, 0, -90, 0, -90, 0]);

T0 = getT(robot.fkine(q_atual));
pos_inicial = T0(1:3,4)';

% Estrutura principal com todos os dados acumulados
traj.q        = q_atual;          % posições das juntas
traj.pos      = pos_inicial;      % posição real do efetuador
traj.u        = zeros(1,6);       % ação de controle
traj.pos_ref  = pos_inicial;      % posição desejada
traj.erro_rpy = zeros(1,3);       % erro de orientação RPY
traj.desenha  = false;            % true quando está desenhando

% Trajetórias isoladas para plotar a bandeira final no plano YZ
pos_retangulo = [];
pos_losango   = [];
pos_circulo   = [];

disp('Calculando trajetória da bandeira...');

%% ============================================================
%  5. DESENHOS
%% ============================================================

%% RETANGULO

% Vai até P0 sem desenhar
[q_atual, traj] = addRegulacao(robot, q_atual, traj, Rd, P0, N_reg, K, dt, epsilon);

% P0 a P1 sem desenhar
[q_atual, traj] = addRegulacao(robot, q_atual, traj, Rd, P1, N_reg, K, dt, epsilon);

% Desenha o retângulo
segmentos_ret = {P1,P2; P2,P3; P3,P4; P4,P1};

for i = 1:size(segmentos_ret,1)
    P_inicio = segmentos_ret{i,1};
    P_fim    = segmentos_ret{i,2};

    [q_atual, traj, pos_trecho] = addReta( ...
        robot, q_atual, traj, Rd, P_inicio, P_fim, N_seg, K, dt);

    pos_retangulo = [pos_retangulo; pos_trecho];
end

% retorna para P0 sem desenhar
[q_atual, traj] = addRegulacao(robot, q_atual, traj, Rd, P0, N_reg, K, dt, epsilon);


%% LOSANGO

% Vai até P5 sem desenhar
[q_atual, traj] = addRegulacao(robot, q_atual, traj, Rd, P5, N_reg, K, dt, epsilon);

% Desenha o losango
segmentos_los = {P5,P5inf; P5inf,P5dir; P5dir,P5sup; P5sup,P5};

for i = 1:size(segmentos_los,1)
    P_inicio = segmentos_los{i,1};
    P_fim    = segmentos_los{i,2};

    [q_atual, traj, pos_trecho] = addReta( ...
        robot, q_atual, traj, Rd, P_inicio, P_fim, N_seg, K, dt);

    pos_losango = [pos_losango; pos_trecho];
end

% retorna para P0 sem desenhar
[q_atual, traj] = addRegulacao(robot, q_atual, traj, Rd, P0, N_reg, K, dt, epsilon);


%%  CIRCULO

% Vai até P6 sem desenhar
[q_atual, traj] = addRegulacao(robot, q_atual, traj, Rd, P6, N_reg, K, dt, epsilon);

% Desenha o círculo
[q_atual, traj, pos_circulo] = addCirculo( ...
    robot, q_atual, traj, Rd, centro_circulo, raio_circulo, N_seg*2, K, dt);

% Retorna para P0 sem desenhar
[q_atual, traj] = addRegulacao(robot, q_atual, traj, Rd, P0, N_reg, K, dt, epsilon);

disp('Trajetória calculada.');

%% ============================================================
%  6. SALVAR q_seq PARA COPPELIASIM
%% ============================================================

% Reduz a quantidade de pontos para deixar a simulação mais leve
fator_decimacao = 2;
q_coppelia = traj.q(1:fator_decimacao:end,:);

% Garante que o último ponto seja incluído
if ~isequal(q_coppelia(end,:), traj.q(end,:))
    q_coppelia = [q_coppelia; traj.q(end,:)];
end

% CoppeliaSim espera matriz 6 x N em graus
q_seq = rad2deg(q_coppelia)';

save('results/q_seq_bandeira.mat','q_seq');
disp('Arquivo q_seq_bandeira.mat salvo.');

%% ============================================================
%  7. ANIMAÇÃO E FIGURAS
%% ============================================================

animacaoRapida(robot, traj.q, traj.pos, traj.desenha);

figuraYZ(pos_retangulo, pos_losango, pos_circulo, P0);

pontos = struct('P0',P0, ...
                'P1',P1, 'P2',P2, 'P3',P3, 'P4',P4, ...
                'P5',P5, 'P5inf',P5inf, ...
                'P5dir',P5dir, 'P5sup',P5sup, ...
                'P6',P6);

figurasAnalise(traj, pontos, dt);

disp('Simulação concluída. Arquivos salvos em /results e /figs.');

%% ============================================================
%  8. FUNÇÕES LOCAIS
%% ============================================================

function [q_final, traj] = addRegulacao(robot, q_inicial, traj, Rd, p_desejado, N, K, dt, epsilon)
% Calcula um trecho de regulação e adiciona na trajetória total.
% Regulação = ir até um ponto fixo sem desenhar.
% Para quando norm(e) < epsilon OU após N iterações (o que vier primeiro).

    [q_final, q_trecho, pos_trecho, u_trecho, pos_ref_trecho, erro_rpy_trecho] = ...
        controleRegulacao(robot, q_inicial, Rd, p_desejado, N, K, dt, epsilon);

    traj = concatenarTrecho(traj, q_trecho, pos_trecho, u_trecho, ...
                            pos_ref_trecho, erro_rpy_trecho, false);
end

function [q_final, traj, pos_trecho] = addReta(robot, q_inicial, traj, Rd, P_inicio, P_fim, N, K, dt)
% Calcula um trecho de reta e adiciona na trajetória total.
% Reta = trecho desenhado da bandeira.

    [q_final, q_trecho, pos_trecho, u_trecho, pos_ref_trecho, erro_rpy_trecho] = ...
        controleSeguimentoReta(robot, q_inicial, Rd, P_inicio, P_fim, N, K, dt);

    traj = concatenarTrecho(traj, q_trecho, pos_trecho, u_trecho, ...
                            pos_ref_trecho, erro_rpy_trecho, true);
end

function [q_final, traj, pos_trecho] = addCirculo(robot, q_inicial, traj, Rd, centro, raio, N, K, dt)
% Calcula o trecho circular e adiciona na trajetória total.

    [q_final, q_trecho, pos_trecho, u_trecho, pos_ref_trecho, erro_rpy_trecho] = ...
        controleSeguimentoCircular(robot, q_inicial, Rd, centro, raio, N, K, dt);

    traj = concatenarTrecho(traj, q_trecho, pos_trecho, u_trecho, ...
                            pos_ref_trecho, erro_rpy_trecho, true);
end

function traj = concatenarTrecho(traj, q_trecho, pos_trecho, u_trecho, pos_ref_trecho, erro_rpy_trecho, desenha)
% Junta o trecho recém-calculado com a trajetória total.

    traj.q        = [traj.q; q_trecho];
    traj.pos      = [traj.pos; pos_trecho];
    traj.u        = [traj.u; u_trecho];
    traj.pos_ref  = [traj.pos_ref; pos_ref_trecho];
    traj.erro_rpy = [traj.erro_rpy; erro_rpy_trecho];

    flag_trecho = repmat(logical(desenha), size(q_trecho,1), 1);
    traj.desenha = [traj.desenha; flag_trecho];
end

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
% erro_rpy é usado apenas para os gráficos.

    R_erro = Rd * R_atual';

    axang = rotm2axang2(R_erro);
    eixo  = axang(1:3)';
    phi   = axang(4);

    erro_ori = eixo * phi;

    erro_rpy = tr2rpy(R_erro);
    erro_rpy = erro_rpy(:)';
end

function [q_final, q_traj, pos_traj, u_traj, pos_ref_traj, erro_rpy_traj] = ...
    controleRegulacao(robot, q_inicial, Rd, p_desejado, N, K, dt, epsilon)
% Controle de regulação:
% leva o efetuador até um ponto fixo p_desejado.
%
% Lei:
% u = pinv(J) * K * e
%
% Para quando norm(e) < epsilon OU após N iterações (o que vier primeiro).
% Equivalente ao critério de parada do control.m de referência.

    q = q_inicial(:);

    % Pré-aloca com tamanho máximo e trunca ao final
    q_traj        = zeros(N,6);
    pos_traj      = zeros(N,3);
    u_traj        = zeros(N,6);
    pos_ref_traj  = repmat(p_desejado', N, 1);
    erro_rpy_traj = zeros(N,3);

    k_final = N;  % será atualizado se convergir antes

    for k = 1:N
        T = getT(robot.fkine(q'));
        p_atual = T(1:3,4);
        R_atual = T(1:3,1:3);

        erro_pos = p_desejado - p_atual;
        [erro_ori, erro_rpy] = erroOrientacao(Rd, R_atual);

        e = [erro_pos; erro_ori];

        % Critério de parada: norma do erro abaixo do limiar
        if norm(e) < epsilon
            k_final = max(k - 1, 1);  % garante ao menos 1 ponto no array
            break;
        end

        J = robot.jacob0(q');
        u = pinv(J) * (K * e);

        q = q + dt * u;

        q_traj(k,:)        = q';
        pos_traj(k,:)      = p_atual';
        u_traj(k,:)        = u';
        erro_rpy_traj(k,:) = erro_rpy;

        % Se chegou à última iteração sem convergir, avisa
        if k == N
            fprintf('[regulacao] aviso: nao convergiu em %d iteracoes. norm(e)=%.4f\n', N, norm(e));
        end
    end

    % Trunca os arrays ao número real de iterações executadas
    q_traj        = q_traj(1:k_final,:);
    pos_traj      = pos_traj(1:k_final,:);
    u_traj        = u_traj(1:k_final,:);
    pos_ref_traj  = pos_ref_traj(1:k_final,:);
    erro_rpy_traj = erro_rpy_traj(1:k_final,:);

    q_final = q';
end

function [q_final, q_traj, pos_traj, u_traj, pos_ref_traj, erro_rpy_traj] = ...
    controleSeguimentoReta(robot, q_inicial, Rd, P_inicio, P_fim, N, K, dt)
% Controle de seguimento de reta:
% faz o efetuador seguir uma linha de P_inicio até P_fim.
%
% Lei:
% u = pinv(J) * (xdot_ref + K * e)

    q = q_inicial(:);

    q_traj        = zeros(N,6);
    pos_traj      = zeros(N,3);
    u_traj        = zeros(N,6);
    pos_ref_traj  = zeros(N,3);
    erro_rpy_traj = zeros(N,3);

    tf = (N-1) * dt;
    vel_ref = (P_fim - P_inicio) / tf;

    for k = 1:N
        s = (k-1) / (N-1);
        p_desejado = P_inicio + s * (P_fim - P_inicio);

        T = getT(robot.fkine(q'));
        p_atual = T(1:3,4);
        R_atual = T(1:3,1:3);

        erro_pos = p_desejado - p_atual;
        [erro_ori, erro_rpy] = erroOrientacao(Rd, R_atual);

        e = [erro_pos; erro_ori];
        xdot_ref = [vel_ref; zeros(3,1)];

        J = robot.jacob0(q');
        u = pinv(J) * (xdot_ref + K * e);

        q = q + dt * u;

        q_traj(k,:)        = q';
        pos_traj(k,:)      = p_atual';
        u_traj(k,:)        = u';
        pos_ref_traj(k,:)  = p_desejado';
        erro_rpy_traj(k,:) = erro_rpy;
    end

    q_final = q';
end

function [q_final, q_traj, pos_traj, u_traj, pos_ref_traj, erro_rpy_traj] = ...
    controleSeguimentoCircular(robot, q_inicial, Rd, centro, raio, N, K, dt)
% Controle de seguimento circular.
% O círculo está no plano YZ, portanto X permanece constante.

    q = q_inicial(:);

    q_traj        = zeros(N,6);
    pos_traj      = zeros(N,3);
    u_traj        = zeros(N,6);
    pos_ref_traj  = zeros(N,3);
    erro_rpy_traj = zeros(N,3);

    tf = (N-1) * dt;
    omega = 2*pi / tf;

    for k = 1:N
        t = (k-1) * dt;
        ang = omega * t;

        p_desejado = centro + [0;
                               raio*sin(ang);
                              -raio*cos(ang)];

        vel_ref = [0;
                   raio*omega*cos(ang);
                   raio*omega*sin(ang)];

        T = getT(robot.fkine(q'));
        p_atual = T(1:3,4);
        R_atual = T(1:3,1:3);

        erro_pos = p_desejado - p_atual;
        [erro_ori, erro_rpy] = erroOrientacao(Rd, R_atual);

        e = [erro_pos; erro_ori];
        xdot_ref = [vel_ref; zeros(3,1)];

        J = robot.jacob0(q');
        u = pinv(J) * (xdot_ref + K * e);

        q = q + dt * u;

        q_traj(k,:)        = q';
        pos_traj(k,:)      = p_atual';
        u_traj(k,:)        = u';
        pos_ref_traj(k,:)  = p_desejado';
        erro_rpy_traj(k,:) = erro_rpy;
    end

    q_final = q';
end

function animacaoRapida(robot, q_traj, pos_traj, flag_desenho)
% Anima o robô e mostra rastro apenas nos trechos de desenho.

    figure('Name','Animação 3D — COMAU SmartSix', ...
           'Color','white', ...
           'Position',[100 100 800 600]);

    robot.plot(q_traj(1,:), ...
               'workspace',[-1.5 1.5 -1.5 1.5 -1.2 2.0], ...
               'jointdiam',1, ...
               'noname');

    ax = gca;
    ax.Color = 'white';
    ax.XColor = 'black';
    ax.YColor = 'black';
    ax.ZColor = 'black';
    ax.GridColor = 'black';
    ax.GridAlpha = 0.3;

    hold on;

    % Eixos da base
    L = 0.3;
    quiver3(0,0,0,L,0,0,'r','LineWidth',2,'AutoScale','off');
    quiver3(0,0,0,0,L,0,'g','LineWidth',2,'AutoScale','off');
    quiver3(0,0,0,0,0,L,'b','LineWidth',2,'AutoScale','off');

    text(L+0.03,0,0,'X_b','Color','r','FontWeight','bold');
    text(0,L+0.03,0,'Y_b','Color','g','FontWeight','bold');
    text(0,0,L+0.03,'Z_b','Color','b','FontWeight','bold');

    % Usa NaN para esconder trechos sem desenho
    trail_x = pos_traj(:,1);
    trail_y = pos_traj(:,2);
    trail_z = pos_traj(:,3);

    trail_x(~flag_desenho) = NaN;
    trail_y(~flag_desenho) = NaN;
    trail_z(~flag_desenho) = NaN;

    hTrail = plot3(ax,NaN,NaN,NaN,'r-','LineWidth',1.8);

    passo_animacao = 20;

    for k = 1:passo_animacao:size(q_traj,1)
        robot.animate(q_traj(k,:));

        set(hTrail,'XData',trail_x(1:k), ...
                   'YData',trail_y(1:k), ...
                   'ZData',trail_z(1:k));

        drawnow limitrate nocallbacks;
    end

    robot.animate(q_traj(end,:));

    set(hTrail,'XData',trail_x, ...
               'YData',trail_y, ...
               'ZData',trail_z);

    drawnow; %limitrate;
    pause(2);
end

function figuraYZ(pos_retangulo, pos_losango, pos_circulo, P0)
% Plota a bandeira final no plano YZ.

    figure('Name','Resultado Final — Plano YZ', ...
           'Color','white', ...
           'Position',[150 150 750 600]);

    hold on; grid on; axis equal; box on;

    plot(pos_retangulo(:,2)*1e3, pos_retangulo(:,3)*1e3, ...
         'Color',[0 0.5 0], 'LineWidth',2.5);

    plot(pos_losango(:,2)*1e3, pos_losango(:,3)*1e3, ...
         'Color',[0.9 0.7 0], 'LineWidth',2.5);

    plot(pos_circulo(:,2)*1e3, pos_circulo(:,3)*1e3, ...
         'b-', 'LineWidth',2.5);

    plot(P0(2)*1e3, P0(3)*1e3, ...
         'ko', 'MarkerSize',7, 'MarkerFaceColor','k');

    text(P0(2)*1e3+20, P0(3)*1e3+15, 'P0', ...
         'FontSize',10, 'FontWeight','bold');

    xlabel('Y (mm)');
    ylabel('Z (mm)');
    title('Trajetória Resultante do Efetuador no Plano YZ');

    legend({'Retângulo','Losango','Círculo','Home (P0)'}, ...
           'Location','best');

    xlim([-750 750]);
    ylim([100 1150]);

    saveas(gcf,fullfile('figs','bandeira_completa_YZ.png'));
end

function figurasAnalise(traj, pontos, dt)
% Gera figuras de análise da trajetória.

    n = min([size(traj.pos,1), size(traj.q,1), size(traj.u,1), ...
             size(traj.pos_ref,1), size(traj.erro_rpy,1), length(traj.desenha)]);

    pos_traj = traj.pos(1:n,:);
    q_traj   = traj.q(1:n,:);
    u_traj   = traj.u(1:n,:);
    pos_ref  = traj.pos_ref(1:n,:);
    erro_rpy = traj.erro_rpy(1:n,:);
    desenha  = traj.desenha(1:n);

    tempo = (0:n-1) * dt;
    erro_pos = sqrt(sum((pos_ref - pos_traj).^2,2));

    % Figura 2 — Caminho 3D
    figure(2); clf; hold on; grid on; axis equal; box on;

    trail_x = pos_traj(:,1);
    trail_y = pos_traj(:,2);
    trail_z = pos_traj(:,3);

    trail_x(~desenha) = NaN;
    trail_y(~desenha) = NaN;
    trail_z(~desenha) = NaN;

    h1 = plot3(pos_traj(~desenha,1), pos_traj(~desenha,2), pos_traj(~desenha,3), ...
               '.', 'Color',[0.65 0.65 0.65], 'MarkerSize',4);

    h2 = plot3(trail_x, trail_y, trail_z, 'r-', 'LineWidth',2);

    h3 = plot3(pontos.P0(1), pontos.P0(2), pontos.P0(3), ...
               'ko', 'MarkerSize',8, 'MarkerFaceColor','k');

    h4 = plot3([pontos.P1(1),pontos.P2(1),pontos.P3(1),pontos.P4(1)], ...
               [pontos.P1(2),pontos.P2(2),pontos.P3(2),pontos.P4(2)], ...
               [pontos.P1(3),pontos.P2(3),pontos.P3(3),pontos.P4(3)], ...
               'go', 'MarkerSize',8, 'MarkerFaceColor','g');

    h5 = plot3([pontos.P5(1),pontos.P5inf(1),pontos.P5dir(1),pontos.P5sup(1)], ...
               [pontos.P5(2),pontos.P5inf(2),pontos.P5dir(2),pontos.P5sup(2)], ...
               [pontos.P5(3),pontos.P5inf(3),pontos.P5dir(3),pontos.P5sup(3)], ...
               'yo', 'MarkerSize',8, 'MarkerFaceColor','y');

    h6 = plot3(pontos.P6(1),pontos.P6(2),pontos.P6(3), ...
               'bo', 'MarkerSize',8, 'MarkerFaceColor','b');

    xlabel('X (m)');
    ylabel('Y (m)');
    zlabel('Z (m)');
    title('Figura 2 — Caminho 3D Percorrido pelo Efetuador');

    legend([h1 h2 h3 h4 h5 h6], ...
           {'Transições sem desenho','Trechos de desenho','P0', ...
            'Pontos do retângulo','Pontos do losango','Ponto inicial do círculo'}, ...
           'Location','best');

    view(3);
    saveas(gcf,fullfile('figs','fig2_caminho_3D_efetuador.png'));

    % Figura 3 — Posição das juntas
    figure(3); clf;

    for i = 1:6
        subplot(3,2,i);
        plot(tempo, rad2deg(q_traj(:,i)), 'LineWidth',1.5);
        xlabel('Tempo (s)');
        ylabel(['q_' num2str(i) ' (°)']);
        title(['Posição da Junta ' num2str(i)]);
        grid on;
    end

    sgtitle('Figura 3 — Posição das Juntas ao Longo do Tempo');
    saveas(gcf,fullfile('figs','fig3_posicao_juntas.png'));

    % Figura 4 — Velocidades das juntas
    figure(4); clf;

    for i = 1:6
        subplot(3,2,i);
        plot(tempo, rad2deg(u_traj(:,i)), 'LineWidth',1.5);
        xlabel('Tempo (s)');
        ylabel(['u_' num2str(i) ' (°/s)']);
        title(['Velocidade da Junta ' num2str(i)]);
        grid on;
    end

    sgtitle('Figura 4 — Ação de Controle / Velocidades das Juntas');
    saveas(gcf,fullfile('figs','fig4_acao_controle_velocidades.png'));

    % Figura 5 — Erro de posição
    figure(5); clf;
    plot(tempo, erro_pos*1e3, 'LineWidth',2);
    xlabel('Tempo (s)');
    ylabel('Erro de posição (mm)');
    title('Figura 5 — Erro de Posição do Efetuador');
    grid on;
    saveas(gcf,fullfile('figs','fig5_erro_posicao.png'));

    % Figura 6 — Erro de orientação
    figure(6); clf;
    nomes = {'Roll','Pitch','Yaw'};

    for i = 1:3
        subplot(3,1,i);
        plot(tempo, rad2deg(erro_rpy(:,i)), 'LineWidth',1.5);
        xlabel('Tempo (s)');
        ylabel([nomes{i} ' (°)']);
        title(['Erro de ' nomes{i}]);
        grid on;
    end

    sgtitle('Figura 6 — Erro de Orientação em Roll, Pitch e Yaw');
    saveas(gcf,fullfile('figs','fig6_erro_orientacao_rpy.png'));
end
