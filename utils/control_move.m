function [q_hist, p_hist] = control_move(CS6, q0, Td_func, t_span, Kp, Ko, dt, mode)
% CONTROL_MOVE  Controle cinematico do COMAU SmartSix via Jacobiana Geometrica.
%
%   Modos:
%     'regulation'  - Td_func e uma transformacao homogenea constante (4x4)
%     'tracking'    - Td_func e um function handle @(t) que retorna T(t)
%
%   Entradas:
%     CS6      - modelo SerialLink
%     q0       - configuracao inicial (1x6, rad)
%     Td_func  - pose desejada: matriz 4x4 (regulacao) ou @(t)->T (seguimento)
%     t_span   - [t0, tf] intervalo de tempo
%     Kp       - ganho de posicao (escalar ou 3x3)
%     Ko       - ganho de orientacao (escalar ou 3x3)
%     dt       - passo de integracao (s)
%     mode     - 'regulation' ou 'tracking'
%
%   Saidas:
%     q_hist   - historico de juntas (n_steps x 6)
%     p_hist   - historico de posicao do efetuador (n_steps x 3)

t0 = t_span(1);
tf = t_span(2);
t_vec = t0:dt:tf;
N = length(t_vec);

q = q0(:)';          % linha
q_hist = zeros(N, 6);
p_hist = zeros(N, 3);

if isscalar(Kp), Kp = Kp * eye(3); end
if isscalar(Ko), Ko = Ko * eye(3); end

for k = 1:N
    t = t_vec(k);

    % Pose desejada no instante t
    if strcmp(mode, 'regulation')
        Td = Td_func;       % constante
    else
        Td = Td_func(t);    % seguimento de trajetoria
    end

    % Cinematica direta
    T = CS6.fkine(q);
    if isobject(T), T = T.T; end   % SE3 -> matriz 4x4

    % --- Erro de posicao ---
    pd = Td(1:3, 4);
    p  = T(1:3, 4);
    ep = pd - p;

    % --- Erro de orientacao (eixo-angulo) ---
    Rd = Td(1:3, 1:3);
    Re = T(1:3, 1:3);
    R_err = Rd * Re';       % rotacao do atual para o desejado (frame base)
    axang = rotm2axang2(R_err);
    eo = axang(4) * axang(1:3)';   % theta * n  (vetor de erro)

    % --- Velocidade de tarefa desejada ---
    v_des = [Kp * ep; Ko * eo];    % 6x1

    % --- Jacobiana geometrica ---
    J = CS6.jacob0(q);   % 6x6

    % Pseudo-inversa com amortecimento (evita singularidades)
    lambda = 0.01;
    J_pinv = J' / (J*J' + lambda^2 * eye(6));

    % --- Velocidade de junta ---
    dq = J_pinv * v_des;

    % --- Integracao (Euler) ---
    q = q + dq' * dt;

    q_hist(k, :) = q;
    p_hist(k, :) = T(1:3, 4)';
end
end
