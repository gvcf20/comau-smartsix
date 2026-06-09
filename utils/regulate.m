function [q_out, qh, ph] = regulate(CS6, q_in, Td, T_reg, Kp, Ko, dt)
% REGULATE  Controle de regulacao: move o efetuador para a pose Td.
%
%   Entradas:
%     CS6    - modelo SerialLink
%     q_in   - configuracao atual (1x6, rad)
%     Td     - pose desejada (matriz 4x4)
%     T_reg  - duracao do movimento (s)
%     Kp, Ko - ganhos (escalares)
%     dt     - passo de integracao (s)
%
%   Saidas:
%     q_out  - configuracao final (1x6, rad)
%     qh     - historico de juntas (N x 6)
%     ph     - historico de posicao (N x 3)

[qh, ph] = control_move(CS6, q_in, Td, [0, T_reg], Kp, Ko, dt, 'regulation');
q_out = qh(end, :);
end
