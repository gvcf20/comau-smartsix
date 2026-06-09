function [q_out, qh, ph] = track_segment(CS6, q_in, Pa, Pb, Rd, T_seg, Kp, Ko, dt)
% TRACK_SEGMENT  Seguimento de trajetoria linear entre dois pontos.
%   Interpola linearmente a posicao de Pa ate Pb em T_seg segundos,
%   mantendo a orientacao Rd constante.
%
%   Entradas:
%     CS6    - modelo SerialLink
%     q_in   - configuracao atual (1x6, rad)
%     Pa, Pb - posicoes inicial e final (3x1, metros)
%     Rd     - matriz de rotacao desejada (3x3), constante
%     T_seg  - duracao do segmento (s)
%     Kp, Ko - ganhos (escalares)
%     dt     - passo de integracao (s)
%
%   Saidas:
%     q_out  - configuracao final (1x6, rad)
%     qh     - historico de juntas (N x 6)
%     ph     - historico de posicao (N x 3)

make_Td = @(p) [Rd, p(:); 0, 0, 0, 1];
Td_func = @(t) make_Td(Pa + (t / T_seg) * (Pb - Pa));

[qh, ph] = control_move(CS6, q_in, Td_func, [0, T_seg], Kp, Ko, dt, 'tracking');
q_out = qh(end, :);
end
