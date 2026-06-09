function axang = rotm2axang2(R)
% ROTM2AXANG2  Calcula a representacao eixo-angulo de uma matriz de rotacao.
%   Compativel com o arquivo auxiliar do Moodle (ELE041/EEE935 - UFMG).
%
%   Entrada:
%     R     - matriz de rotacao 3x3
%   Saida:
%     axang - vetor [n1, n2, n3, theta] onde [n1,n2,n3] e o eixo unitario
%             e theta e o angulo de rotacao (rad)

% Angulo
theta = acos(max(-1, min(1, (trace(R) - 1) / 2)));

if abs(theta) < 1e-10
    % Sem rotacao
    axang = [0, 0, 1, 0];
    return
end

if abs(theta - pi) < 1e-10
    % Caso singular: theta = pi
    [~, idx] = max([R(1,1), R(2,2), R(3,3)]);
    switch idx
        case 1
            n = [sqrt((R(1,1)+1)/2); R(2,1)/(2*sqrt((R(1,1)+1)/2)); R(3,1)/(2*sqrt((R(1,1)+1)/2))];
        case 2
            n = [R(1,2)/(2*sqrt((R(2,2)+1)/2)); sqrt((R(2,2)+1)/2); R(3,2)/(2*sqrt((R(2,2)+1)/2))];
        case 3
            n = [R(1,3)/(2*sqrt((R(3,3)+1)/2)); R(2,3)/(2*sqrt((R(3,3)+1)/2)); sqrt((R(3,3)+1)/2)];
    end
    axang = [n(:)', theta];
    return
end

% Caso geral
n = (1/(2*sin(theta))) * [R(3,2)-R(2,3); R(1,3)-R(3,1); R(2,1)-R(1,2)];
axang = [n(:)', theta];
end
