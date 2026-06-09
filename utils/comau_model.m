function CS6 = comau_model()
% COMAU_MODEL  Cria o modelo SerialLink do COMAU Smart SiX
%   Parametros DH baseados em Pena (2013) - convencao padrao.
%
%   Retorna:
%     CS6  - objeto SerialLink do Robotics Toolbox

% --- Parametros DH: theta_offset, d, a, alpha ---
L(1) = Link([0,  -0.450,  0.150,  pi/2,  0], 'standard');
L(2) = Link([0,   0,      0.590,  pi,    0], 'standard');
L(3) = Link([0,   0,      0.130, -pi/2,  0], 'standard');
L(4) = Link([0,  -0.6471, 0,     -pi/2,  0], 'standard');
L(5) = Link([0,   0,      0,      pi/2,  0], 'standard');
L(6) = Link([0,  -0.095,  0,      pi,    0], 'standard');

% Offsets mecanicos
L(2).offset = -pi/2;
L(3).offset =  pi/2;
L(6).offset =  pi;

CS6 = SerialLink(L, 'name', 'COMAU SmartSix');

% Rotacao de base: 180 graus em X (alinha Z0 com fabricante)
CS6.base = trotx(pi);
end
