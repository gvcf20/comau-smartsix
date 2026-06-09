% =========================================================
%  reproJuntasCoppelia.m
%  Envia a sequencia de juntas q_seq para o CoppeliaSim.
%
%  PRE-REQUISITOS:
%   1. q_seq deve estar no workspace (rode parte2_controle.m antes)
%      OU carregue com: load('results/q_seq.mat')
%   2. CoppeliaSim 4.3.0 rodando com o arquivo smartsix.ttt aberto
%      e simulacao em play.
%   3. Os arquivos de comunicacao Matlab-CoppeliaSim devem estar
%      no path (pasta descompactada do Moodle).
% =========================================================

%% Carrega q_seq se nao estiver no workspace
if ~exist('q_seq', 'var')
    fprintf('q_seq nao encontrado no workspace. Carregando results/q_seq.mat...\n');
    load('results/q_seq.mat', 'q_seq');
end

fprintf('q_seq: %d juntas x %d amostras\n', size(q_seq, 1), size(q_seq, 2));

%% Conexao com CoppeliaSim via API legada (remoteApi)
% Encerra conexoes anteriores
try
    simxFinish(-1);
catch
end

clientID = simxStart('127.0.0.1', 19997, true, true, 5000, 5);

if clientID < 0
    error(['Nao foi possivel conectar ao CoppeliaSim.\n' ...
           'Verifique se:\n' ...
           '  1. O CoppeliaSim esta aberto\n' ...
           '  2. O arquivo smartsix.ttt foi carregado\n' ...
           '  3. A simulacao esta em PLAY\n']);
end

fprintf('Conectado ao CoppeliaSim (clientID = %d)\n', clientID);

%% Obtem handles das juntas
joint_names = {'CS6_joint1','CS6_joint2','CS6_joint3', ...
               'CS6_joint4','CS6_joint5','CS6_joint6'};
jh = zeros(1, 6);
for j = 1:6
    [~, jh(j)] = simxGetObjectHandle(clientID, joint_names{j}, simx_opmode_blocking);
end
fprintf('Handles das juntas obtidos.\n');

%% Envia posicoes sequencialmente
n_steps = size(q_seq, 2);
fprintf('Enviando %d amostras...\n', n_steps);

for k = 1:n_steps
    for j = 1:6
        simxSetJointTargetPosition(clientID, jh(j), ...
            deg2rad(q_seq(j, k)), simx_opmode_oneshot);
    end
    % Pequena pausa para sincronizacao (~100Hz)
    pause(0.01);

    if mod(k, 500) == 0
        fprintf('  %.1f%% concluido (%d/%d)\n', 100*k/n_steps, k, n_steps);
    end
end

fprintf('Reproducao concluida.\n');
simxFinish(clientID);
