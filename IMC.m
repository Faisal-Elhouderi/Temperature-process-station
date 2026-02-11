%% IMC / Lambda Tuning for Voltage Plant (PI Controller)  [V/V]

%clear; clc; close all;

% ---- Use the V/V model from your estimator (Dataset 2 for example)
Kp_plant = 0.4604;    % <-- THIS MUST BE V/V (example only!)
tau      = 1459.68;    % time constant (s) (same)

G = tf(Kp_plant, [tau 1]);
%G = G2;
% IMC tuning parameter
lambda = 600;         % try 800, 1000, 1200

% IMC-PI formulas (no delay)
Kp = tau / (Kp_plant * lambda);
Ki = 1 / lambda;

C_IMC = pid(Kp, Ki);

% Closed-loop
T_IMC = feedback(C_IMC * G, 1);

% Step response (this is a 1-V reference step in output volts)
figure;
step(T_IMC, 8000)
grid on;
title(['IMC / \lambda-Tuned PI (V/V)  \lambda = ' num2str(lambda)]);
xlabel('Time (s)');
ylabel('Output voltage deviation (V)');

fprintf('\nIMC / Lambda-Tuned PI Controller (V/V plant):\n');
fprintf('Kp = %.6f\n', Kp);
fprintf('Ki = %.6f\n', Ki);
