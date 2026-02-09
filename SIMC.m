%% SIMC (Skogestad) PI Tuning

clear; clc; close all;

%% Plant
Kp_plant = 16.57;
tau = 1459.68;
G = tf(Kp_plant, [tau 1]);

%% SIMC tuning parameter
tau_c = tau;   % closed-loop time constant (try tau/2, tau, 2*tau)

%% SIMC PI formulas (no dead time)
Kp = tau / (Kp_plant * tau_c);
Ti = min(tau, 4*tau_c);
Ki = Kp / Ti;

C_SIMC = pid(Kp, Ki);

%% Closed-loop response
T_SIMC = feedback(C_SIMC * G, 1);

figure;
step(T_SIMC, 8000)
grid on
title('SIMC / Skogestad PI Controller')

%% Display gains
fprintf('\nSIMC PI Controller:\n');
fprintf('Kp = %.6f\n', Kp);
fprintf('Ki = %.6f\n', Ki);
