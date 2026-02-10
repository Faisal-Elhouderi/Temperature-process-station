%% Constrained ITAE PI Tuning (Physically Meaningful)

clear; clc; close all;

%% Plant (nominal)
Kp_plant = 0.460407;
tau = 1459.68;
G = tf(Kp_plant, [tau 1]);

%% Simulation setup
T_end = 8000;
t = linspace(0, T_end, 4000);
r = ones(size(t));

%% Weight on control effort (CRITICAL)
alpha = 1e-4;   % increase if gains still too large

%% Gain bounds (physical intuition)
Kp_max = 0.5;
Ki_max = 0.01;

%% Initial guess
x0 = [0.05, 0.001];

%% Optimization options
options = optimset('Display','iter','TolX',1e-6,'TolFun',1e-6);

%% Optimization
x_opt = fminsearch(@(x) costFun(x, G, t, r, alpha, Kp_max, Ki_max), x0, options);

%% Extract gains
Kp = abs(x_opt(1));
Ki = abs(x_opt(2));

C_ITAE = pid(Kp, Ki);
T_ITAE = feedback(C_ITAE * G, 1);

%% Step response
figure;
step(T_ITAE, T_end)
grid on
title('Fixed ITAE-Optimized PI Controller')

%% Display results
fprintf('\nFixed ITAE PI Controller:\n');
fprintf('Kp = %.6f\n', Kp);
fprintf('Ki = %.6f\n', Ki);


%% -------- COST FUNCTION --------
function J = costFun(x, G, t, r, alpha, Kp_max, Ki_max)

    % Enforce positivity and bounds
    Kp = min(abs(x(1)), Kp_max);
    Ki = min(abs(x(2)), Ki_max);

    C = pid(Kp, Ki);
    T = feedback(C * G, 1);

    % Simulate
    y = lsim(T, r, t);
    e = r(:) - y(:);

    % Control effort
    u = lsim(C, e, t);

    % ITAE + control penalty
    J = trapz(t, t(:).*abs(e) + alpha*u.^2);
end
