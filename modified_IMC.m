%% IMC / Lambda Tuning for Voltage Plant (PI Controller)  [V/V]  -- Faster (smaller lambda)
clear; clc; close all;

% -------------------- Choose your plant --------------------
% Option 1: Use your identified transfer function directly:
% G = G2;

% Option 2: If you only have K and tau (V/V):
Kp_plant = 0.4604;     % MUST be V/V (dcgain of your V/V plant)
tau      = 1459.68;    % seconds
G = tf(Kp_plant, [tau 1]);

% -------------------- Extract K and tau robustly --------------------
K = dcgain(G);                         % V/V gain
p = pole(G);
p = p(abs(p) > 1e-12);
tau_est = -1/real(p(1));
if isfinite(tau_est) && tau_est > 0
    tau = tau_est;
end
fprintf("Plant: K = %.6g (V/V), tau = %.3f s\n", K, tau);

% -------------------- Tuning goals / constraints --------------------
r_step = 1.0;                 % 1-V reference step in OUTPUT
t_end  = max(8000, 8*tau);

u_min = 0.0;                  % station input min (V)  (adjust if needed)
u_max = 3.3;                  % station input max (V)

% -------------------- Lambda sweep (smaller -> faster) --------------------
% Smaller lambda makes the loop faster but can saturate/overshoot.
% Try ~0.02*tau ... 0.30*tau for "faster" behavior.
lambda_grid = logspace(log10(0.02*tau), log10(0.30*tau), 30);

best = struct('J', inf, 'lambda', NaN, 'Kp', NaN, 'Ki', NaN, ...
              'Ti', NaN, 'Td', NaN, 'OS', NaN, 'Ts', NaN, 'Tr', NaN, ...
              'u_peak', NaN, 'ess', NaN);

for lambda = lambda_grid

    % ---- IMC PI for 1st-order, no delay: G(s)=K/(tau s + 1) ----
    % Controller in STANDARD form: C(s)=Kp*(1 + 1/(Ti s))  (Td=0 for PI)
    Kp = tau / (K * lambda);
    Ti = tau;
    Td = 0.0;

    % Convert to parallel gains (MATLAB pid uses parallel form by default)
    Ki = Kp / Ti;              % = 1/(K*lambda)
    C  = pid(Kp, Ki, 0);

    % Closed-loop output transfer
    T = feedback(C*G, 1);

    % Control signal transfer u/r = C*S where S = 1/(1+CG)
    S = feedback(1, C*G);
    U = minreal(C*S);

    % Step responses
    [y, t] = step(r_step*T, t_end);
    [u, ~] = step(r_step*U, t_end);

    info = stepinfo(y, t, y(end), 'SettlingTimeThreshold', 0.02);

    OS = info.Overshoot;            % %
    Ts = info.SettlingTime;         % s
    Tr = info.RiseTime;             % s
    u_peak = max(u);

    % Tracking steady-state error (should be ~0 unless saturation)
    ess = abs(r_step - y(end));

    % Objective: prefer fast settling, low overshoot, avoid saturation, and (slightly) prefer smaller lambda
    J = Ts ...
        + 5e3*max(0, OS-5)/100 ...       % penalize overshoot > 5%
        + 5e3*max(0, u_peak - u_max) ... % penalize actuator saturation
        + 5e4*ess ...                    % penalize nonzero steady-state error
        + 1e-3*lambda;                   % tiny bias toward smaller lambda (faster)

    if J < best.J
        best.J = J;
        best.lambda = lambda;
        best.Kp = Kp;
        best.Ki = Ki;
        best.Ti = Ti;
        best.Td = Td;
        best.OS = OS;
        best.Ts = Ts;
        best.Tr = Tr;
        best.u_peak = u_peak;
        best.ess = ess;
    end
end

% -------------------- Use the best lambda --------------------
lambda = best.lambda;
Kp = best.Kp; Ki = best.Ki;
Ti = best.Ti; Td = best.Td;

C_IMC  = pid(Kp, Ki, 0);
T_IMC  = feedback(C_IMC*G, 1);
S_IMC  = feedback(1, C_IMC*G);
U_IMC  = minreal(C_IMC*S_IMC);

fprintf('\nBest IMC PI (V/V) found (smaller lambda search):\n');
fprintf('lambda = %.3f s\n', lambda);
fprintf('Kp     = %.6f\n', Kp);
fprintf('Ki     = %.6f  (1/s)\n', Ki);
fprintf('Ti     = %.6f s\n', Ti);
fprintf('Td     = %.6f s\n', Td);
fprintf('Overshoot = %.2f %%\n', best.OS);
fprintf('RiseTime  = %.2f s\n', best.Tr);
fprintf('Settling  = %.2f s\n', best.Ts);
fprintf('u_peak    = %.3f V (station input)\n', best.u_peak);
fprintf('ess       = %.6g V\n', best.ess);

% -------------------- Plot output and control action --------------------
figure;
step(r_step*T_IMC, t_end);
grid on;
title(sprintf('IMC PI (V/V) | \\lambda=%.1f s | Kp=%.3g  Ti=%.3g s', lambda, Kp, Ti));
xlabel('Time (s)');
ylabel('Output voltage deviation (V)');

figure;
step(r_step*U_IMC, t_end);
grid on;
title('Control action u(t) for 1-V output reference step');
xlabel('Time (s)');
ylabel('Station input voltage u (V)');
yline(u_max,'--','u_{max}');
yline(u_min,'--','u_{min}');
