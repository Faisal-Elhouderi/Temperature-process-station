%% LQG (LQI + Kalman) auto-tuning using open-loop data2.csv (training setpoint = 1.5 V)
% Goal: minimize settling time (target <= 30 minutes if feasible)
%
% What this script does:
%   1) Loads ./data/data2.csv (open-loop step test; setpoint_v ~= 1.5 V)
%   2) Converts measured sensor voltage to Temperature using your linear equation:
%        T = 36*(G_div*(V_adc + 0.15)) - 24.315
%   3) Identifies a first-order transfer function (P1, no delay):  ΔT / Δu
%   4) Designs an LQG servo controller (LQI state-feedback + Kalman filter)
%   5) Auto-tunes LQR weights (Q,R) via a grid-search to reduce settling time
%      while respecting actuator saturation u ∈ [1, 5] V.
%   6) Simulates the tuned closed-loop response and prints the achieved settling time.
%
% Notes / assumptions:
%   - This is a linear design around the operating region of data2.csv. Real thermal plants are nonlinear;
%     if you push u near 5 V, the linear model may over-predict temperature.
%   - LQG does NOT inherently enforce saturation; we apply saturation in the simulation.
%   - Requires: System Identification Toolbox (procest) and Control System Toolbox.
%
% You can change the reference target below if you want a different setpoint.

clear; clc; close all;

%% --------------------------- USER SETTINGS ---------------------------
dataDir  = "./data";
dataFile = "data2.csv";

% Actuator (station input) absolute voltage limits
U_MIN = 1.0;     % V  (per your station spec)
U_MAX = 5.0;     % V

% Temperature conversion (your equation)
G_div = 1.5;            % undo divider (if divider is 2/3 => G_div=3/2=1.5)
V_SENSOR_OFFSET = 0.15;
TEMP_GAIN       = 36;
TEMP_BIAS       = -24.315;
V2T = @(Vadc) TEMP_GAIN .* ( G_div .* (Vadc + V_SENSOR_OFFSET) ) + TEMP_BIAS;

% Identification choices
STEP_THRESH = 0.05;     % V threshold to detect the input step in setpoint_v
N_PREPEND_ZEROS = 10;   % helps procest

% Control target (settling time)
Ts_target_sec = 30*60;  % 30 minutes

% Simulation time (seconds)
Tsim = 2*Ts_target_sec; % simulate 60 minutes by default (adjust if you want)

% Settling tolerance (2% band of the step amplitude, with a minimum absolute tolerance)
SETTLE_PCT = 0.02;
SETTLE_MIN_ABS = 0.5;   % °C minimum band

% Discrete-time controller sample time (choose >= your logger Ts)
Ts_ctrl = 1.0;          % seconds

%% --------------------------- LOAD TRAINING DATA ---------------------------
tbl = readtable(fullfile(dataDir, dataFile));

t = tbl.timestamp_ms/1000;
t = t - t(1);

u_raw = tbl.setpoint_v;        % station input command in volts (from your log)
y_vadc = tbl.sensor_v;         % ADC-side sensor voltage (from your log)

% Convert output to Temperature
y_T = V2T(y_vadc);

% Enforce actuator limits on the logged input to match station spec (clip)
u_abs = min(max(u_raw, U_MIN), U_MAX);

% Detect step index in the clipped input
stepIdx = find(abs([0; diff(u_abs)]) > STEP_THRESH, 1, "first");
if isempty(stepIdx), stepIdx = 1; end

% Pre-step operating point (averaged)
if stepIdx > 5
    u0 = mean(u_abs(1:stepIdx-1));
    T0 = mean(y_T(1:stepIdx-1));
else
    u0 = u_abs(1);
    T0 = y_T(1);
end

% Post-step signals aligned to t=0
tpost = t(stepIdx:end) - t(stepIdx);
upost = u_abs(stepIdx:end);
Tpost = y_T(stepIdx:end);

% Steady-state (last 10% of post-step)
Nss = max(20, round(0.1*numel(upost)));
u_ss = mean(upost(end-Nss+1:end));
T_ss = mean(Tpost(end-Nss+1:end));

% Reference we will track in closed-loop (temperature)
T_ref = T_ss;   % "training setpoint" target temperature from data2's steady state

fprintf("Training data: u0=%.3f V, u_ss=%.3f V | T0=%.2f °C, T_ss=%.2f °C\n", u0, u_ss, T0, T_ss);
fprintf("Closed-loop target: T_ref = %.2f °C\n", T_ref);

%% --------------------------- IDENTIFY PLANT TF: ΔT / Δu ---------------------------
% Deviation signals for identification
du = upost - u0;
dT = Tpost - T0;

% Estimate raw sample time from data (and then use iddata with that Ts)
Ts_data = median(diff(tpost));
du_id = [zeros(N_PREPEND_ZEROS,1); du];
dT_id = [zeros(N_PREPEND_ZEROS,1); dT];

z = iddata(dT_id, du_id, Ts_data);
z.TimeUnit = "s";

optID = procestOptions;
optID.Focus = "simulation";
optID.InitialCondition = "zero";
optID.Display = "off";

sysP1 = procest(z, "P1", optID);    % first-order, no delay
[~, fitP1] = compare(z, sysP1);

Kp  = sysP1.Kp;
tau = sysP1.Tp1;

G = tf(Kp, [tau 1]);               % ΔT / Δu  (°C/V)

fprintf("\nIdentified plant (P1): G(s) = %.6g / (%.6g s + 1)\n", Kp, tau);
fprintf("Fit on training data (data2.csv): %.2f %%\n", fitP1);

%% --------------------------- DISCRETIZE PLANT ---------------------------
% Use a minimal state-space realization
Gss = ss(G);
Gss = minreal(Gss, 1e-8);

Gd = c2d(Gss, Ts_ctrl, "zoh");
Ad = Gd.A; Bd = Gd.B; Cd = Gd.C; Dd = Gd.D;

n = size(Ad,1);

%% --------------------------- KALMAN FILTER (DISCRETE) ---------------------------
% Estimate measurement noise from steady-state portion of training data:
Tss_seg = Tpost(end-Nss+1:end);
sigma_y = std(Tss_seg - mean(Tss_seg));
Rv = max(sigma_y^2, 1e-6);             % measurement noise variance (°C^2)

% Process noise (small, scaled). Increase if estimator is too sluggish.
Qw = 1e-5 * eye(n);                    % process noise covariance
Gnoise = eye(n);                       % assume process noise enters each state

L = dlqe(Ad, Gnoise, Cd, Qw, Rv);      % estimator gain

%% --------------------------- LQI (STATE-FEEDBACK + INTEGRATOR) ---------------------------
% Augment with integrator on temperature tracking error:
%   xI[k+1] = xI[k] + Ts_ctrl*(T_ref - T[k])
% where T[k] = Cd*x[k] + Dd*u[k] + T0 (absolute); in deviations, dy = Cd*x + Dd*du
% We'll do the integrator on the deviation error: e = (T_ref - T0) - dT
dT_ref = T_ref - T0;

Aaug = [Ad, zeros(n,1);
        -Ts_ctrl*Cd, 1];
Baug = [Bd;
        -Ts_ctrl*Dd];

% We'll tune Q and R automatically:
% - Qx: penalize temperature deviation
% - QI: penalize integral error strongly (helps eliminate steady-state error fast)
% - R: penalize control effort (prevents excessive saturation)
%
% Grid-search ranges (adjust if needed)
qI_list  = [1e2, 1e4, 1e6];
rho_list = logspace(-3, 3, 25);        % R = rho

best = struct("Ts", inf, "K", [], "qI", NaN, "rho", NaN, "satFrac", NaN);

for qI = qI_list
    % Output-focused Q: weight the output state energy via C' C
    Qx = (Cd')*(Cd);                   % makes "temperature deviation" costly
    Q  = blkdiag(Qx, qI);              % include integrator weight
    for rho = rho_list
        R = rho;

        try
            Kaug = dlqr(Aaug, Baug, Q, R);
        catch
            continue;
        end

        Kx = Kaug(1:n);
        Ki = Kaug(n+1);

        sim = simulate_lqg_lqi(Ad,Bd,Cd,Dd, L, Kx, Ki, Ts_ctrl, dT_ref, T0, u0, U_MIN, U_MAX, Tsim);

        % Prefer solutions that meet the 30 min target, then minimize Ts
        if sim.Ts <= Ts_target_sec
            if sim.Ts < best.Ts
                best.Ts = sim.Ts; best.K = Kaug; best.qI=qI; best.rho=rho; best.satFrac=sim.satFrac;
                best.sim = sim;
            end
        else
            % If nothing meets the target, still keep the best Ts overall
            if isinf(best.Ts) && sim.Ts < best.Ts
                best.Ts = sim.Ts; best.K = Kaug; best.qI=qI; best.rho=rho; best.satFrac=sim.satFrac;
                best.sim = sim;
            end
        end
    end
end

if isempty(best.K)
    error("Failed to find any stabilizing LQI gain. Try changing qI_list / rho_list.");
end

Kaug = best.K;
Kx = Kaug(1:n);
Ki = Kaug(n+1);

fprintf("\n=== SELECTED LQG/LQI TUNING ===\n");
fprintf("qI = %.3g, rho = %.3g\n", best.qI, best.rho);
fprintf("Predicted settling time (2%%): %.1f s (%.1f min)\n", best.sim.Ts, best.sim.Ts/60);
fprintf("Saturation fraction: %.1f %% of the time\n", 100*best.sim.satFrac);
fprintf("Kx = "); disp(Kx);
fprintf("Ki = %.6g\n", Ki);
fprintf("Kalman L = "); disp(L);

%% --------------------------- PLOTS: OPEN-LOOP vs TUNED LQG ---------------------------
% Open-loop training response (temperature)
figure("Color","w","Name","Open-loop training vs tuned LQG (Temperature)");
plot(tpost, Tpost, "k", "LineWidth", 1.3); hold on;

% Tuned simulation (absolute temperature)
plot(best.sim.t, best.sim.Tabs, "r--", "LineWidth", 1.6);

yline(T_ref, "LineWidth", 1.2);
xline(Ts_target_sec, ":", "LineWidth", 1.2);

grid on;
xlabel("Time (s)");
ylabel("Temperature (°C)");
title(sprintf("Training open-loop (data2) vs tuned LQG | Ts=%.1f min (target 30 min)", best.sim.Ts/60));
legend("Open-loop measured T (data2)", "Simulated tuned LQG", "T_{ref}", "30 min target", "Location","best");

figure("Color","w","Name","Tuned LQG control input (absolute volts)");
plot(best.sim.t, best.sim.uabs, "LineWidth", 1.4); grid on;
yline(U_MIN, "--", "LineWidth", 1.0);
yline(U_MAX, "--", "LineWidth", 1.0);
xlabel("Time (s)");
ylabel("Actuator input u (V)");
title("Tuned LQG control input (with saturation)");

%% --------------------------- Helper functions ---------------------------
function sim = simulate_lqg_lqi(Ad,Bd,Cd,Dd, L, Kx, Ki, Ts, dT_ref, T0, u0, Umin, Umax, Tsim)
% Simulate discrete-time plant with LQG (Kalman) + integral action and input saturation.
%
% Plant is in deviations: x[k+1]=Ad x + Bd du,  dT = Cd x + Dd du
% Absolute temperature: T_abs = T0 + dT
% Absolute actuator voltage: u_abs = clamp(u0 + du, Umin, Umax)

n = size(Ad,1);
N = max(1, round(Tsim / Ts));
t = (0:Ts:Ts*(N-1)).';

x = zeros(n,1);          % true plant state (deviation)
xhat = zeros(n,1);       % estimated state
xI = 0;                  % integrator state

du = 0;                  % deviation input
uabs = u0;               % absolute input (V)

Tabs = zeros(N,1);
u_hist = zeros(N,1);
satCount = 0;

for k = 1:N
    % Output (true)
    dT = Cd*x + Dd*du;
    Tabs(k) = T0 + dT;

    % Control law (using estimate + integrator)
    e = dT_ref - dT;              % deviation tracking error
    xI = xI + Ts*e;

    du_cmd = -(Kx*xhat + Ki*xI);

    % Convert to absolute and saturate
    u_cmd_abs = u0 + du_cmd;
    u_cmd_abs = min(max(u_cmd_abs, Umin), Umax);

    if u_cmd_abs ~= (u0 + du_cmd)
        satCount = satCount + 1;
    end

    uabs = u_cmd_abs;
    du = uabs - u0;

    u_hist(k) = uabs;

    % Plant update
    x = Ad*x + Bd*du;

    % Measurement (assume we measure temperature perfectly here; noise can be added if needed)
    y = Cd*x + Dd*du;

    % Kalman update (predict-correct form)
    xhat = Ad*xhat + Bd*du + L*(y - (Cd*xhat + Dd*du));
end

% Settling time (2% of step amplitude, with minimum absolute band)
Tfinal = T0 + dT_ref;
amp = abs(Tfinal - Tabs(1));
tol = max(0.02*amp, 0.5);   % °C

err = abs(Tabs - Tfinal);
idx_last = find(err > tol, 1, "last");
if isempty(idx_last)
    Ts_settle = 0;
elseif idx_last >= N
    Ts_settle = t(end);
else
    Ts_settle = t(idx_last);
end

sim.t = t;
sim.Tabs = Tabs;
sim.uabs = u_hist;
sim.Ts = Ts_settle;
sim.satFrac = satCount / N;
end
