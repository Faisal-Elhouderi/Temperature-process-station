%% P1 (no delay) identification for 3 datasets - V/V version
% Creates G1,G2,G3 as tf objects and plots measured output VOLTAGE vs model step response.

%clear; clc; close all;

% -------- Files ----------
dataDir = "./data";
files = ["data1.csv","data2.csv","data3.csv"];

% -------- Station constants ----------
U_OFFSET_V = 0.17;     % station input = setpoint_v - 0.10
Y_OFFSET_V = 0.1;     % station output equiv = sensor_v + 0.17 (sensor_v after divider)
DIV_GAIN   = 3/2;      % keep 3/2 if same divider; otherwise set to 1 if no divider

STEP_THRESH = 0.01;          % step detect threshold on setpoint_v
INIT_SEC_IF_NO_PRESTEP = 2;  % for y0 if stepIdx==1
N_PREPEND_ZEROS = 10;        % for procest transient warning

% -------- System ID options ----------
opt = procestOptions;
opt.Focus = "simulation";
opt.InitialCondition = "estimate";
opt.Display = "off";

G = cell(3,1);
Fit = zeros(3,1);
du_step = zeros(3,1);

fprintf("\n=========== FINAL P1 TRANSFER FUNCTIONS (V/V, NO DELAY) ===========\n");

for k = 1:3
    fname = fullfile(dataDir, files(k));
    tbl = readtable(fname);

    % Time (s)
    t = tbl.timestamp_ms/1000;
    t = t - t(1);
    Ts = median(diff(t));

    % Input (station input voltage)
    u_cmd = tbl.setpoint_v;
    u = u_cmd - U_OFFSET_V;

    % Output (station output voltage)  <-- V/V OUTPUT HERE
    v_afterDiv = tbl.sensor_v;                          % ADC-side voltage
    y = (v_afterDiv + Y_OFFSET_V) * DIV_GAIN;           % station-side voltage (V)

    % Detect step in setpoint_v
    stepIdx = find(abs([0; diff(u_cmd)]) > STEP_THRESH, 1, "first");
    if isempty(stepIdx), stepIdx = 1; end

    % Post-step data, aligned so step is at t=0
    tpost = t(stepIdx:end) - t(stepIdx);
    upost = u(stepIdx:end);
    ypost = y(stepIdx:end);

    % Operating point
    if stepIdx > 1
        u0 = mean(u(1:stepIdx-1));
        y0 = mean(y(1:stepIdx-1));
    else
        u0 = 0; % assumed pre-step input (same as your original code)
        N0 = min(numel(y), max(5, round(INIT_SEC_IF_NO_PRESTEP/Ts)));
        y0 = mean(y(1:N0));
    end

    % Step magnitude from last 10% of post-step samples
    Nss = max(10, round(0.1*numel(upost)));
    u_ss = mean(upost(end-Nss+1:end));
    du_step(k) = u_ss - u0;

    % Deviation signals (for ID)
    du = upost - u0;
    dy = ypost - y0;

    % Prepend equilibrium zeros (helps procest)
    du_id = [zeros(N_PREPEND_ZEROS,1); du];
    dy_id = [zeros(N_PREPEND_ZEROS,1); dy];
    z = iddata(dy_id, du_id, Ts);
    z.TimeUnit = "s";

    % Estimate P1
    sysP1 = procest(z, "P1", opt);
    [~, fitP1] = compare(z, sysP1);
    Fit(k) = fitP1;

    % Build tf object: ΔVout/ΔVin  (V/V)
    K = sysP1.Kp;
    Tconst = sysP1.Tp1;
    G{k} = tf(K, [Tconst 1]);

    fprintf("\n%s:\n", files(k));
    fprintf("G%d(s) = %.6g / (%.6g*s + 1)    | Fit = %.2f %%\n", k, K, Tconst, Fit(k));

    % Create uniform time vector for step()
    N = numel(tpost);
    t_uniform = (0:Ts:Ts*(N-1)).';
    y_uniform = interp1(tpost, ypost, t_uniform, "linear", "extrap");

    % Step response with actual input step magnitude
    [dV, tstep] = step(du_step(k) * G{k}, t_uniform);
    y_model_abs = y0 + dV;

    % Plot
    figure(k);
    plot(t_uniform, y_uniform, "k", "LineWidth", 1.4); hold on;
    plot(tstep, y_model_abs, "--", "LineWidth", 1.4);
    grid on;
    xlabel("Time (s)");
    ylabel("Output Voltage (V)");
    title(sprintf("%s | P1 (No Delay) | \\Deltau = %.3f V | Fit %.2f%%", files(k), du_step(k), Fit(k)));
    legend("Measured Vout (resampled)", sprintf("Step of G%d(s)", k), "Location", "best");
end

% Final tf objects
G1 = G{1}; G2 = G{2}; G3 = G{3};

fprintf("\n=========== FINAL tf OBJECTS (V/V) ===========\n");
disp("G1 = "); disp(G1);
disp("G2 = "); disp(G2);
disp("G3 = "); disp(G3);
