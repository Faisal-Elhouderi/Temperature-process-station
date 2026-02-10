%% P1 (no delay) identification for 3 datasets - V/V (UPDATED with your offsets)
% Output: Vout (station-side volts) / Input: u (station input volts)

clear; clc; close all;

% -------- Files ----------
dataDir = "./data";  % change to "." if CSVs are in same folder as this script
files = ["data1.csv","data2.csv","data3.csv"];

% -------- Offsets / scaling (UPDATED) ----------
U_OFFSET_V = 0.17;     % commanded -> station input: u = setpoint_v - 0.17
Y_OFFSET_V = 0.15;     % station output reconstruction offset: Vstation ≈ sensor_v + 0.15 (before divider correction)
DIV_GAIN   = 3/2;      % voltage divider correction (set to 1 if no divider)

% If a dataset doesn't contain a visible step (e.g., logging started after step),
% assume pre-step commanded voltage was 0V:
PRESTEP_CMD_V_ASSUMED = 0.0;

% -------- Step detection / preprocessing ----------
STEP_THRESH = 0.01;          % threshold to detect a jump in commanded setpoint_v
INIT_SEC_IF_NO_PRESTEP = 2;  % estimate y0 from first seconds if no pre-step segment
N_PREPEND_ZEROS = 10;        % helps procest initial transient warning

% -------- System ID options ----------
opt = procestOptions;
opt.Focus = "simulation";
opt.InitialCondition = "estimate";
opt.Display = "off";

G = cell(numel(files),1);
Fit = zeros(numel(files),1);
du_step = zeros(numel(files),1);

fprintf("\n=========== FINAL P1 TRANSFER FUNCTIONS (V/V, NO DELAY) ===========\n");

for k = 1:numel(files)
    fname = fullfile(dataDir, files(k));
    tbl = readtable(fname);

    % Time (s)
    t = tbl.timestamp_ms/1000;
    t = t - t(1);
    Ts = median(diff(t));

    % Input (station input voltage)
    u_cmd = tbl.setpoint_v;      % commanded voltage (what ESP32 outputs)
    u = u_cmd - U_OFFSET_V;      % station input voltage (after input offset)

    % Output (station output voltage)  -> V/V output
    v_adc = tbl.sensor_v;                         % ADC/monitor reading (after divider path)
    y = (v_adc + Y_OFFSET_V) * DIV_GAIN;          % reconstructed station output voltage (V)

    % --- Detect step as a JUMP in commanded input ---
    stepIdx = find(abs([0; diff(u_cmd)]) > STEP_THRESH, 1, "first");
    if isempty(stepIdx), stepIdx = 1; end

    % Post-step data, aligned so "step time" is t=0
    tpost = t(stepIdx:end) - t(stepIdx);
    upost = u(stepIdx:end);
    ypost = y(stepIdx:end);

    % Operating point (baseline)
    if stepIdx > 1
        u0 = mean(u(1:stepIdx-1));
        y0 = mean(y(1:stepIdx-1));
    else
        % No visible step inside file -> assume command was 0V before logging
        u0 = (PRESTEP_CMD_V_ASSUMED - U_OFFSET_V);

        % Estimate y0 from first seconds (best available when no pre-step)
        N0 = min(numel(y), max(5, round(INIT_SEC_IF_NO_PRESTEP/Ts)));
        y0 = mean(y(1:N0));
    end

    % Step magnitude from last 10% of post-step samples
    Nss = max(10, round(0.1*numel(upost)));
    u_ss = mean(upost(end-Nss+1:end));
    du_step(k) = u_ss - u0;

    % Deviation signals for ID
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
    fprintf("G%d(s) = %.6g / (%.6g*s + 1)    | Fit = %.2f %% | Δu = %.3f V\n", ...
        k, K, Tconst, Fit(k), du_step(k));

    % Create uniform time vector for plotting step response
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
    ylabel("Output Voltage (station, V)");
    title(sprintf("%s | P1 (No Delay) | \\Delta u = %.3f V | Fit %.2f%%", files(k), du_step(k), Fit(k)));
    legend("Measured Vout (resampled)", sprintf("Step of G%d(s)", k), "Location", "best");
end

% Final tf objects
G1 = G{1}; G2 = G{2}; G3 = G{3};

fprintf("\n=========== FINAL tf OBJECTS (V/V) ===========\n");
disp("G1 = "); disp(G1);
disp("G2 = "); disp(G2);
disp("G3 = "); disp(G3);
