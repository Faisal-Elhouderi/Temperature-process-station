%% P1 (no delay) identification for 3 datasets - T/V version
% Requirements (as you requested):
%   - Force u0 = 0 for all datasets
%   - Set y0 = first output sample in the CSV (after converting to Temperature)
%   - Convert output using: T = 36*(G_div*(V_sensor + 0.15)) - 24.315
%
% CSV columns expected: timestamp_ms, setpoint_v, sensor_v
% Step may have been applied before logging (so stepIdx may be 1).

clear; clc; close all;

% -------- Files ----------
dataDir = "./data";
files = ["data1_a.csv","data2_a.csv","data3_a.csv"];

% -------- Station constants (from your report) ----------
U_OFFSET_V = 0.10;    % input offset (station input = setpoint_v - 0.10 V)

% Divider gain: convert ADC-side voltage back to station sensor voltage.
% If your divider is 2/3 (5V -> 3.33V), then G_div = 3/2 = 1.5
G_div = 1.5;          % <-- change if your divider ratio is different

% Temperature conversion (as you gave it)
V_SENSOR_OFFSET = 0.15;
TEMP_A = 36;
TEMP_B = -24.315;

% Step detect (still used for alignment, but u0 will be forced to 0)
STEP_THRESH = 0.01;        % threshold on setpoint_v changes
N_PREPEND_ZEROS = 10;      % helps procest transient warning
USE_RESAMPLE = true;       % keep plots smooth

% -------- System ID options ----------
opt = procestOptions;
opt.Focus = "simulation";
opt.InitialCondition = "zero";   % because we explicitly handle y0 and u0
opt.Display = "off";

G = cell(3,1);
Fit = zeros(3,1);
du_step = zeros(3,1);

fprintf("\n=========== FINAL P1 TRANSFER FUNCTIONS (T/V, NO DELAY) ===========\n");

for k = 1:3
    fname = fullfile(dataDir, files(k));
    tbl = readtable(fname);

    % --- Time (s) ---
    t = tbl.timestamp_ms/1000;
    t = t - t(1);
    Ts = median(diff(t));

    % --- Input u(t): station input voltage ---
    % CSV column is setpoint_v; apply input offset
    u_cmd = tbl.setpoint_v;
    u = u_cmd - U_OFFSET_V;

    % --- Output y(t): temperature ---
    % CSV column sensor_v is ADC-side voltage (after divider)
    V_adc = tbl.sensor_v;

    % Convert to temperature exactly as requested:
    % T = 36*(G_div*(V_sensor + 0.15)) - 24.315
    yT = TEMP_A * ( G_div * (V_adc + V_SENSOR_OFFSET) ) + TEMP_B;

    % --- Detect step in the logged command (for alignment only) ---
    stepIdx = find(abs([0; diff(u_cmd)]) > STEP_THRESH, 1, "first");
    if isempty(stepIdx)
        stepIdx = 1; % step may have happened before logging
    end

    % Post-step data aligned so "step time" is t=0 (even if stepIdx=1)
    tpost = t(stepIdx:end) - t(stepIdx);
    upost = u(stepIdx:end);
    ypost = yT(stepIdx:end);

    % ===== Your requested operating point handling =====
    u0 = 0;               % forced
    y0 = yT(1);            % first output sample from the CSV (temperature)

    % Step magnitude (use last 10% of samples as steady state)
    Nss = max(10, round(0.1*numel(upost)));
    u_ss = mean(upost(end-Nss+1:end));
    du_step(k) = u_ss - u0;   % with u0=0, this is just u_ss

    % Deviation signals for identification
    du = upost - u0;          % = upost
    dy = ypost - y0;          % first dy is generally not 0 unless stepIdx==1, but that's OK

    % Optional: force the deviation output to start at 0 exactly at the first post-step sample
    % (This helps when stepIdx==1 and you want the model curve to start at y0 exactly.)
    % dy = dy - dy(1);

    % Prepend equilibrium zeros (helps procest)
    du_id = [zeros(N_PREPEND_ZEROS,1); du];
    dy_id = [zeros(N_PREPEND_ZEROS,1); dy];

    z = iddata(dy_id, du_id, Ts);
    z.TimeUnit = "s";

    % Estimate P1 model
    sysP1 = procest(z, "P1", opt);
    [~, fitP1] = compare(z, sysP1);
    Fit(k) = fitP1;

    % Build tf object: ΔT/ΔV_in
    K = sysP1.Kp;
    Tconst = sysP1.Tp1;
    G{k} = tf(K, [Tconst 1]);

    fprintf("\n%s:\n", files(k));
    fprintf("G%d(s) = %.6g / (%.6g*s + 1)    | Fit = %.2f %%\n", k, K, Tconst, Fit(k));

    % ----- Plot measured vs model (absolute temperature) -----
    if USE_RESAMPLE
        N = numel(tpost);
        t_uniform = (0:Ts:Ts*(N-1)).';
        y_uniform = interp1(tpost, ypost, t_uniform, "linear", "extrap");
    else
        t_uniform = tpost(:);
        y_uniform = ypost(:);
    end

    % Model response to a step of magnitude du_step(k)
    % step(du_step*G, t) returns ΔT(t)
    [dT_model, tstep] = step(du_step(k) * G{k}, t_uniform);

    % Absolute prediction: y_hat = y0 + ΔT_model
    y_model_abs = y0 + dT_model;
%{
    figure(k);
    plot(t_uniform, y_uniform, "k", "LineWidth", 1.4); hold on;
    plot(tstep, y_model_abs, "--", "LineWidth", 1.4);
    grid on;
    xlabel("Time (s)");
    ylabel("Temperature (°C)");
    title(sprintf("%s | P1 (No Delay) | u0=0, y0=first sample | \\Deltau=%.3f V | Fit %.2f%%", ...
                  files(k), du_step(k), Fit(k)));
    legend("Measured Temperature (resampled)", sprintf("Model: y0 + step(\\Deltau*G%d)", k), "Location", "best");
%}
    
end

% Final tf objects
G1 = G{1}; G2 = G{2}; G3 = G{3};

fprintf("\n=========== FINAL tf OBJECTS (T/V) ===========\n");
disp("G1 = "); disp(G1);
disp("G2 = "); disp(G2);
disp("G3 = "); disp(G3);
