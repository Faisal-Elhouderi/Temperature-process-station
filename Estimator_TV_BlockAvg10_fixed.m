%% P1 (no delay) identification for 3 datasets - T/V version (BLOCK-AVERAGED)
% What this script does (per your request):
%   1) Forces u0 = 0 for ALL datasets
%   2) Sets y0 = first output sample from the CSV file (after converting to Temperature)
%   3) Converts voltage to temperature using:
%        T = 36*(G_div*(V_sensor + 0.15)) - 24.315
%   4) Averages every 10 consecutive samples into ONE sample (block average)
%      and uses these averaged samples for BOTH plotting and identification
%   5) Prints the NEW fit percentage after block-averaging
%
% Expected CSV columns:
%   timestamp_ms, setpoint_v, sensor_v
%
% You may need: System Identification Toolbox (procest, iddata, compare).

clear; clc; close all;

% -------- Files ----------
dataDir = "./data";
files = ["data1.csv","data2.csv","data3.csv"];

% -------- Station constants ----------
U_OFFSET_V = 0.10;      % station input voltage = setpoint_v - 0.10

% Divider gain to undo your divider (ADC-side -> true sensor voltage)
% If divider is 2/3, then G_div = 3/2 = 1.5
G_div = 1.5;            % <-- change if your divider ratio differs

% Temperature conversion (your exact equation)
V_SENSOR_OFFSET = 0.15;
TEMP_GAIN       = 36;
TEMP_BIAS       = -24.315;

% Step detect threshold on setpoint_v
STEP_THRESH = 0.01;

% Block averaging
N_AVG = 10;             % average 10 samples -> 1 sample (your requested change)

% Prepend zeros to help procest (optional)
N_PREPEND_ZEROS = 10;

% -------- System ID options ----------
opt = procestOptions;
opt.Focus = "simulation";
opt.InitialCondition = "zero";   % we handle y0 explicitly
opt.Display = "off";

G = cell(3,1);
Fit = zeros(3,1);
du_step = zeros(3,1);

fprintf("\n=========== P1 TRANSFER FUNCTIONS (T/V, NO DELAY) | Block Avg N=%d ===========\n", N_AVG);

for k = 1:3
    fname = fullfile(dataDir, files(k));
    tbl = readtable(fname);

    % ---- Time (s) ----
    t = tbl.timestamp_ms/1000;
    t = t - t(1);

    % Estimate sample time from raw data (before averaging)
    Ts_raw = median(diff(t));

    % ---- Input u(t) (station input volts) ----
    u_cmd = tbl.setpoint_v;
    u = u_cmd - U_OFFSET_V;

    % ---- Output y(t) = Temperature ----
    V_adc = tbl.sensor_v; % ADC-side voltage (after divider)
    T = TEMP_GAIN * ( G_div * (V_adc + V_SENSOR_OFFSET) ) + TEMP_BIAS;

    % ---- Step detection (alignment only) ----
    stepIdx = find(abs([0; diff(u_cmd)]) > STEP_THRESH, 1, "first");
    if isempty(stepIdx)
        stepIdx = 1; % step may have happened before logging
    end

    % Align so step time is t=0
    tpost = t(stepIdx:end) - t(stepIdx);
    upost = u(stepIdx:end);
    Tpost = T(stepIdx:end);

    % ===== Your required operating points =====
    u0 = 0;      % forced
    y0 = T(1);   % first temperature sample in the file

    % ============================================================
    % BLOCK AVERAGE: collapse every N_AVG samples -> 1 sample
    % ============================================================
    N = numel(tpost);
    M = floor(N / N_AVG);          % number of complete blocks
    if M < 5
        error("Not enough samples after step for block-averaging (dataset %d).", k);
    end

    idx = 1:(M * N_AVG);
    t_use = tpost(idx);
    u_use = upost(idx);
    T_use = Tpost(idx);

    t_mat = reshape(t_use, N_AVG, M);
    u_mat = reshape(u_use, N_AVG, M);
    T_mat = reshape(T_use, N_AVG, M);

    t_avg = mean(t_mat, 1).';      % one timestamp per block
    u_avg = mean(u_mat, 1).';
    T_avg = mean(T_mat, 1).';

    % New effective sample time
    Ts = median(diff(t_avg));

    % ===== Step magnitude after averaging (use last 10% as steady state) =====
    Nss = max(2, round(0.1*numel(u_avg)));
    u_ss = mean(u_avg(end-Nss+1:end));
    du_step(k) = u_ss - u0;        % since u0=0 => du_step = u_ss

    % ===== Deviation signals for identification =====
    du = u_avg - u0;               % = u_avg
    dy = T_avg - y0;

    % Prepend equilibrium zeros (optional)
    du_id = [zeros(N_PREPEND_ZEROS,1); du];
    dy_id = [zeros(N_PREPEND_ZEROS,1); dy];

    z = iddata(dy_id, du_id, Ts);
    z.TimeUnit = "s";

    % ---- Estimate P1 model ----
    sysP1 = procest(z, "P1", opt);

    % Fit percentage on the SAME (averaged) identification dataset:
    [~, fitP1] = compare(z, sysP1);
    Fit(k) = fitP1;

    % Build tf: ΔT / ΔVin
    K   = sysP1.Kp;        % °C/V
    tau = sysP1.Tp1;       % s
    G{k} = tf(K, [tau 1]);

    fprintf("\n%s:\n", files(k));
    fprintf("G%d(s) = %.6g / (%.6g*s + 1)    | NEW Fit (avg %d) = %.2f %%\n", ...
        k, K, tau, N_AVG, Fit(k));

    % ---- Plot averaged measured vs reconstructed model ----
    % NOTE:
    %   MATLAB's step() requires an evenly-spaced time vector.
    %   Your averaged timestamps t_avg can still be slightly non-uniform (logger jitter),
    %   so we simulate on a uniform grid using lsim().

    Nplot = numel(t_avg);
    t_uniform = (0:Ts:Ts*(Nplot-1)).';

    % Interpolate the averaged signals onto the uniform grid
    % (linear is fine; use 'previous' if you prefer ZOH on input)
    u_uniform = interp1(t_avg, u_avg, t_uniform, "linear", "extrap");
    T_uniform = interp1(t_avg, T_avg, t_uniform, "linear", "extrap");

    % Deviation input for simulation (u0 is forced to 0, but keep the form)
    du_uniform = u_uniform - u0;

    % Simulate deviation temperature using the identified transfer function
    dT_model = lsim(G{k}, du_uniform, t_uniform);

    % Absolute prediction: T_hat = y0 + ΔT_model
    T_model_abs = y0 + dT_model;

    figure(k);
    plot(t_uniform, T_uniform, "k.-", "LineWidth", 1.2); hold on;
    plot(t_uniform, T_model_abs, "r--", "LineWidth", 1.4);
    grid on;
    xlabel("Time (s)");
    ylabel("Temperature (°C)");
    title(sprintf("%s | Block-Avg(%d) | u0=0, y0=T(1) | Fit %.2f%%", ...
        files(k), N_AVG, Fit(k)));
    legend("Measured (avg blocks)", sprintf("Model (lsim): y0 + lsim(G%d, u)", k), "Location", "best");
end

% Final tf objects
G1 = G{1}; G2 = G{2}; G3 = G{3};

fprintf("\n=========== FINAL tf OBJECTS (T/V) | Block Avg N=%d ===========\n", N_AVG);
disp("G1 = "); disp(G1);
disp("G2 = "); disp(G2);
disp("G3 = "); disp(G3);

