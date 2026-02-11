%% P1 Identification (No Delay) – V/V (Station Output Voltage / Station Input Voltage)
clear; clc; close all;

% -------- Files ----------
files = ["data1_a.csv","data3_a.csv","data2_a.csv"];

% -------- Station constants ----------
U_OFFSET_V = 0.17;     % station input voltage u = setpoint_v - 0.17
Y_OFFSET_V = 0.15;     % output reconstruction offset: Vstation_out ≈ sensor_v + 0.15 (ADC-side)
DIV_GAIN   = 3/2;      % divider correction (set to 1 if no divider)

STEP_THRESH = 0.01;
N_PREPEND_ZEROS = 10;

% If a dataset does NOT include the pre-step segment (step not visible),
% assume the commanded voltage before logging was 0 V:
PRESTEP_CMD_V_ASSUMED = 0.0;

% -------- System ID options ----------
opt = procestOptions;
opt.Focus = "simulation";
opt.InitialCondition = "estimate";
opt.Display = "off";

G = cell(numel(files),1);

fprintf("\n===== P1 IDENTIFICATION RESULTS (V/V) =====\n");

for k = 1:numel(files)

    %% ---- Load data ----
    tbl = readtable(fullfile(files(k)));

    % Time (s)
    t = tbl.timestamp_ms / 1000;
    t = t - t(1);
    Ts = median(diff(t));

    % Input (station input voltage)
    u_cmd = tbl.setpoint_v;        % commanded DAC voltage
    u     = u_cmd - U_OFFSET_V;    % station input voltage (V)

    % Output (station output voltage)  -> V/V
    v_adc     = tbl.sensor_v;                          % ADC/monitor voltage
    y         = (v_adc + Y_OFFSET_V) * DIV_GAIN;       % station-side output voltage (V)

    %% ---- Step detection (detect JUMP in commanded voltage) ----
    stepIdx = find(abs([0; diff(u_cmd)]) > STEP_THRESH, 1, "first");
    if isempty(stepIdx), stepIdx = 1; end

    % Post-step signals
    u_post = u(stepIdx:end);
    y_post = y(stepIdx:end);

    %% ---- Operating point (baseline) ----
    if stepIdx > 1
        u0 = mean(u(1:stepIdx-1));
        y0 = mean(y(1:stepIdx-1));
    else
        % No visible step inside file -> assume command was 0V before logging
        u0 = PRESTEP_CMD_V_ASSUMED - U_OFFSET_V;

        % Estimate y0 from first ~2 seconds
        N0 = min(numel(y), max(5, round(2/Ts)));
        y0 = mean(y(1:N0));
    end

    %% ---- Deviation signals ----
    du = u_post - u0;
    dy = y_post - y0;

    % Prepend zeros (important for procest)
    du_id = [zeros(N_PREPEND_ZEROS,1); du];
    dy_id = [zeros(N_PREPEND_ZEROS,1); dy];

    z = iddata(dy_id, du_id, Ts);
    z.TimeUnit = "s";

    %% ---- P1 identification ----
    sysP1 = procest(z, "P1", opt);

    % Convert to tf: ΔVout(s) / ΔVin(s)   (V/V)
    K  = sysP1.Kp;
    Tp = sysP1.Tp1;
    G{k} = tf(K, [Tp 1]);

    %% ---- Fit ----
    [~, fit] = compare(z, sysP1);

    fprintf("\n%s\n", files(k));
    fprintf("G%d(s) = %.6g / (%.6g s + 1)   [V/V]\n", k, K, Tp);
    fprintf("Model fit: %.2f %%\n", fit);
end

% Final transfer functions
G1 = G{1};
G2 = G{2};
G3 = G{3};

disp(" "); disp("Final Identified Models (V/V):");
disp(G1); disp(G2); disp(G3);
