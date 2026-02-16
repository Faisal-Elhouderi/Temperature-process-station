%% LQG_from_Estimator_v3.m
% -------------------------------------------------------------------------
% v3 FIXES (addresses: overshoot, no improvement, AGGRESSIVENESS no effect)
%
% Root causes in v1/v2:
%   (A) The plant identified in Estimator.m is a STEP/DEVIATION model (ΔT/Δu).
%       But v1/v2 treated the controller output as an ABSOLUTE station command
%       starting from 1V, which often forces saturation/rate limits and makes
%       "AGGRESSIVENESS" appear to do nothing.
%   (B) v2 removed proper anti-windup, so the integrator could wind up and
%       create overshoot.
%   (C) The old aggressiveness scaling could cancel out (Q and R scaled similarly).
%
% What v3 does:
%   1) Uses a measured OPERATING POINT (bias) from NOISE_FILE:
%        U_STATION_BIAS = mean(setpoint_v in the first PRE_SEC seconds)
%        T0 = temperature from the first ADC sample
%      Control output is Δu around that bias; plant uses Δu.
%   2) Adds conditional anti-windup (integrate only when it helps).
%   3) Redefines AGGRESSIVENESS so it ALWAYS changes K:
%        R = R0 / AGGRESSIVENESS^2   (larger aggressiveness => stronger control)
%   4) Keeps optional ADC moving average and optional rate limit.
%
% Usage:
%   Put next to Estimator.m and run this file.
%   Results saved to workspace: LQG_results
% -------------------------------------------------------------------------
clear; clc; close all;

%% ----------------------- ENSURE TRANSFER FUNCTIONS EXIST -----------------------
if ~(exist('G1','var')==1 || exist('G2','var')==1 || exist('G3','var')==1)
    if exist('Estimator.m','file')==2
        fprintf("Running Estimator.m to create G1,G2,G3 in the workspace...\n");
        run('Estimator.m');   % may clear workspace
    else
        error("Estimator.m not found on MATLAB path.");
    end
end

plants = struct();
if exist('G1','var')==1, plants.G1 = G1; end
if exist('G2','var')==1, plants.G2 = G2; end
if exist('G3','var')==1, plants.G3 = G3; end
if isempty(fieldnames(plants))
    error("No transfer functions found (G1/G2/G3). Run Estimator.m successfully first.");
end

%% ----------------------- USER SETTINGS -----------------------
RUN_ALL_PLANTS   = false;
PLANT_TO_USE     = "G2";

T_REF_STEP_C     = 20;     % temperature step magnitude (degC)
T_STEP_TIME_S    = 10;     % step time (s)
SIM_TIME_MIN     = 90;     % simulation length (min)

AGGRESSIVENESS   = 1.5;    % >1 stronger/faster; <1 softer/slower (now ALWAYS affects K)

DATA_DIR         = "./data";
NOISE_FILE       = "data2.csv";   % used to estimate Ts, bias (u), T0, noise

PRE_SEC          = 10;     % seconds used to estimate the operating point bias

%% ----------------------- FILTER / LIMIT SETTINGS -----------------------
USE_MEAS_NOISE           = false;   % set false for clean plots
USE_ADC_MOVING_AVG       = true;   % emulate your ESP32 averaging
ADC_AVG_N                = 10;

% Integrator robustness
INTEGRATOR_ON_ESTIMATE   = true;   % recommended
ERROR_DEADBAND_C         = 0.05;   % degC
T_AW_SEC                 = 60;     % anti-windup time constant (seconds) for back-calculation

% Actuator smoothing
USE_RATE_LIMIT           = false;  % start false (so aggressiveness is visible)
RATE_LIMIT_V_PER_S       = 0.30;   % station V/s (only used if USE_RATE_LIMIT=true)

%% ----------------------- STATION / SCALING CONSTANTS -----------------------
% Actuator (ESP32 DAC -> amplifier -> station input)
U_DAC_MIN        = 0.0;
U_DAC_MAX        = 3.3;
U_STATION_MIN    = 1.0;
U_STATION_MAX    = 5.0;

% Station input offset used in Estimator.m (model input = station_input - U_OFFSET_V)
U_OFFSET_V       = 0.10;

% Measurement scaling + temp conversion (Estimator.m)
G_div            = 1.5;
V_SENSOR_OFFSET  = 0.15;
TEMP_A           = 36;
TEMP_B           = -24.315;

V_ADC_MIN        = 0.67;
V_ADC_MAX        = 3.30;

% DAC->station mapping (assumes linear map 0V->1V and 3.3V->5V)
GAIN_DAC2STATION = (U_STATION_MAX - U_STATION_MIN) / (U_DAC_MAX - U_DAC_MIN);  % 4/3.3
OFFS_DAC2STATION = U_STATION_MIN;

%% ----------------------- PICK PLANT(S) -----------------------
if RUN_ALL_PLANTS
    plantNames = string(fieldnames(plants));
else
    if ~isfield(plants, PLANT_TO_USE)
        plantNames = string(fieldnames(plants));
        warning("Requested %s not found. Will use %s.", PLANT_TO_USE, plantNames(1));
        plantNames = plantNames(1);
    else
        plantNames = PLANT_TO_USE;
    end
end

%% ----------------------- ESTIMATE Ts, OPERATING POINT, NOISE -----------------------
Ts = 0.5; T0 = 25.0; sigmaT_meas = 0.2;
U_STATION_BIAS = 1.0;   % will be overwritten if NOISE_FILE exists

dataPath = fullfile(DATA_DIR, NOISE_FILE);
if exist(dataPath,'file')==2
    tbl = readtable(dataPath);
    t_s = (tbl.timestamp_ms - tbl.timestamp_ms(1))/1000;
    Ts = median(diff(t_s));

    % Operating point: mean of first PRE_SEC seconds of command
    idx0 = find(t_s <= min(PRE_SEC, t_s(end)));
    if isempty(idx0), idx0 = 1:min(10,height(tbl)); end
    U_STATION_BIAS = mean(tbl.setpoint_v(idx0));   % station-domain volts (as logged)

    % Initial temperature from first ADC sample
    Vadc0 = tbl.sensor_v(1);
    T0 = adcToTemp(Vadc0, TEMP_A, TEMP_B, G_div, V_SENSOR_OFFSET);

    % Measurement noise estimate from first ~60 seconds
    tWin = min(60, t_s(end));
    idxN = find(t_s <= tWin);
    yT = adcToTemp(tbl.sensor_v(idxN), TEMP_A, TEMP_B, G_div, V_SENSOR_OFFSET);
    p = polyfit(t_s(idxN), yT, 1);
    yDetrend = yT - polyval(p, t_s(idxN));
    sigmaT_meas = std(yDetrend);

    fprintf("Estimated Ts=%.4g s, Bias U_station=%.3f V, T0=%.3f C, sigmaT=%.4g C\n", Ts, U_STATION_BIAS, T0, sigmaT_meas);
else
    warning("Could not find %s. Using fallback Ts/T0/bias/noise.", dataPath);
end

%% ----------------------- SIM SETUP -----------------------
tEnd = SIM_TIME_MIN*60;
N = max(200, round(tEnd/Ts));
t = (0:N-1)'*Ts;

Tref = T0*ones(N,1);
Tref(t >= T_STEP_TIME_S) = T0 + T_REF_STEP_C;

results = struct();

%% ----------------------- LOOP: DESIGN + SIM -----------------------
for ii = 1:numel(plantNames)
    name = plantNames(ii);
    Gct  = plants.(name);

    % Plant model is ΔT / Δu (from Estimator). We'll simulate deviations.
    Pct = minreal(ss(Gct));
    Pdt = c2d(Pct, Ts, 'zoh');

    A = Pdt.A; B = Pdt.B; C = Pdt.C; D = Pdt.D;
    nx = size(A,1);

    % ---- LQR with integrator on output error (servo) ----
    % x_a = [x; xi], where xi integrates (Tref - T_used)
    Aa = [A            zeros(nx,1);
         -Ts*C         1          ];
    Ba = [B;
         -Ts*D          ];

    % Normalized-ish weights:
    T_ERR_MAX  = max(2, 0.25*T_REF_STEP_C); % allowed temp error scale (degC)
    DU_MAX     = 1.5;                       % allowed control deviation scale (V)

    qy = 1/(T_ERR_MAX^2);
    Qx = qy * (C'*C);      % penalize output via states

    qi = 10*qy;            % integrator weight (keep moderate to avoid overshoot)
    Q  = blkdiag(Qx, qi);

    R0 = 1/(DU_MAX^2);
    R  = R0 / max(0.2,AGGRESSIVENESS)^2;  % <-- THIS is the aggressiveness knob

    K = dlqr(Aa, Ba, Q, R);
    Kx = K(1:nx);
    Ki = K(nx+1);

    % ---- Kalman filter ----
    V = sigmaT_meas^2;
    W = (0.02)^2 * eye(nx);      % keep fixed; you can tune later
    L = dlqe(A, eye(nx), C, W, V);

    % ---- Simulation (deviation variables) ----
    x = zeros(nx,1);      % true deviation state
    xhat = zeros(nx,1);   % estimated deviation state
    xi = 0;

    du_prev = 0;          % previous Δu (station volts deviation)

    % ADC moving average buffer
    if USE_ADC_MOVING_AVG
        Vadc_init = invTempToAdc(T0, TEMP_A, TEMP_B, G_div, V_SENSOR_OFFSET);
        buf = Vadc_init*ones(ADC_AVG_N,1);
        bufIdx = 1;
    end

    T_true_log = zeros(N,1);
    T_meas_log = zeros(N,1);
    Vadc_log   = zeros(N,1);

    u_station_log = zeros(N,1);
    du_log        = zeros(N,1);
    u_dac_log     = zeros(N,1);

    satCount = 0;

    for k = 1:N
        % True plant output deviation ΔT
        dT_true = C*x + D*du_prev;
        T_true  = T0 + dT_true;

        % Sensor chain
        Vadc_true = invTempToAdc(T_true, TEMP_A, TEMP_B, G_div, V_SENSOR_OFFSET);
        if USE_MEAS_NOISE
            Vadc_noisy = Vadc_true + (sigmaT_meas/54)*randn();
        else
            Vadc_noisy = Vadc_true;
        end
        Vadc_clip = min(max(Vadc_noisy, V_ADC_MIN), V_ADC_MAX);

        if USE_ADC_MOVING_AVG
            buf(bufIdx) = Vadc_clip;
            bufIdx = bufIdx + 1;
            if bufIdx > ADC_AVG_N, bufIdx = 1; end
            Vadc = mean(buf);
        else
            Vadc = Vadc_clip;
        end

        T_meas = adcToTemp(Vadc, TEMP_A, TEMP_B, G_div, V_SENSOR_OFFSET);

        % Observer update uses measured deviation ΔT_meas
        dT_meas = T_meas - T0;
        xhat = A*xhat + B*du_prev + L*(dT_meas - (C*xhat + D*du_prev));

        % Choose signal for integrator (estimate vs measurement)
        dT_hat = C*xhat + D*du_prev;
        T_hat  = T0 + dT_hat;

        if INTEGRATOR_ON_ESTIMATE
            e = Tref(k) - T_hat;
        else
            e = Tref(k) - T_meas;
        end
        if abs(e) < ERROR_DEADBAND_C
            e = 0;
        end

        % Candidate control (unsaturated) in deviation volts
        du_unsat = -Kx*xhat - Ki*xi;

        % Convert to absolute station command and saturate
        u_station_unsat = U_STATION_BIAS + du_unsat;
        u_station_sat   = min(max(u_station_unsat, U_STATION_MIN), U_STATION_MAX);
        du_sat          = u_station_sat - U_STATION_BIAS;

        if abs(u_station_sat - u_station_unsat) > 1e-9
            satCount = satCount + 1;
        end

        % Optional rate limit on deviation command
        if USE_RATE_LIMIT
            du_max = RATE_LIMIT_V_PER_S * Ts;
            ddu = du_sat - du_prev;
            ddu = min(max(ddu, -du_max), du_max);
            du_cmd = du_prev + ddu;
        else
            du_cmd = du_sat;
        end

        % Anti-windup (back-calculation):
        % xi_dot = e + (du_cmd - du_unsat)/Taw
        xi = xi + Ts*( e + (du_cmd - du_unsat)/max(1e-6,T_AW_SEC) );

        % Plant update
        x = A*x + B*du_cmd;
        du_prev = du_cmd;

        % Logs
        u_station = U_STATION_BIAS + du_cmd;
        u_dac = (u_station - OFFS_DAC2STATION)/GAIN_DAC2STATION;
        u_dac = min(max(u_dac, U_DAC_MIN), U_DAC_MAX);

        T_true_log(k) = T_true;
        T_meas_log(k) = T_meas;
        Vadc_log(k)   = Vadc;

        u_station_log(k) = u_station;
        du_log(k)        = du_cmd;
        u_dac_log(k)     = u_dac;
    end

    satPct = 100*satCount/N;

    results.(name).Kx = Kx;
    results.(name).Ki = Ki;
    results.(name).L  = L;
    results.(name).Ts = Ts;
    results.(name).U_station_bias = U_STATION_BIAS;
    results.(name).satPct = satPct;

    results.(name).T_true = T_true_log;
    results.(name).T_meas = T_meas_log;
    results.(name).Vadc   = Vadc_log;
    results.(name).u_station = u_station_log;
    results.(name).du        = du_log;
    results.(name).u_dac     = u_dac_log;
    results.(name).Tref      = Tref;

    % ---- Plots ----
    figure('Name', sprintf('Closed-loop Temperature - %s', name));
    plot(t/60, T_true_log, 'LineWidth', 1.4); hold on;
    plot(t/60, T_meas_log, '--', 'LineWidth', 1.1);
    plot(t/60, Tref, ':', 'LineWidth', 1.6);
    grid on; xlabel('Time (min)'); ylabel('Temperature (°C)');
    title(sprintf('LQG-Servo | %s | Ts=%.3gs | Agg=%.2f | Sat=%.1f%%', name, Ts, AGGRESSIVENESS, satPct));
    legend('True T', 'Measured T', 'Reference', 'Location', 'best');

    figure('Name', sprintf('Station Command - %s', name));
    plot(t/60, u_station_log, 'LineWidth', 1.4); hold on;
    yline(U_STATION_MIN, '--'); yline(U_STATION_MAX, '--');
    grid on; xlabel('Time (min)'); ylabel('u_{station} (V)');
    title(sprintf('Absolute station command | Bias=%.3fV | %s', U_STATION_BIAS, name));
    legend('u_{station}', 'Limits', 'Location', 'best');

    figure('Name', sprintf('ESP32 DAC and ADC - %s', name));
    plot(t/60, u_dac_log, 'LineWidth', 1.4); hold on;
    plot(t/60, Vadc_log, '--', 'LineWidth', 1.2);
    yline(U_DAC_MIN, ':'); yline(U_DAC_MAX, ':');
    yline(V_ADC_MIN, ':'); yline(V_ADC_MAX, ':');
    grid on; xlabel('Time (min)'); ylabel('Volts');
    title(sprintf('ESP32-side DAC/ADC | %s', name));
    legend('u_{DAC} (0..3.3V)', 'V_{ADC} (0.67..3.3V)', 'Limits', 'Location', 'best');

    fprintf("\n[%s] Agg=%.2f => Ki=%.6g, ||Kx||=%.6g, saturation=%.1f%%\n", ...
        name, AGGRESSIVENESS, Ki, norm(Kx), satPct);
end

assignin('base', 'LQG_results', results);
fprintf("\nDone. Results saved in workspace variable: LQG_results\n");

%% ----------------------- Helper functions -----------------------
function T = adcToTemp(V_adc, TEMP_A, TEMP_B, G_div, V_SENSOR_OFFSET)
    T = TEMP_A * ( G_div * (V_adc + V_SENSOR_OFFSET) ) + TEMP_B;
end

function V_adc = invTempToAdc(T, TEMP_A, TEMP_B, G_div, V_SENSOR_OFFSET)
    V_adc = (T - TEMP_B) / (TEMP_A * G_div) - V_SENSOR_OFFSET;
end
