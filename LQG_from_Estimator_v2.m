%% LQG_from_Estimator_v2.m
% -------------------------------------------------------------------------
% v2 changes (to address "noisy DAC" + "noisy ADC" plots):
%   1) Uses INTEGRATOR_ON_ESTIMATE (default true) so the integrator does NOT
%      integrate raw measurement noise.
%   2) Adds ADC moving-average (default N=10) similar to what you do on ESP32.
%   3) Adds actuator rate limiter (default 0.05 V/s in station-domain).
%   4) Adds small integrator leak + deadband to prevent random-walk at steady state.
%
% Place next to Estimator.m and run this script.
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

T_REF_STEP_C     = 20;
T_STEP_TIME_S    = 10;
SIM_TIME_MIN     = 90;

AGGRESSIVENESS   = 3000;    % 0.6..2.0

DATA_DIR         = "./data";
NOISE_FILE       = "data2.csv";

%% ----------------------- NOISE / FILTER SETTINGS -----------------------
USE_MEAS_NOISE           = false;   % set false for "clean" plots
USE_ADC_MOVING_AVG       = true;   % set false to see raw ADC noise
ADC_AVG_N                = 10;     % moving average samples
INTEGRATOR_ON_ESTIMATE   = true;   % << recommended
ERROR_DEADBAND_C         = 0.05;   % don't integrate tiny errors
INTEGRATOR_LEAK_PER_S    = 0.001;  % small leak to stop random-walk (1/s)

% Station actuator smoothing
USE_RATE_LIMIT           = true;
RATE_LIMIT_V_PER_S       = 0.05;   % station volts per second (adjust)

%% ----------------------- STATION / SCALING CONSTANTS -----------------------
U_DAC_MIN        = 0.0;
U_DAC_MAX        = 3.3;
U_STATION_MIN    = 1.0;
U_STATION_MAX    = 5.0;

U_OFFSET_V       = 0.10;

G_div            = 1.5;
V_SENSOR_OFFSET  = 0.15;
TEMP_A           = 36;
TEMP_B           = -24.315;

V_ADC_MIN        = 0.67;
V_ADC_MAX        = 3.30;

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

%% ----------------------- GET Ts, T0, sigmaT -----------------------
Ts = 0.5; T0 = 25.0; sigmaT_meas = 0.2;

dataPath = fullfile(DATA_DIR, NOISE_FILE);
if exist(dataPath,'file')==2
    tbl = readtable(dataPath);
    t_s = (tbl.timestamp_ms - tbl.timestamp_ms(1))/1000;
    Ts = median(diff(t_s));

    Vadc0 = tbl.sensor_v(1);
    T0 = adcToTemp(Vadc0, TEMP_A, TEMP_B, G_div, V_SENSOR_OFFSET);

    tWin = min(60, t_s(end));
    idx = find(t_s <= tWin);
    yT = adcToTemp(tbl.sensor_v(idx), TEMP_A, TEMP_B, G_div, V_SENSOR_OFFSET);
    p = polyfit(t_s(idx), yT, 1);
    yDetrend = yT - polyval(p, t_s(idx));
    sigmaT_meas = std(yDetrend);

    fprintf("Estimated Ts=%.4g s, T0=%.3f C, sigmaT=%.4g C\n", Ts, T0, sigmaT_meas);
else
    warning("Noise file not found. Using fallback Ts/T0/sigmaT.");
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

    Pct = minreal(ss(Gct));
    Pdt = c2d(Pct, Ts, 'zoh');

    A = Pdt.A; B = Pdt.B; C = Pdt.C; D = Pdt.D;
    nx = size(A,1);

    % ---- LQR with integrator (servo) ----
    Aa = [A            zeros(nx,1);
         -Ts*C         1          ];
    Ba = [B;
         -Ts*D          ];

    T_ERR_MAX  = max(2, 0.25*T_REF_STEP_C);
    U_MOVE_MAX = 1.5;

    qy = (AGGRESSIVENESS) * (1/(T_ERR_MAX^2));
    Qx = qy * (C'*C);
    qi = (AGGRESSIVENESS) * 20*qy;

    Q  = blkdiag(Qx, qi);
    R  = 1/((U_MOVE_MAX/max(0.2,AGGRESSIVENESS))^2);

    K = dlqr(Aa, Ba, Q, R);
    Kx = K(1:nx);
    Ki = K(nx+1);

    % ---- Kalman filter ----
    V = sigmaT_meas^2;
    W = (0.02*AGGRESSIVENESS)^2 * eye(nx);
    L = dlqe(A, eye(nx), C, W, V);

    % ---- Simulation ----
    x = zeros(nx,1);
    xhat = zeros(nx,1);
    xi = 0;

    u_station = U_STATION_MIN;
    u_model   = u_station - U_OFFSET_V;
    u_station_prev = u_station;

    % ADC moving average buffer
    if USE_ADC_MOVING_AVG
        buf = Vadc0 * ones(ADC_AVG_N,1);
        bufIdx = 1;
    end

    T_true_log = zeros(N,1);
    T_meas_log = zeros(N,1);
    Vadc_log   = zeros(N,1);
    u_station_log = zeros(N,1);
    u_dac_log  = zeros(N,1);

    for k = 1:N
        % True plant output (ΔT)
        dT_true = C*x + D*u_model;
        T_true  = T0 + dT_true;

        % Sensor chain (T -> Vadc), plus optional noise and averaging
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

        % Observer update using measured ΔT
        dT_meas = T_meas - T0;
        xhat = A*xhat + B*u_model + L*(dT_meas - (C*xhat + D*u_model));

        % Estimated temperature (used for integral if enabled)
        dT_hat = C*xhat + D*u_model;
        T_hat  = T0 + dT_hat;

        % Integrator error signal
        if INTEGRATOR_ON_ESTIMATE
            e = Tref(k) - T_hat;
        else
            e = Tref(k) - T_meas;
        end

        % Deadband
        if abs(e) < ERROR_DEADBAND_C
            e = 0;
        end

        % Integrator with leak (prevents random-walk)
        xi = (1 - INTEGRATOR_LEAK_PER_S*Ts)*xi + Ts*e;

        % LQR control (model-domain)
        u_model_unsat = -Kx*xhat - Ki*xi;

        % Convert to station-domain and saturate
        u_station_unsat = u_model_unsat + U_OFFSET_V;
        u_station_sat   = min(max(u_station_unsat, U_STATION_MIN), U_STATION_MAX);

        % Optional rate limit in station-domain
        if USE_RATE_LIMIT
            du_max = RATE_LIMIT_V_PER_S * Ts;
            du = u_station_sat - u_station_prev;
            du = min(max(du, -du_max), du_max);
            u_station = u_station_prev + du;
        else
            u_station = u_station_sat;
        end
        u_station_prev = u_station;

        u_model = u_station - U_OFFSET_V;

        % True plant update
        x = A*x + B*u_model;

        % Convert station command to DAC volts
        u_dac = (u_station - OFFS_DAC2STATION) / GAIN_DAC2STATION;
        u_dac = min(max(u_dac, U_DAC_MIN), U_DAC_MAX);

        % Log
        T_true_log(k) = T_true;
        T_meas_log(k) = T_meas;
        Vadc_log(k)   = Vadc;
        u_station_log(k) = u_station;
        u_dac_log(k)  = u_dac;
    end

    results.(name).Kx = Kx;
    results.(name).Ki = Ki;
    results.(name).L  = L;
    results.(name).Ts = Ts;

    results.(name).T_true = T_true_log;
    results.(name).T_meas = T_meas_log;
    results.(name).Vadc   = Vadc_log;
    results.(name).u_station = u_station_log;
    results.(name).u_dac     = u_dac_log;
    results.(name).Tref      = Tref;

    % ---- Plots ----
    figure('Name', sprintf('Closed-loop Temperature - %s', name));
    plot(t/60, T_true_log, 'LineWidth', 1.4); hold on;
    plot(t/60, T_meas_log, '--', 'LineWidth', 1.1);
    plot(t/60, Tref, ':', 'LineWidth', 1.6);
    grid on; xlabel('Time (min)'); ylabel('Temperature (°C)');
    title(sprintf('LQG-Servo Closed-loop Step | %s | Ts=%.3gs | Agg=%.2f', name, Ts, AGGRESSIVENESS));
    legend('True T', 'Measured T', 'Reference', 'Location', 'best');

    figure('Name', sprintf('Actuator Command - %s', name));
    plot(t/60, u_station_log, 'LineWidth', 1.4); hold on;
    yline(U_STATION_MIN, '--'); yline(U_STATION_MAX, '--');
    grid on; xlabel('Time (min)'); ylabel('Station input u (V)');
    title(sprintf('Actuator (station-domain) | %s', name));
    legend('u_{station}', 'Limits', 'Location', 'best');

    figure('Name', sprintf('ESP32 DAC and ADC - %s', name));
    plot(t/60, u_dac_log, 'LineWidth', 1.4); hold on;
    plot(t/60, Vadc_log, '--', 'LineWidth', 1.2);
    yline(U_DAC_MIN, ':'); yline(U_DAC_MAX, ':');
    yline(V_ADC_MIN, ':'); yline(V_ADC_MAX, ':');
    grid on; xlabel('Time (min)'); ylabel('Volts');
    title(sprintf('ESP32-side DAC and ADC | %s', name));
    legend('u_{DAC} (0..3.3V)', 'V_{ADC} (0.67..3.3V)', 'Limits', 'Location', 'best');

    fprintf("\n[%s] Gains: Ki=%.6g, ||Kx||=%.6g\n", name, Ki, norm(Kx));
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
