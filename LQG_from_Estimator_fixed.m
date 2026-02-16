%% LQG_from_Estimator.m
% -------------------------------------------------------------------------
% FIXED VERSION:
%   Estimator.m may contain "clear" which wipes variables defined earlier in
%   this script. So we FIRST ensure G1/G2/G3 exist (running Estimator.m if
%   needed), THEN we define all user settings and continue.
%
% LQG (Kalman + LQR) with integral action (servo) for your temperature station.
%
% How to use:
%   1) Put this file in the same folder as Estimator.m (or add that folder to path).
%   2) Run:  LQG_from_Estimator
%
% Output:
%   - Figures for closed-loop temperature, actuator saturation, DAC/ADC voltages
%   - Workspace struct: LQG_results
% -------------------------------------------------------------------------
clear; clc; close all;

%% ----------------------- ENSURE TRANSFER FUNCTIONS EXIST -----------------------
% If Estimator.m runs, it may clear the workspace, so do this BEFORE settings.
if ~(exist('G1','var')==1 || exist('G2','var')==1 || exist('G3','var')==1)
    if exist('Estimator.m','file')==2
        fprintf("Running Estimator.m to create G1,G2,G3 in the workspace...\n");
        run('Estimator.m');   % may "clear"; we re-define everything after this block
    else
        error("Estimator.m not found on MATLAB path. Put this script next to Estimator.m or add its folder to path.");
    end
end

% Collect available plants (after Estimator)
plants = struct();
if exist('G1','var')==1, plants.G1 = G1; end
if exist('G2','var')==1, plants.G2 = G2; end
if exist('G3','var')==1, plants.G3 = G3; end
if isempty(fieldnames(plants))
    error("No transfer functions found (G1/G2/G3). Run Estimator.m successfully first.");
end

%% ----------------------- USER SETTINGS -----------------------
RUN_ALL_PLANTS   = false;    % true -> design/sim for all available G1,G2,G3
PLANT_TO_USE     = "G2";     % "G1" or "G2" or "G3" (ignored if RUN_ALL_PLANTS=true)

% Closed-loop test: temperature reference step (degC)
T_REF_STEP_C     = 20;       % step size in degrees C (relative to initial temperature)
T_STEP_TIME_S    = 10;       % step time (seconds)
SIM_TIME_MIN     = 90;       % simulation length (minutes)

% One-knob tuning (bigger -> faster, but more actuator stress / risk of overshoot)
AGGRESSIVENESS   = 0.5;      % try 0.6 .. 2.0

% If data files are available, we'll estimate Ts, initial temperature, and noise from one file.
DATA_DIR         = "./data";
NOISE_FILE       = "data2.csv";   % used only to estimate Ts, T0, and sigma_T

%% ----------------------- STATION / SCALING CONSTANTS -----------------------
% Actuator (ESP32 DAC -> amplifier -> station input)
U_DAC_MIN        = 0.0;      % ESP32 side (V)
U_DAC_MAX        = 3.3;      % ESP32 side (V)
U_STATION_MIN    = 1.0;      % station input (V)
U_STATION_MAX    = 5.0;      % station input (V)

% Station input offset used in Estimator.m (model input = station_input - U_OFFSET_V)
U_OFFSET_V       = 0.10;

% Measurement scaling + temp conversion used in Estimator.m:
% T = 36*(G_div*(V_adc + 0.15)) - 24.315
G_div            = 1.5;
V_SENSOR_OFFSET  = 0.15;
TEMP_A           = 36;
TEMP_B           = -24.315;

% ADC rails (after divider)
V_ADC_MIN        = 0.67;
V_ADC_MAX        = 3.30;

% DAC->station gain (assumes linear map 0V->1V and 3.3V->5V)
GAIN_DAC2STATION = (U_STATION_MAX - U_STATION_MIN) / (U_DAC_MAX - U_DAC_MIN);  % 4/3.3
OFFS_DAC2STATION = U_STATION_MIN;  % u_station = OFFS + GAIN*u_dac

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

%% ----------------------- GET Ts, T0, and MEASUREMENT NOISE (OPTIONAL) -----------------------
Ts = 0.5;          % fallback
T0 = 25.0;         % fallback initial temperature (degC)
sigmaT_meas = 0.2; % fallback temperature measurement std (degC)

dataPath = fullfile(DATA_DIR, NOISE_FILE);
if exist(dataPath,'file')==2
    tbl = readtable(dataPath);
    t_s = (tbl.timestamp_ms - tbl.timestamp_ms(1))/1000;
    Ts = median(diff(t_s));

    % Initial temperature from the first ADC sample
    Vadc0 = tbl.sensor_v(1);
    T0 = adcToTemp(Vadc0, TEMP_A, TEMP_B, G_div, V_SENSOR_OFFSET);

    % Estimate measurement noise from an early window (first ~60 seconds)
    tWin = min(60, t_s(end));
    idx = find(t_s <= tWin);
    yT = adcToTemp(tbl.sensor_v(idx), TEMP_A, TEMP_B, G_div, V_SENSOR_OFFSET);
    p = polyfit(t_s(idx), yT, 1);
    yDetrend = yT - polyval(p, t_s(idx));
    sigmaT_meas = std(yDetrend);

    fprintf("Estimated Ts = %.4g s, T0 = %.3f C, sigma_T(meas) = %.4g C (from %s)\n", Ts, T0, sigmaT_meas, NOISE_FILE);
else
    warning("Could not find %s. Using fallback Ts=%.3g s, T0=%.1f C, sigmaT=%.2f C.", dataPath, Ts, T0, sigmaT_meas);
end

%% ----------------------- SIMULATION SETTINGS -----------------------
tEnd = SIM_TIME_MIN * 60;
N = max(200, round(tEnd / Ts));
t = (0:N-1)' * Ts;

Tref = T0 * ones(N,1);
Tref(t >= T_STEP_TIME_S) = T0 + T_REF_STEP_C;

%% ----------------------- DESIGN + SIM LOOP -----------------------
results = struct();

for ii = 1:numel(plantNames)
    name = plantNames(ii);
    Gct = plants.(name);

    % Convert to state-space and discretize
    Pct = minreal(ss(Gct));
    Pdt = c2d(Pct, Ts, 'zoh');

    A = Pdt.A; B = Pdt.B; C = Pdt.C; D = Pdt.D;
    nx = size(A,1);

    % --------- LQR with integral action (discrete) ----------
    % Augment with integrator: xi[k+1] = xi[k] + Ts*(Tref - Tmeas)
    Aa = [A            zeros(nx,1);
         -Ts*C         1          ];
    Ba = [B;
         -Ts*D          ];

    % Weighting via output-penalty (works even if ss states are not physical):
    T_ERR_MAX  = max(2, 0.25*T_REF_STEP_C); % degC
    U_MOVE_MAX = 1.5;                        % V (station-domain) typical move allowance

    qy = (AGGRESSIVENESS) * (1/(T_ERR_MAX^2));
    Qx = qy * (C'*C);

    qi = (AGGRESSIVENESS) * 20*qy;           % integral weight (tune)
    Q  = blkdiag(Qx, qi);

    R  = 1/((U_MOVE_MAX/max(0.2,AGGRESSIVENESS))^2);

    K = dlqr(Aa, Ba, Q, R);
    Kx = K(1:nx);
    Ki = K(nx+1);

    % --------- Kalman filter (discrete) ----------
    V = sigmaT_meas^2;                           % measurement noise variance (degC^2)
    W = (0.02*AGGRESSIVENESS)^2 * eye(nx);       % process noise variance (tune knob)
    L = dlqe(A, eye(nx), C, W, V);

    % --------- Closed-loop simulation with saturation + scaling ----------
    x    = zeros(nx,1);         % true plant state (Δ)
    xhat = zeros(nx,1);         % estimated plant state (Δ)
    xi   = 0;                   % integrator
    u_station = U_STATION_MIN;  % station input command (V)
    u_model   = u_station - U_OFFSET_V;

    T_true_log    = zeros(N,1);
    T_meas_log    = zeros(N,1);
    Vadc_log      = zeros(N,1);
    u_station_log = zeros(N,1);
    u_dac_log     = zeros(N,1);

    for k = 1:N
        % True plant output (temperature deviation)
        dT_true = C*x + D*u_model;   % ΔT
        T_true = T0 + dT_true;

        % Measurement chain: T_true -> Vadc -> clip -> add noise -> back to T_meas
        Vadc_true  = invTempToAdc(T_true, TEMP_A, TEMP_B, G_div, V_SENSOR_OFFSET);

        % T = 54*Vadc - 16.215 => dT/dV = 54 => sigmaV ~ sigmaT/54
        Vadc_noisy = Vadc_true + (sigmaT_meas/54)*randn();

        Vadc = min(max(Vadc_noisy, V_ADC_MIN), V_ADC_MAX);
        T_meas = adcToTemp(Vadc, TEMP_A, TEMP_B, G_div, V_SENSOR_OFFSET);

        % Observer update (uses measured ΔT)
        dT_meas = T_meas - T0;
        xhat = A*xhat + B*u_model + L*(dT_meas - (C*xhat + D*u_model));

        % Integrator + anti-windup
        e = (Tref(k) - T_meas);       % temperature error (degC)
        xi_candidate = xi + Ts*e;

        % Control in MODEL input coordinates: u_model = u_station - U_OFFSET_V
        u_model_unsat = -Kx*xhat - Ki*xi_candidate;

        % Convert to station-domain and saturate [1,5]V
        u_station_unsat = u_model_unsat + U_OFFSET_V;
        u_station_sat   = min(max(u_station_unsat, U_STATION_MIN), U_STATION_MAX);
        u_model_sat     = u_station_sat - U_OFFSET_V;

        % Anti-windup clamp
        if abs(u_station_sat - u_station_unsat) < 1e-9
            xi = xi_candidate;
        end

        u_station = u_station_sat;
        u_model   = u_model_sat;

        % True plant update
        x = A*x + B*u_model;

        % Convert station command to DAC volts (for reporting)
        u_dac = (u_station - OFFS_DAC2STATION) / GAIN_DAC2STATION;
        u_dac = min(max(u_dac, U_DAC_MIN), U_DAC_MAX);

        % Log
        T_true_log(k)    = T_true;
        T_meas_log(k)    = T_meas;
        Vadc_log(k)      = Vadc;
        u_station_log(k) = u_station;
        u_dac_log(k)     = u_dac;
    end

    % Store
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

    % --------- Plots ----------
    figure('Name', sprintf('Closed-loop Temperature - %s', name));
    plot(t/60, T_true_log, 'LineWidth', 1.4); hold on;
    plot(t/60, T_meas_log, '--', 'LineWidth', 1.1);
    plot(t/60, Tref, ':', 'LineWidth', 1.6);
    grid on;
    xlabel('Time (min)');
    ylabel('Temperature (°C)');
    title(sprintf('LQG-Servo Closed-loop Step | %s | Ts=%.3gs | Agg=%.2f', name, Ts, AGGRESSIVENESS));
    legend('True T', 'Measured T', 'Reference', 'Location', 'best');

    figure('Name', sprintf('Actuator Command - %s', name));
    plot(t/60, u_station_log, 'LineWidth', 1.4); hold on;
    yline(U_STATION_MIN, '--'); yline(U_STATION_MAX, '--');
    grid on;
    xlabel('Time (min)');
    ylabel('Station input u (V)');
    title(sprintf('Actuator (station-domain) with saturation | %s', name));
    legend('u_{station}', 'Limits', 'Location', 'best');

    figure('Name', sprintf('ESP32 DAC and ADC - %s', name));
    plot(t/60, u_dac_log, 'LineWidth', 1.4); hold on;
    plot(t/60, Vadc_log, '--', 'LineWidth', 1.2);
    yline(U_DAC_MIN, ':'); yline(U_DAC_MAX, ':');
    yline(V_ADC_MIN, ':'); yline(V_ADC_MAX, ':');
    grid on;
    xlabel('Time (min)');
    ylabel('Volts');
    title(sprintf('ESP32-side DAC (command) and ADC (measurement) | %s', name));
    legend('u_{DAC} (0..3.3V)', 'V_{ADC} (0.67..3.3V)', 'Limits', 'Location', 'best');

    fprintf("\n[%s] Designed gains:\n", name);
    fprintf("  Kx size: %dx%d, Ki: %.6g\n", size(Kx,1), size(Kx,2), Ki);
    fprintf("  Kalman L size: %dx%d\n", size(L,1), size(L,2));
end

assignin('base', 'LQG_results', results);
fprintf("\nDone. Results are saved in workspace variable: LQG_results\n");

%% ----------------------- Local helper functions -----------------------
function T = adcToTemp(V_adc, TEMP_A, TEMP_B, G_div, V_SENSOR_OFFSET)
    % Forward conversion used in Estimator.m:
    % T = TEMP_A * ( G_div * (V_adc + V_SENSOR_OFFSET) ) + TEMP_B
    T = TEMP_A * ( G_div * (V_adc + V_SENSOR_OFFSET) ) + TEMP_B;
end

function V_adc = invTempToAdc(T, TEMP_A, TEMP_B, G_div, V_SENSOR_OFFSET)
    % Inverse of adcToTemp:
    % V_adc = (T - TEMP_B)/(TEMP_A*G_div) - V_SENSOR_OFFSET
    V_adc = (T - TEMP_B) / (TEMP_A * G_div) - V_SENSOR_OFFSET;
end
