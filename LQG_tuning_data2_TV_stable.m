%% Robust LQG Servo Tuning (Stable) using data/data2.csv as training
% هدف هذا السكربت: تصميم LQG (Kalman + LQI) أكثر استقراراً بدون تجاوز حدود المحطة.
%
% Why your previous tuning oscillated:
%   - LQI can create complex poles (under-damped) if weights are aggressive.
%   - Saturation without anti-windup can cause integrator windup and oscillations.
%   - A thermal plant often has delay; ignoring it reduces stability margin.
%
% What this script improves:
%   1) Identifies a First-Order-Plus-Delay model (P1D) from data2.csv (if possible)
%   2) Approximates delay with a low-order Pade for state-space design
%   3) Designs a discrete LQG servo (Kalman + LQI) with:
%        - damping-ratio constraint on linear poles (zeta >= zeta_min)
%        - reference prefilter (first-order) to reduce kick
%        - anti-windup (back-calculation + conditional integration)
%   4) Grid-searches weights to get the fastest settling time while keeping:
%        - overshoot <= OS_max
%        - stable / well-damped poles
%        - actuator saturation u in [1, 5] V
%
% IMPORTANT:
%   Settling time <= 30 minutes may not be physically achievable with u∈[1,5]V
%   depending on plant time constant and delay. The script will report the best feasible result.
%
% Requires:
%   - System Identification Toolbox (procest)
%   - Control System Toolbox (ss, c2d, dlqr, dlqe, pade)
%
% -------------------------------------------------------------------------

clear; clc; close all;

%% --------------------------- USER SETTINGS ---------------------------
dataDir  = "./data";
dataFile = "data2.csv";                 % training dataset

% Actuator absolute voltage limits (your station)
U_MIN = 1.0;                            % V
U_MAX = 5.0;                            % V

% Temperature conversion (your linear equation)
G_div = 1.5;                            % undo divider (if divider is 2/3)
V_SENSOR_OFFSET = 0.15;
TEMP_GAIN       = 36;
TEMP_BIAS       = -24.315;
V2T = @(Vadc) TEMP_GAIN .* ( G_div .* (Vadc + V_SENSOR_OFFSET) ) + TEMP_BIAS;

% Identification
STEP_THRESH = 0.05;                     % detect step in input (V)
N_PREPEND_ZEROS = 10;

% Controller sampling time (s): use something reasonable for thermal (1–5 s)
Ts_ctrl = 2.0;

% Performance targets / constraints
Ts_target_sec = 30*60;                  % 30 min target (s)
zeta_min = 0.8;                         % require well-damped poles (reduces oscillations)
OS_max  = 0.05;                         % max overshoot (5%)
SETTLE_PCT = 0.02;                      % 2% settling band
SETTLE_MIN_ABS = 0.5;                   % at least 0.5°C band

% Simulation horizon
Tsim = 2*Ts_target_sec;                 % simulate up to 60 min

% Delay approximation
PADE_ORDER = 3;                         % 3rd-order Pade is a good compromise

%% --------------------------- LOAD TRAINING DATA ---------------------------
tbl = readtable(fullfile(dataDir, dataFile));

t = tbl.timestamp_ms/1000;
t = t - t(1);

u_raw = tbl.setpoint_v;                 % station input command (V)
y_vadc = tbl.sensor_v;                  % ADC-side sensor voltage (V)
y_T = V2T(y_vadc);                      % Temperature (°C)

% Step detection in the raw command
stepIdx = find(abs([0; diff(u_raw)]) > STEP_THRESH, 1, "first");
if isempty(stepIdx), stepIdx = 1; end

% Pre-step operating point
if stepIdx > 5
    u0_raw = mean(u_raw(1:stepIdx-1));
    T0 = mean(y_T(1:stepIdx-1));
else
    u0_raw = u_raw(1);
    T0 = y_T(1);
end

% Post-step aligned
tpost = t(stepIdx:end) - t(stepIdx);
upost = u_raw(stepIdx:end);
Tpost = y_T(stepIdx:end);

% Steady-state from last 10%
Nss = max(20, round(0.1*numel(upost)));
u_ss = mean(upost(end-Nss+1:end));
T_ss = mean(Tpost(end-Nss+1:end));

% Training reference target temperature: use the steady-state temperature from data2
T_ref = T_ss;
dT_ref = T_ref - T0;

fprintf("Training: u0_raw=%.3f V, u_ss=%.3f V | T0=%.2f°C, T_ref=%.2f°C\n", u0_raw, u_ss, T0, T_ref);

%% --------------------------- IDENTIFY PLANT: ΔT / Δu ---------------------------
% Use deviation variables around the measured pre-step point (u0_raw, T0).
du = upost - u0_raw;
dT = Tpost - T0;

Ts_data = median(diff(tpost));
du_id = [zeros(N_PREPEND_ZEROS,1); du];
dT_id = [zeros(N_PREPEND_ZEROS,1); dT];

z = iddata(dT_id, du_id, Ts_data);
z.TimeUnit = "s";

optID = procestOptions;
optID.Focus = "simulation";
optID.InitialCondition = "zero";
optID.Display = "off";

% Try P1D (first-order + delay). If not available, fall back to P1.
try
    sys = procest(z, "P1D", optID);
    hasDelay = true;
catch
    sys = procest(z, "P1", optID);
    hasDelay = false;
end
[~, fitID] = compare(z, sys);

Kp  = sys.Kp;
tau = sys.Tp1;
if hasDelay
    % For procest P1D, delay is usually in property Td
    if isprop(sys, "Td")
        L = sys.Td;
    else
        L = sys.InputDelay;
    end
else
    L = 0;
end

fprintf("\nIdentified model: K=%.6g, tau=%.6g s, delay L=%.3f s | Fit=%.2f%%\n", Kp, tau, L, fitID);

G_nodelay = tf(Kp, [tau 1]);            % ΔT/Δu
if L > 0
    [numD, denD] = pade(L, PADE_ORDER);
    D = tf(numD, denD);                 % approx e^{-Ls}
    Gc = minreal(G_nodelay * D, 1e-8);
else
    Gc = G_nodelay;
end

% State-space and discretize
Gss = minreal(ss(Gc), 1e-8);
Gd = c2d(Gss, Ts_ctrl, "zoh");
Ad = Gd.A; Bd = Gd.B; Cd = Gd.C; Dd = Gd.D;

n = size(Ad,1);

%% --------------------------- KALMAN FILTER (DISCRETE) ---------------------------
% Measurement noise estimate from steady-state segment
Tss_seg = Tpost(end-Nss+1:end);
sigma_y = std(Tss_seg - mean(Tss_seg));
Rv = max(sigma_y^2, 1e-6);

% Process noise: small but not zero. Scale with dynamics.
Qw_scale = 1e-4;
Qw = Qw_scale * eye(n);
Gnoise = eye(n);

Lk = dlqe(Ad, Gnoise, Cd, Qw, Rv);

%% --------------------------- LQI AUGMENTATION ---------------------------
% Integrator on deviation tracking error: e = dT_ref - dT
Aaug = [Ad, zeros(n,1);
        -Ts_ctrl*Cd, 1];
Baug = [Bd;
        -Ts_ctrl*Dd];

%% --------------------------- GRID SEARCH TUNING (STABILITY-FIRST) ---------------------------
% We search weights to get fastest Ts while meeting stability/overshoot constraints.
qI_list  = [1e2, 1e3, 1e4, 1e5];         % integrator penalty
qY_list  = [0.1, 1, 10, 100];            % output penalty scaling
rho_list = logspace(-1, 4, 30);          % control penalty R = rho

% Reference prefilter time constants (s). Using a little filtering often removes oscillation.
Tr_list  = [0, 30, 60, 120, 300];

% Anti-windup tracking time constants (s)
Tt_list  = [10, 30, 60, 120];

best = struct("Ts", inf, "OS", inf, "satFrac", inf, "Kaug", [], "qI", NaN, "qY", NaN, "rho", NaN, "Tr", NaN, "Tt", NaN, "sim", []);

for qI = qI_list
    for qY = qY_list
        % Output-focused Qx ~ qY * C'C
        Qx = qY * (Cd')*(Cd);
        Q = blkdiag(Qx, qI);

        for rho = rho_list
            R = rho;

            % LQR gain
            try
                Kaug = dlqr(Aaug, Baug, Q, R);
            catch
                continue;
            end

            % Check linear closed-loop damping
            Acl = Aaug - Baug*Kaug;
            zPoles = eig(Acl);
            sPoles = log(zPoles) / Ts_ctrl;            % map to continuous for damping estimate

            zeta = zeros(size(sPoles));
            for i = 1:numel(sPoles)
                if imag(sPoles(i)) == 0
                    zeta(i) = 1;
                else
                    zeta(i) = -real(sPoles(i)) / abs(sPoles(i));
                end
            end
            if any(real(sPoles) >= 0) || min(zeta) < zeta_min
                continue;   % reject under-damped / unstable
            end

            Kx = Kaug(1:n);
            Ki = Kaug(n+1);

            for Tr = Tr_list
                for Tt = Tt_list
                    sim = simulate_lqg_servo(Ad,Bd,Cd,Dd, Lk, Kx, Ki, Ts_ctrl, dT_ref, T0, u0_raw, ...
                                            U_MIN, U_MAX, Tsim, SETTLE_PCT, SETTLE_MIN_ABS, OS_max, Tr, Tt, Rv);

                    if ~sim.ok
                        continue;
                    end

                    % Primary objective: meet 30 min, then minimize Ts
                    if sim.Ts <= Ts_target_sec
                        isBetter = sim.Ts < best.Ts;
                    else
                        % If nothing meets target, keep best Ts anyway
                        isBetter = (isinf(best.Ts) || sim.Ts < best.Ts);
                    end

                    if isBetter
                        best.Ts = sim.Ts;
                        best.OS = sim.OS;
                        best.satFrac = sim.satFrac;
                        best.Kaug = Kaug;
                        best.qI = qI; best.qY=qY; best.rho=rho; best.Tr=Tr; best.Tt=Tt;
                        best.sim = sim;
                    end
                end
            end
        end
    end
end

if isempty(best.Kaug)
    error("No feasible stable tuning found. Try reducing zeta_min or expanding weight ranges.");
end

Kaug = best.Kaug;
Kx = Kaug(1:n);
Ki = Kaug(n+1);

fprintf("\n================= BEST STABLE LQG TUNING =================\n");
fprintf("qY=%.3g, qI=%.3g, rho=%.3g | Tr=%g s | Tt=%g s\n", best.qY, best.qI, best.rho, best.Tr, best.Tt);
fprintf("Settling time: %.1f s (%.1f min)\n", best.Ts, best.Ts/60);
fprintf("Overshoot: %.2f %%\n", 100*best.OS);
fprintf("Saturation fraction: %.1f %%\n", 100*best.satFrac);
fprintf("Kx = "); disp(Kx);
fprintf("Ki = %.6g\n", Ki);
fprintf("Kalman L = "); disp(Lk);

%% --------------------------- PLOTS ---------------------------
% Open-loop training (measured)
figure("Color","w","Name","Open-loop training vs STABLE tuned LQG (Temperature)");
plot(tpost, Tpost, "k", "LineWidth", 1.2); hold on;

% Tuned closed-loop sim
plot(best.sim.t, best.sim.Tabs, "r--", "LineWidth", 1.6);
yline(T_ref, "LineWidth", 1.2);
xline(Ts_target_sec, ":", "LineWidth", 1.2);

grid on;
xlabel("Time (s)");
ylabel("Temperature (°C)");
title(sprintf("Stable LQG tuning | Ts=%.1f min (target 30 min) | OS=%.1f%% | sat=%.1f%%", ...
    best.Ts/60, 100*best.OS, 100*best.satFrac));
legend("Open-loop measured T (data2)", "Simulated stable LQG", "T_{ref}", "30 min target", "Location","best");

figure("Color","w","Name","Stable LQG control input (absolute volts)");
plot(best.sim.t, best.sim.uabs, "LineWidth", 1.4); grid on;
yline(U_MIN, "--", "LineWidth", 1.0);
yline(U_MAX, "--", "LineWidth", 1.0);
xlabel("Time (s)");
ylabel("u (V)");
title("Stable LQG control input with saturation");

%% --------------------------- Helper: LQG servo simulation ---------------------------
function sim = simulate_lqg_servo(Ad,Bd,Cd,Dd, Lk, Kx, Ki, Ts, dT_ref, T0, u0_raw, Umin, Umax, Tsim, settlePct, settleMinAbs, OSmax, Tr, Tt, Rv)
% Simulates discrete plant in deviations with:
%  - Kalman estimator
%  - LQI feedback (state + integrator)
%  - Reference prefilter (optional, time constant Tr)
%  - Input saturation u_abs in [Umin, Umax]
%  - Anti-windup (conditional integration + back-calculation with time constant Tt)
%
% Returns:
%   sim.ok: true if overshoot <= OSmax and stable behavior
%   sim.Ts: settling time (s)
%   sim.OS: overshoot fraction
%   sim.satFrac: fraction of time saturated
%   sim.Tabs, sim.uabs over time

n = size(Ad,1);
N = max(1, round(Tsim / Ts));
t = (0:Ts:Ts*(N-1)).';

% Initial conditions
x = zeros(n,1);          % true state (deviation)
xhat = zeros(n,1);       % estimated state
xI = 0;                  % integrator state

% Reference filter state (deviation reference)
rF = 0;

% Choose operating point for actuator:
% We cannot command below Umin physically, so clamp baseline.
u0 = min(max(u0_raw, Umin), Umax);

Tabs = zeros(N,1);
u_hist = zeros(N,1);
satCount = 0;

% Precompute constants
alphaRef = 1;
if Tr > 0
    alphaRef = Ts / Tr;
end
alphaRef = min(max(alphaRef, 0), 1);

for k = 1:N
    % True output (deviation)
    dT = Cd*x + Dd*(0);   %#ok<NASGU>
    % We'll compute output after we apply du at end of loop, so keep last du variable:
    % (For clarity, store du from previous step)
    if k == 1
        du_prev = 0;
    else
        du_prev = (u_hist(k-1) - u0);
    end
    dT = Cd*x + Dd*du_prev;
    Tabs(k) = T0 + dT;

    % Reference prefilter (on deviation target)
    if Tr > 0
        rF = rF + alphaRef*(dT_ref - rF);
    else
        rF = dT_ref;
    end

    % Tracking error (deviation)
    e = rF - dT;

    % ---- Controller (unsaturated) ----
    du_unsat = -(Kx*xhat + Ki*xI);
    u_unsat = u0 + du_unsat;

    % ---- Saturate ----
    u_sat = min(max(u_unsat, Umin), Umax);
    du_sat = u_sat - u0;

    if u_sat ~= u_unsat
        satCount = satCount + 1;
    end
    u_hist(k) = u_sat;

    % ---- Anti-windup integrator update ----
    % Conditional integration: if saturated AND error would push further into saturation, freeze the "Ts*e" part.
    integrate = true;
    if u_sat >= Umax - 1e-9 && e > 0
        integrate = false;
    end
    if u_sat <= Umin + 1e-9 && e < 0
        integrate = false;
    end

    if integrate
        xI = xI + Ts*e;
    end

    % Back-calculation term to unwind (tracking time constant Tt)
    % Correct xI so that control output follows saturation smoothly.
    if Tt > 0 && abs(Ki) > 1e-8
        xI = xI - (Ts/Tt) * (du_sat - du_unsat) / Ki;
    end

    % ---- Plant update ----
    x = Ad*x + Bd*du_sat;

    % ---- Measurement with noise ----
    y_meas = (Cd*x + Dd*du_sat) + sqrt(Rv)*randn;

    % ---- Kalman update ----
    xhat = Ad*xhat + Bd*du_sat + Lk*(y_meas - (Cd*xhat + Dd*du_sat));
end

% Performance metrics
Tfinal = T0 + dT_ref;
amp = max(abs(Tfinal - Tabs(1)), 1e-6);
tol = max(settlePct*amp, settleMinAbs);

err = abs(Tabs - Tfinal);
idx_last = find(err > tol, 1, "last");
if isempty(idx_last)
    Ts_settle = 0;
else
    Ts_settle = t(idx_last);
end

OS = max(0, (max(Tabs) - Tfinal) / amp);   % overshoot fraction

sim.ok = (OS <= OSmax);   % only accept low-overshoot solutions
sim.Ts = Ts_settle;
sim.OS = OS;
sim.satFrac = satCount / N;
sim.t = t;
sim.Tabs = Tabs;
sim.uabs = u_hist;
end
