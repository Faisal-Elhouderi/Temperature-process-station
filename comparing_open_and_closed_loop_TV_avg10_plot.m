%% Open-loop vs Closed-loop comparison in Temperature domain (T/V) + Open-loop Avg(10)
% What you asked for:
%   1) Remove the extra highlighted "data1" entry and any unwanted extra lines.
%      -> This script creates ONLY 3 plotted objects (open-loop avg10, closed-loop, reference step)
%         and the legend is built from explicit line handles (no auto-added legend items).
%   2) Filter the OPEN-LOOP data (data/data1.csv) by averaging every 10 samples and PLOT that filtered curve.
%
% Clarification (your question "what is data1 in the figure?"):
%   "data1" refers to the OPEN-LOOP file name: ./data/data1.csv.
%   In your screenshot, the extra legend item "data1" was NOT supposed to appear;
%   it came from an additional plotted object being picked up by MATLAB's legend auto-update.
%   This script prevents that.

%clear; clc; close all;

%% ----------------- Files -----------------
openDir  = "./data";
openFile = "data1.csv";          % open-loop file you mentioned

pidDir   = "./data_PID";
pidFile  = "data_PID1.csv";      % closed-loop file you mentioned

STEP_THRESH = 0.01;              % step detection threshold in volts
REF_EPS     = 1e-3;              % treat ref as constant if span < REF_EPS

% Open-loop filtering: block-average every 10 samples -> 1 sample
N_AVG_OL = 10;

%% -------- Temperature conversion (your equation) ----------
% If your divider is 2/3 (1–5V -> ~0.67–3.33V), then to undo it use G_div = 3/2 = 1.5
G_div = 1.5;              % <-- change if your divider ratio differs
V_SENSOR_OFFSET = 0.15;
TEMP_GAIN       = 36;
TEMP_BIAS       = -24.315;

V2T = @(Vadc) TEMP_GAIN .* ( G_div .* (Vadc + V_SENSOR_OFFSET) ) + TEMP_BIAS;

%% =========================================================
%  CLOSED-LOOP dataset (I-P): used for plotting and to get the reference final value
% =========================================================
tblCL = readtable(fullfile(pidDir, pidFile));

t_cl = tblCL.timestamp_ms/1000;  t_cl = t_cl - t_cl(1);
r_clV = tblCL.ref_v;             % reference in volts (station/reference units)
y_clV = tblCL.sensor_v_avg;      % ADC-side sensor voltage

% Convert measured output to Temperature
y_clT = V2T(y_clV);

% Optional: remove saturated samples (sat==0)
useSatFilter = true;
if useSatFilter && ismember("sat", string(tblCL.Properties.VariableNames))
    ok = (tblCL.sat == 0);
else
    ok = true(height(tblCL),1);
end

t_cl = t_cl(ok);
r_clV = r_clV(ok);
y_clT = y_clT(ok);

% Resample closed-loop to uniform time (for clean plotting)
Ts_cl = median(diff(t_cl));
t_clu = (0:Ts_cl:t_cl(end)).';
r_clu = interp1(t_cl, r_clV, t_clu, "linear", "extrap");
y_clu = interp1(t_cl, y_clT, t_clu, "linear", "extrap");

% Align closed-loop to its reference step if present; otherwise assume step at start
refSpanCL = max(r_clu) - min(r_clu);
if refSpanCL < REF_EPS
    idxCL = 1;
else
    idxCL = find(abs([0; diff(r_clu)]) > STEP_THRESH, 1, "first");
    if isempty(idxCL), idxCL = 1; end
end

t_cla = t_clu(idxCL:end) - t_clu(idxCL);
y_cla = y_clu(idxCL:end);

% Reference final value (steady state) equals step final value (your note).
% We estimate it from the last 10% of ref samples.
NssCL = max(10, round(0.1*numel(r_clu)));
VREF  = mean(r_clu(end-NssCL+1:end));      % final reference in volts
TREF  = V2T(VREF);                          % final reference in Temperature (°C)

%% =========================================================
%  OPEN-LOOP dataset: filter by Avg(10) and plot the filtered curve
% =========================================================
tblOL = readtable(fullfile(openDir, openFile));

t_ol = tblOL.timestamp_ms/1000;  t_ol = t_ol - t_ol(1);
u_ol = tblOL.setpoint_v;         % command / setpoint voltage (for alignment)
y_olV = tblOL.sensor_v;          % ADC-side sensor voltage

y_olT = V2T(y_olV);

% Resample open-loop to uniform time first
Ts_ol = median(diff(t_ol));
t_olu = (0:Ts_ol:t_ol(end)).';
u_olu = interp1(t_ol, u_ol,  t_olu, "linear", "extrap");
y_olu = interp1(t_ol, y_olT, t_olu, "linear", "extrap");

% Block-average every N_AVG_OL samples -> 1 sample
N = numel(t_olu);
M = floor(N / N_AVG_OL);
if M < 5
    error("Open-loop data is too short for Avg(%d).", N_AVG_OL);
end
idx = 1:(M * N_AVG_OL);

t_blk = reshape(t_olu(idx), N_AVG_OL, M);
u_blk = reshape(u_olu(idx), N_AVG_OL, M);
y_blk = reshape(y_olu(idx), N_AVG_OL, M);

t_olb = mean(t_blk, 1).';
u_olb = mean(u_blk, 1).';
y_olb = mean(y_blk, 1).';

% Align open-loop to its step in setpoint
stepIdxOL = find(abs([0; diff(u_olb)]) > STEP_THRESH, 1, "first");
if isempty(stepIdxOL), stepIdxOL = 1; end

t_ola = t_olb(stepIdxOL:end) - t_olb(stepIdxOL);
y_ola = y_olb(stepIdxOL:end);

%% =========================
%   PLOT (ONLY 3 CURVES)
% =========================
figure("Color","w","Name","Open-loop Avg(10) vs Closed-loop (T/V) + Reference step");

% 1) Open-loop filtered (Avg 10)
hOL = plot(t_ola, y_ola, "k", "LineWidth", 1.5); hold on;

% 2) Closed-loop measured
hCL = plot(t_cla, y_cla, "r--", "LineWidth", 1.6);

% 3) Reference Temperature step: 0 -> TREF at t=0
tmax = max([t_ola(end), t_cla(end)]);
tsref = min([median(diff(t_olb)), Ts_cl]);   % for a visible vertical edge
hRef = stairs([-tsref 0 tmax], [0 TREF TREF], "LineWidth", 1.6);

grid on;
xlabel("Time since event (s)");
ylabel("Temperature (°C)");
title(sprintf("Open-loop Avg(%d) vs Closed-loop (T/V) | Reference final = T(VREF=%.3fV)=%.3f°C", ...
    N_AVG_OL, VREF, TREF));

% Legend built from explicit handles => no extra items like "data1"
lgd = legend([hOL, hCL, hRef], ...
    sprintf("Open-loop measured T (Avg %d) | %s/%s", N_AVG_OL, openDir, openFile), ...
    sprintf("Closed-loop measured T | %s/%s", pidDir, pidFile), ...
    sprintf("Reference step final = T(VREF)=%.3f °C", TREF), ...
    "Location","best");
set(lgd, "AutoUpdate", "off");
