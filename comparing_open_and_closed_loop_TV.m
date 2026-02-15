%% Compare OPEN-loop vs CLOSED-loop responses in Temperature domain (T/V)
% Converted from comparing_open_and_closed_loop.m
%
% Key change:
%   Convert measured sensor voltages to Temperature using:
%     T = 36*(G_div*(V_adc + 0.15)) - 24.315
%
% Note (per your reminder):
%   The CLOSED-loop steady-state value should match the STEP final value (setpoint).
%   لذلك نرسم خطوة مرجعية في درجة الحرارة بحيث قيمتها النهائية = T(VREF).
%
% Files:
%   Open-loop:   ./data/data1.csv
%   Closed-loop: ./data_PID/data_PID1.csv

clear; clc; close all;

openDir  = "./data";
openFile = "data1.csv";

pidDir   = "./data_PID";
pidFile  = "data_PID1.csv";

VREF = 1.7;      % reference step FINAL VALUE (in the same voltage units as ref_v / sensor_v_avg)
STEP_THRESH = 0.01;

%% -------- Temperature conversion (your equation) ----------
% If your divider is 2/3 (1–5V -> ~0.67–3.33V), then to "undo" it use G_div = 3/2 = 1.5
G_div = 1.5;              % <-- change if your divider ratio differs
V_SENSOR_OFFSET = 0.15;
TEMP_GAIN       = 36;
TEMP_BIAS       = -24.315;

V2T = @(Vadc) TEMP_GAIN .* ( G_div .* (Vadc + V_SENSOR_OFFSET) ) + TEMP_BIAS;

TREF = V2T(VREF);         % final reference value in Temperature (°C)

%% -------- OPEN-LOOP dataset ----------
tbl1 = readtable(fullfile(openDir, openFile));

t1 = tbl1.timestamp_ms/1000;  t1 = t1 - t1(1);
y1V = tbl1.sensor_v;          % ADC-side sensor voltage
u1  = tbl1.setpoint_v;        % for alignment only (command/setpoint)

% Convert to Temperature
y1T = V2T(y1V);

% Resample to uniform time
Ts1 = median(diff(t1));
t1u = (0:Ts1:t1(end)).';
u1u = interp1(t1, u1,  t1u, "linear", "extrap");
y1Tu = interp1(t1, y1T, t1u, "linear", "extrap");

% Align open-loop to its step in setpoint_v
idx1 = find(abs([0; diff(u1u)]) > STEP_THRESH, 1, "first");
if isempty(idx1), idx1 = 1; end

t1a = t1u(idx1:end) - t1u(idx1);
y1a = y1Tu(idx1:end);

%% -------- CLOSED-LOOP dataset (I-P) ----------
tbl2 = readtable(fullfile(pidDir, pidFile));

t2 = tbl2.timestamp_ms/1000;  t2 = t2 - t2(1);
r2V = tbl2.ref_v;             % reference voltage
y2V = tbl2.sensor_v_avg;      % ADC-side sensor voltage (averaged)

% Convert to Temperature
r2T = V2T(r2V);
y2T = V2T(y2V);

hasU = ismember("u_v", string(tbl2.Properties.VariableNames));
if hasU, u2 = tbl2.u_v; end

% Optional: remove saturated samples (sat==0)
useSatFilter = true;
if useSatFilter && ismember("sat", string(tbl2.Properties.VariableNames))
    ok = (tbl2.sat == 0);
else
    ok = true(height(tbl2),1);
end
t2 = t2(ok); r2T = r2T(ok); y2T = y2T(ok);
if hasU, u2 = u2(ok); end

% Resample to uniform time
Ts2 = median(diff(t2));
t2u = (0:Ts2:t2(end)).';
r2u = interp1(t2, r2T, t2u, "linear", "extrap");
y2u = interp1(t2, y2T, t2u, "linear", "extrap");
if hasU
    u2u = interp1(t2, u2, t2u, "linear", "extrap");
end

% Align PID log:
% Prefer reference step if it exists, otherwise fall back to output change (or u_v change)
idx2 = find(abs([0; diff(r2u)]) > STEP_THRESH, 1, "first");
if isempty(idx2)
    dy = [0; diff(y2u)];
    thrY = max(5*mad(dy,1), 1e-3);
    idx2 = find(abs(dy) > thrY, 1, "first");
end
if isempty(idx2) && hasU
    du = [0; diff(u2u)];
    thrU = max(5*mad(du,1), 1e-3);
    idx2 = find(abs(du) > thrU, 1, "first");
end
if isempty(idx2), idx2 = 1; end

t2a = t2u(idx2:end) - t2u(idx2);
y2a = y2u(idx2:end);

%% -------- Plot both measured TEMPERATURE responses + reference step ----------
figure("Color","w","Name","Measured responses + Temperature reference step");

plot(t1a, y1a, "k", "LineWidth", 1.5); hold on;
plot(t2a, y2a, "--", "LineWidth", 1.5);

% Add a visible Temperature step from 0 -> TREF at t=0
tmax  = max([t1a(end), t2a(end)]);
tsref = min(Ts1, Ts2);
stairs([-tsref 0 tmax], [0 TREF TREF], "LineWidth", 1.5);

grid on;
xlabel("Time since event (s)");
ylabel("Temperature (°C)");
title(sprintf("Open-loop vs Closed-loop (T/V) with reference step final = %.3f °C", TREF));
legend( ...
    sprintf("Open-loop measured T | %s/%s", openDir, openFile), ...
    sprintf("Closed-loop measured T | %s/%s", pidDir, pidFile), ...
    sprintf("Reference step final = T(VREF=%.2f V) = %.3f °C", VREF, TREF), ...
    "Location","best");

% Optional: plot the CLOSED-loop reference (converted) on right axis for verification
yyaxis right
plot(t2u, r2u, "Color", [0.3 0.3 0.3], "LineWidth", 1.0);
ylabel("Reference Temperature (°C)");
yyaxis left
