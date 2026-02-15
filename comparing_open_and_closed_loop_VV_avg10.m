%% Plot two REAL measured responses + add 1.7V reference step
clear; clc; close all;

openDir  = "./data";
openFile = "data1.csv";

pidDir   = "./data_PID";
pidFile  = "data_PID1.csv";

VREF = 1.7;      % <-- requested reference step magnitude
STEP_THRESH = 0.01;
N_AVG_OL = 10;   % open-loop: average every 10 samples -> 1 sample

%% -------- OPEN-LOOP dataset ----------
tbl1 = readtable(fullfile(openDir, openFile));

t1 = tbl1.timestamp_ms/1000;  t1 = t1 - t1(1);
y1 = tbl1.sensor_v;           % RAW
u1 = tbl1.setpoint_v;         % for alignment only

Ts1 = median(diff(t1));
t1u = (0:Ts1:t1(end)).';
u1u = interp1(t1, u1, t1u, "linear", "extrap");
y1u = interp1(t1, y1, t1u, "linear", "extrap");

% --- Open-loop filter: block-average every N_AVG_OL samples -> 1 sample ---
N1 = numel(t1u);
M1 = floor(N1 / N_AVG_OL);
if M1 < 5
    error("Open-loop data is too short for Avg(%d).", N_AVG_OL);
end
idxKeep = 1:(M1 * N_AVG_OL);

t_mat = reshape(t1u(idxKeep), N_AVG_OL, M1);
u_mat = reshape(u1u(idxKeep), N_AVG_OL, M1);
y_mat = reshape(y1u(idxKeep), N_AVG_OL, M1);

t1b = mean(t_mat, 1).';
u1b = mean(u_mat, 1).';
y1b = mean(y_mat, 1).';

% Align to step using the averaged open-loop command
idx1 = find(abs([0; diff(u1b)]) > STEP_THRESH, 1, "first");
if isempty(idx1), idx1 = 1; end

t1a = t1b(idx1:end) - t1b(idx1);
y1a = y1b(idx1:end);

%% -------- PID (closed-loop) dataset ----------
tbl2 = readtable(fullfile(pidDir, pidFile));

t2 = tbl2.timestamp_ms/1000;  t2 = t2 - t2(1);
r2 = tbl2.ref_v;
y2 = tbl2.sensor_v_avg;       % RAW

hasU = ismember("u_v", string(tbl2.Properties.VariableNames));
if hasU, u2 = tbl2.u_v; end

useSatFilter = true;
if useSatFilter && ismember("sat", string(tbl2.Properties.VariableNames))
    ok = (tbl2.sat == 0);
else
    ok = true(height(tbl2),1);
end
t2 = t2(ok); r2 = r2(ok); y2 = y2(ok);
if hasU, u2 = u2(ok); end

Ts2 = median(diff(t2));
t2u = (0:Ts2:t2(end)).';
r2u = interp1(t2, r2, t2u, "linear", "extrap");
y2u = interp1(t2, y2, t2u, "linear", "extrap");
if hasU
    u2u = interp1(t2, u2, t2u, "linear", "extrap");
end

% Align PID log:
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

%% -------- Plot both REAL responses + 1.7V step reference ----------
figure("Color","w","Name","Measured responses + 1.7V reference");

plot(t1a, y1a, "k", "LineWidth", 1.5); hold on;
plot(t2a, y2a, "--", "LineWidth", 1.5);

% Add a visible step from 0 -> 1.7 at t=0
tmax  = max([t1a(end), t2a(end)]);
tsref = min(Ts1*N_AVG_OL, Ts2);
stairs([-tsref 0 tmax], [0 VREF VREF], "LineWidth", 1.5);

grid on;
xlabel("Time since event (s)");
ylabel("Measured output voltage (V)");
title(sprintf("Real measured responses (open-loop Avg %d) with %.2fV reference step", N_AVG_OL, VREF));
legend( ...
    sprintf("Open-loop measured y (sensor\\_v) | %s/%s", openDir, openFile), ...
    sprintf("Closed-loop measured y (sensor\\_v\\_avg) | %s/%s", pidDir, pidFile), ...
    "Reference step = 1.7 V", ...
    "Location","best");
