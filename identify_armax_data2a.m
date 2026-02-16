%% identify_armax_data2a.m
% Identifies a DISCRETE-TIME ARMAX model from ONE open-loop step test:
%   data/data2_a.csv  (timestamp_ms,setpoint_v,sensor_v)
%
% IMPORTANT (per your clarification):
%   - setpoint_v is ALREADY the direct station input in the range 1..5 V
%     -> NO INPUT SCALING is applied.
%   - sensor_v is typically the ADC voltage after divider (0.67..3.3 V)
%     -> it is converted back to station output volts (1..5 V) if needed.
%
% Fixes the “step happens too early” warning by automatically PREPENDING
% enough equilibrium samples (zeros in deviation variables) before the step.
%
% Outputs:
%   bestSys   (idpoly ARMAX)
%   A,B,C,nk,Ts,u0,y0
%   plots: data, compare(), overlay, residuals
%
% Requires: System Identification Toolbox

clear; clc; close all;

%% -------------------- FILE --------------------
filePath = fullfile("data","data2_a.csv");
assert(isfile(filePath), "File not found: %s", filePath);

%% -------------------- OUTPUT SCALING (edit if needed) -----------------
% Station output y_st in [1,5] -> ADC y_adc in [0.67,3.30]
ky = (3.30 - 0.67)/4;
by = 0.67 - ky*1.0;                      % y_adc = by + ky*y_st
% invert: y_st = (y_adc - by)/ky

%% -------------------- STEP DETECTION + BASELINE -----------------------
STEP_THRESH_V = 0.01;   % step detect threshold on setpoint_v
BASE_WIN      = 20;     % baseline mean window (samples, if available)

%% -------------------- ARMAX SEARCH RANGES (typical thermal) ----------
na_list = 1:3;
nb_list = 1:3;
nc_list = 1:3;
nk_list = 1:25;         % delay in samples (scan)

% Minimum number of equilibrium samples BEFORE the step to avoid warnings.
N_PRE_MIN = max([na_list nb_list nc_list]) + max(nk_list) + 5;

%% -------------------- OPTIONS --------------------
opt = armaxOptions;
opt.Focus   = "simulation";
opt.Display = "off";

copt = compareOptions;
copt.InitialCondition = "estimate";

%% -------------------- LOAD --------------------
tbl = readtable(filePath);
req = ["timestamp_ms","setpoint_v","sensor_v"];
for n = req
    assert(any(strcmp(tbl.Properties.VariableNames, n)), ...
        "Missing column '%s' in %s", n, filePath);
end

t = tbl.timestamp_ms/1000;
sp_raw = tbl.setpoint_v;   % station input (1..5 V) per your note
y_raw  = tbl.sensor_v;     % ADC volts (0.67..3.3) OR station volts (1..5)

% Sort by time and drop duplicate timestamps (keep last)
[t, idx] = sort(t);
sp_raw = sp_raw(idx);
y_raw  = y_raw(idx);
[t, ia] = unique(t, "stable");
sp_raw = sp_raw(ia);
y_raw  = y_raw(ia);

t = t - t(1);

% Estimate Ts, retime if needed
dt = diff(t);
Ts = median(dt);
fprintf("Estimated Ts = %.4f s\n", Ts);

if any(abs(dt - Ts) > 0.05*Ts)
    fprintf("Non-uniform sampling detected -> retiming to uniform grid.\n");
    tt = timetable(seconds(t), sp_raw, y_raw, 'VariableNames', {'sp','y'});
    tUniform = (0:Ts:t(end))';
    tt2 = retime(tt, seconds(tUniform), "linear");
    t = tUniform;
    sp_raw = tt2.sp;
    y_raw  = tt2.y;
end

%% -------------------- INPUT/OUTPUT IN STATION VOLTS -------------------
% INPUT: already station volts (1..5) -> no scaling
u_st = sp_raw;
fprintf("Input: setpoint_v treated as DIRECT station input (expected 1..5 V). No scaling.\n");
if any(u_st < 0.5) || any(u_st > 5.5)
    warning("setpoint_v has values outside typical station range 1..5 V. Please verify your file.");
end

% OUTPUT: if sensor_v is ADC-side (<=3.3) convert to station volts
if max(y_raw) <= 3.35
    y_st = (y_raw - by)/ky;          % ADC -> station output
    fprintf("Output: sensor_v treated as ADC (0.67..3.3V) -> station output (1..5V).\n");
else
    y_st = y_raw;
    fprintf("Output: sensor_v treated as station output (1..5V). No scaling.\n");
end

%% -------------------- FIND STEP INDEX --------------------
du_in = [0; diff(u_st)];
stepIdx = find(abs(du_in) > STEP_THRESH_V, 1, "first");
if isempty(stepIdx)
    error("No step detected. Lower STEP_THRESH_V or check your file.");
end
fprintf("Detected step at sample %d (t = %.3f s)\n", stepIdx, t(stepIdx));

%% -------------------- BUILD DEVIATION DATA WITH PREPENDING ------------
preAvail = stepIdx - 1;
baseN = min([BASE_WIN, preAvail, length(u_st)]);
if baseN < 3
    baseN = min(5, length(u_st));
end

u0 = mean(u_st(1:baseN));
y0 = mean(y_st(1:baseN));

du = u_st - u0;
dy = y_st - y0;

% Ensure at least N_PRE_MIN equilibrium samples before the step:
needPad = max(0, N_PRE_MIN - (stepIdx-1));
if needPad > 0
    fprintf("Prepending %d equilibrium samples to satisfy transient-data requirement.\n", needPad);
    du = [zeros(needPad,1); du];
    dy = [zeros(needPad,1); dy];

    % Also pad absolute signals for plotting
    u_st = [u0*ones(needPad,1); u_st];
    y_st = [y0*ones(needPad,1); y_st];

    t = (0:Ts:Ts*(length(du)-1))';
    stepIdx = stepIdx + needPad;
else
    t = (0:Ts:Ts*(length(du)-1))';
end

% Sanity checks
assert(std(du) > 1e-8, "Input deviation is ~zero; identification not possible.");
assert(length(du) == length(dy), "Input/output length mismatch.");

z = iddata(dy, du, Ts);
z.TimeUnit   = "s";
z.InputName  = "du_st";
z.OutputName = "dy_st";

%% -------------------- QUICK DATA PLOTS --------------------
figure('Name','Data used for ARMAX (absolute station volts)');
subplot(2,1,1);
plot(t, u_st, 'LineWidth', 1.1); grid on;
xline(t(stepIdx),'--');
ylabel('u_{st} (V)'); title('Station input (absolute)');

subplot(2,1,2);
plot(t, y_st, 'LineWidth', 1.1); grid on;
xline(t(stepIdx),'--');
ylabel('y_{st} (V)'); xlabel('Time (s)');
title('Station output (absolute)');

%% -------------------- SEARCH ARMAX --------------------
bestScore = -Inf;
bestSys   = [];
bestInfo  = struct("na",NaN,"nb",NaN,"nc",NaN,"nk",NaN);
bestFitPct = NaN;

fprintf("\nSearching ARMAX orders...\n");
for na = na_list
for nb = nb_list
for nc = nc_list
for nk = nk_list
    try
        sys = armax(z, [na nb nc nk], opt);

        % Simulation fit
        [~, fit] = compare(z, sys, copt);
        if isempty(fit) || any(~isfinite(fit)), continue; end
        fitPct = mean(fit);

        % Mild complexity penalty
        score = fitPct - 0.2*(na+nb+nc);

        if score > bestScore
            bestScore  = score;
            bestFitPct = fitPct;
            bestSys    = sys;
            bestInfo   = struct("na",na,"nb",nb,"nc",nc,"nk",nk);
        end
    catch
        % ignore infeasible candidates
    end
end
end
end
end

if isempty(bestSys)
    error("All ARMAX candidates failed. Expand ranges or check your data.");
end

fprintf("\nBest simulation fit = %.2f %%\n", bestFitPct);
fprintf("Best orders: na=%d, nb=%d, nc=%d, nk=%d\n", ...
    bestInfo.na, bestInfo.nb, bestInfo.nc, bestInfo.nk);

% Avoid polydata(); read coefficients directly
A  = bestSys.A;
B  = bestSys.B;
C  = bestSys.C;
nk = bestSys.nk;

fprintf("\nA(q^-1) = "); disp(A);
fprintf("B(q^-1) = "); disp(B);
fprintf("C(q^-1) = "); disp(C);
fprintf("nk = %d samples (%.3f s)\n", nk, nk*Ts);

%% -------------------- VERIFY: compare() + overlay --------------------
figure('Name','compare() - ARMAX simulation vs measured');
compare(z, bestSys, copt);
grid on;
title(sprintf("ARMAX compare() | na=%d nb=%d nc=%d nk=%d | fit=%.2f%%", ...
    bestInfo.na,bestInfo.nb,bestInfo.nc,bestInfo.nk,bestFitPct));

% Manual overlay in ABSOLUTE volts
y_sim_iddata = sim(bestSys, z, copt);             % returns dy_sim
dy_sim = y_sim_iddata.OutputData;
y_abs_sim = dy_sim + y0;

figure('Name','Step response overlay (absolute station volts)');
plot(t, y_st, 'LineWidth', 1.2); hold on;
plot(t, y_abs_sim, '--', 'LineWidth', 1.2);
xline(t(stepIdx),'--');
grid on;
xlabel('Time (s)'); ylabel('y_{st} (V)');
legend('Measured y_{st}','ARMAX simulated y_{st}','Location','best');
title('Measured step response vs ARMAX model (simulation)');

figure('Name','Residual analysis');
resid(z, bestSys);
grid on;

%% -------------------- SAVE RESULTS --------------------
save("best_armax_data2a.mat", "bestSys","A","B","C","nk","Ts","u0","y0","bestInfo","bestFitPct");
fprintf("\nSaved: best_armax_data2a.mat\n");
