%% Identify "applied system" from PID log (NO offsets, NO divider)
%  - Builds a synthetic reference step if ref_v is constant in the log
%  - Identifies:
%     (A) Closed-loop T(s): ref_v -> sensor_v_avg (2 poles, 0 zeros)
%     (B) Plant-ish model Gp(s): u_v   -> sensor_v_avg (2 poles, 0 zeros) [optional]
%  - Compares measured vs identified vs your open-loop G(s)

%clear; clc; close all;

%% -------- File ----------
dataDir  = "./data_PID";
fileName = "data_PID1.csv";
tbl = readtable(fullfile(dataDir, fileName));

%% -------- Signals (RAW, no offsets, no divider) ----------
t = tbl.timestamp_ms/1000;  t = t - t(1);

r = tbl.ref_v;          % reference (V)
y = tbl.sensor_v_avg;   % output (V)

hasU = ismember("u_v", string(tbl.Properties.VariableNames));
if hasU
    u = tbl.u_v;        % controller output (V)
end

% Optional: drop saturated samples
useSatFilter = true;
if useSatFilter && ismember("sat", string(tbl.Properties.VariableNames))
    ok = (tbl.sat == 0);
else
    ok = true(height(tbl),1);
end

t = t(ok); r = r(ok); y = y(ok);
if hasU, u = u(ok); end

%% -------- Resample to uniform Ts (recommended) ----------
Ts = median(diff(t));
t_uni = (0:Ts:t(end)).';

r_uni = interp1(t, r, t_uni, "linear", "extrap");
y_uni = interp1(t, y, t_uni, "linear", "extrap");
if hasU
    u_uni = interp1(t, u, t_uni, "linear", "extrap");
end

%% -------- Open-loop model for comparison ----------
G = tf(0.4604, [1458.68 1]);

%% ============================================================
%  (A) Identify CLOSED-LOOP applied system: T(s) from ref -> y
%      If ref is constant, we create a synthetic step by prepending
%      a short pre-step segment of ref=0 and y=y(1).
% ============================================================

% Check if reference actually changes in the logged window
refSpan = max(r_uni) - min(r_uni);
REF_EPS = 1e-3;  % V

N_PRE = 25;  % prepend samples to create a visible step for the estimator

if refSpan < REF_EPS
    fprintf("ref_v is ~constant in the log (span %.6f V). Creating synthetic pre-step.\n", refSpan);

    % Assume the reference stepped from 0 -> r(1) at the start of logging
    r_pre = zeros(N_PRE,1);
    y_pre = y_uni(1)*ones(N_PRE,1);

    r_id_abs = [r_pre; r_uni];
    y_id_abs = [y_pre; y_uni];

    % Use deviation form for ID (helps numerics, not "offset correction")
    r0 = 0;                 % pre-step level
    y0 = y_uni(1);          % initial output level
    dr = r_id_abs - r0;
    dy = y_id_abs - y0;

    t_id = (0:Ts:Ts*(numel(dy)-1)).';
    dr_step = r_uni(1) - 0; % step amplitude (≈1.7)
else
    % If ref has a real step inside the log, detect it and align
    STEP_THRESH = 0.01;
    stepIdx = find(abs([0; diff(r_uni)]) > STEP_THRESH, 1, "first");
    if isempty(stepIdx), stepIdx = 1; end

    tpost = t_uni(stepIdx:end) - t_uni(stepIdx);
    rpost = r_uni(stepIdx:end);
    ypost = y_uni(stepIdx:end);

    if stepIdx > 1
        r0 = mean(r_uni(1:stepIdx-1));
        y0 = mean(y_uni(1:stepIdx-1));
    else
        r0 = r_uni(1);
        y0 = y_uni(1);
    end

    dr = rpost - r0;
    dy = ypost - y0;

    t_id = tpost;

    Nss = max(10, round(0.1*numel(rpost)));
    dr_step = mean(rpost(end-Nss+1:end)) - r0;
end

zT = iddata(dy, dr, Ts);  zT.TimeUnit = "s";

opt = tfestOptions;
opt.Focus = "simulation";
opt.InitialCondition = "estimate";
opt.Display = "off";

np = 2; nz = 0;
That = tfest(zT, np, nz, opt);
[~, fitT] = compare(zT, That);

fprintf("\n=== IDENTIFIED CLOSED-LOOP T(s): ref_v -> sensor_v_avg ===\n");
disp(That);
fprintf("Fit = %.2f %%\n", fitT);

% Simulate closed-loop model step (absolute output)
[dY_T, t_T] = step(dr_step * That, t_id);
yT_abs = y0 + dY_T;

% Compare open-loop step (same amplitude, just for visual comparison)
[dY_G, t_G] = step(dr_step * G, t_id);
yG_abs = y0 + dY_G;

%% ============================================================
%  (B) (Optional but recommended) Identify "plant-ish" model: u -> y
%      This often works better because u_v changes in your log.
% ============================================================

Gp = []; fitGp = NaN; yGp_abs = [];

if hasU
    % Use the whole resampled record (no need for ref step)
    u0 = u_uni(1);
    y0_u = y_uni(1);

    du = u_uni - u0;
    dy_u = y_uni - y0_u;

    zP = iddata(dy_u, du, Ts);  zP.TimeUnit = "s";

    Gp = tfest(zP, np, nz, opt);
    [~, fitGp] = compare(zP, Gp);

    fprintf("\n=== IDENTIFIED MODEL Gp(s): u_v -> sensor_v_avg ===\n");
    disp(Gp);
    fprintf("Fit = %.2f %%\n", fitGp);

    % Simulate response of Gp to the measured u(t)
    yGp_dev = lsim(Gp, du, t_uni);
    yGp_abs = y0_u + yGp_dev;
end

%% -------- Plots ----------
figure("Color","w","Name","Measured vs identified (closed-loop) vs open-loop");

plot(t_id, (y0 + dy), "k", "LineWidth", 1.3); hold on;
plot(t_T, yT_abs, "--", "LineWidth", 1.3);
plot(t_G, yG_abs, ":", "LineWidth", 1.6);
grid on;
xlabel("Time (s)");
ylabel("sensor\_v\_avg (V)");
title(sprintf("%s | step %.3f V | T(s) fit %.2f%%", fileName, dr_step, fitT));
legend("Measured y", "Identified closed-loop T(s)", "Open-loop G(s) step", "Location","best");

if hasU
    figure("Color","w","Name","Plant-ish identification using u_v -> y");
    plot(t_uni, y_uni, "k", "LineWidth", 1.3); hold on;
    plot(t_uni, yGp_abs, "--", "LineWidth", 1.3);
    grid on;
    xlabel("Time (s)");
    ylabel("sensor\_v\_avg (V)");
    title(sprintf("u_v -> y identification | Gp(s) fit %.2f%%", fitGp));
    legend("Measured y", "Identified Gp(s) response to measured u_v(t)", "Location","best");
end
