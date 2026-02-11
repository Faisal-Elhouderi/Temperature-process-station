%% Closed-loop ID from PID log + comparison with open-loop model
%clear; clc; close all;

% --------- File ----------
fname = "data_PID/data_PID1.csv";   % adjust if needed (relative/absolute path)

% --------- Known open-loop plant for comparison ----------
G_open = tf(0.4604, [1458.68 1]);   % V/V (or whatever your G is defined in)

% --------- Read CSV ----------
tbl = readtable(fname);

t = (tbl.timestamp_ms - tbl.timestamp_ms(1)) / 1000;   % seconds
Ts = median(diff(t));                                   % sample time (s)

r = tbl.ref_v;              % reference (input) in volts
y = tbl.sensor_v_avg;       % measured output in volts
u = tbl.u_v;                % controller output in volts
sat = tbl.sat;              % saturation flag (0/1)

fprintf("Loaded %d samples. Ts ≈ %.3f s\n", height(tbl), Ts);

% --------- Plot raw signals ----------
figure;
plot(t, r, 'LineWidth', 1.4); hold on;
plot(t, y, 'LineWidth', 1.4);
grid on;
xlabel('Time (s)');
ylabel('Voltage (V)');
title('Closed-loop response: reference vs measured output');
legend('ref\_v (R)', 'sensor\_v\_avg (Y)', 'Location', 'best');

figure;
yyaxis left;
plot(t, u, 'LineWidth', 1.4);
ylabel('Control output u\_v (V)');
yyaxis right;
stairs(t, sat, 'LineWidth', 1.2);
ylabel('sat flag');
grid on;
xlabel('Time (s)');
title('Control effort and saturation');

% --------- Find step start in reference (if it exists) ----------
STEP_THRESH = 0.01;
stepIdx = find(abs([0; diff(r)]) > STEP_THRESH, 1, "first");
if isempty(stepIdx)
    stepIdx = 1;
end

% If there is no noticeable change in r, ID from r->y is not possible
if stepIdx == 1 && (max(r) - min(r) < STEP_THRESH)
    warning("ref_v does not change (no step). You cannot identify T(s)=Y/R from this file unless r varies.");
    warning("If you want, you can instead identify plant-like dynamics using u_v -> sensor_v_avg.");
end

% --------- Build deviation data around the step (recommended) ----------
t2 = t(stepIdx:end) - t(stepIdx);
r2 = r(stepIdx:end);
y2 = y(stepIdx:end);
u2 = u(stepIdx:end);
sat2 = sat(stepIdx:end);

% Baselines (use a short window BEFORE the step if available)
if stepIdx > 5
    r0 = mean(r(1:stepIdx-1));
    y0 = mean(y(1:stepIdx-1));
else
    % if step is at start, estimate from first 2 seconds
    N0 = min(numel(r), max(5, round(2/Ts)));
    r0 = mean(r(1:N0));
    y0 = mean(y(1:N0));
end

dr = r2 - r0;
dy = y2 - y0;

% Remove saturated samples from identification (important)
idxGood = (sat2 == 0);
dr_id = dr(idxGood);
dy_id = dy(idxGood);

% Create iddata (closed-loop I/O: R -> Y)
z = iddata(dy_id, dr_id, Ts);
z.TimeUnit = "s";

% --------- Identify T(s) as 2 poles, 0 zeros ----------
np = 2; nz = 0;
opt = tfestOptions;
opt.Display = "off";
T_est = tfest(z, np, nz, opt);

disp("Identified closed-loop T_est(s) = Y/R (2nd-order, no zeros):");
disp(T_est);

% --------- Compare measured vs model (time-domain) ----------
% Simulate model output for the same input dr_id
t_id = (0:Ts:Ts*(numel(dr_id)-1)).';
yhat = lsim(T_est, dr_id, t_id);     % predicted deviation output

figure;
plot(t_id, dy_id, 'k', 'LineWidth', 1.4); hold on;
plot(t_id, yhat, '--', 'LineWidth', 1.4);
grid on;
xlabel('Time (s)');
ylabel('Output deviation (V)');
title('Measured vs identified closed-loop response (deviation signals)');
legend('Measured \Delta y', 'Model \Delta y (T\_est)', 'Location', 'best');

% --------- Step response comparison (open-loop vs closed-loop) ----------
t_end = max(8000, 8*1458.68);  % long horizon for slow thermal dynamics

figure;
step(G_open, t_end); hold on;
step(T_est, t_end);
grid on;
title('Step response comparison (unit step)');
xlabel('Time (s)');
ylabel('Output (V)');
legend('Open-loop G(s)', 'Closed-loop T\_est(s)=Y/R', 'Location', 'best');

% --------- Step metrics for the identified closed-loop T(s) ----------
info = stepinfo(T_est);
fprintf("\nClosed-loop identified T(s) step metrics (unit step):\n");
fprintf("RiseTime: %.2f s\n", info.RiseTime);
fprintf("SettlingTime (2%%): %.2f s\n", info.SettlingTime);
fprintf("Overshoot: %.2f %%\n", info.Overshoot);
fprintf("SteadyStateValue: %.4f\n", dcgain(T_est));
