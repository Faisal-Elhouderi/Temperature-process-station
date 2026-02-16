%% tune_and_simulate_mvc_data2a_v2.m
% FIXED MVC/GMV DESIGN + SIMULATION (causality bug fixed)
%
% Your previous result (u stuck at 5V and y acting open-loop) happened because
% the simulation used y(k) before it was computed, so the term -F*y had NO effect.
% This v2 script computes y(k) FIRST (from past inputs), then computes u(k)
% using the AVAILABLE measurement y(k). That restores true closed-loop feedback.
%
% Loads: best_armax_data2a.mat
% Saves: mvc_data2a.mat (controller coefficients + chosen tuning)
%
% Requires: System Identification Toolbox

clear; clc; close all;

%% -------------------- Load identified model ---------------------------
matPath = "best_armax_data2a.mat";
assert(isfile(matPath), "Missing %s. Run identification first.", matPath);
S = load(matPath);

if isfield(S,"bestSys") && ~isempty(S.bestSys)
    sys  = S.bestSys;
    Ts   = sys.Ts;
    A    = sys.A(:).';
    Braw = sys.B(:).';
    C    = sys.C(:).';
    nk_saved = [];
    if isprop(sys,"nk") && ~isempty(sys.nk), nk_saved = sys.nk; end
else
    Ts = S.Ts;
    A  = S.A(:).';
    Braw = S.B(:).';
    C  = S.C(:).';
    nk_saved = [];
    if isfield(S,"nk"), nk_saved = S.nk; end
end

fprintf("Loaded model. Ts=%.4f s\n", Ts);

%% -------------------- Normalize delay & B (remove leading zeros) -------
epsB = 1e-12 * max(1, max(abs(Braw)));
idx = find(abs(Braw) > epsB, 1, "first");
assert(~isempty(idx), "B polynomial appears to be all zeros.");

d_from_B = idx - 1;
Bbar = Braw(idx:end);
while numel(Bbar) > 1 && abs(Bbar(end)) <= epsB
    Bbar(end) = [];
end

if ~isempty(nk_saved)
    d = max(nk_saved, d_from_B);
else
    d = d_from_B;
end

fprintf("Delay d = %d samples (%.3f s)\n", d, d*Ts);
fprintf("A(q^-1)   = "); disp(A);
fprintf("Bbar(q^-1)= "); disp(Bbar);
fprintf("C(q^-1)   = "); disp(C);

%% -------------------- MVC polynomials (Diophantine) -------------------
% Solve: C = A*E + q^{-d}F
[E,F] = diophantine_mv(A, C, d);
Q = conv(E, Bbar);

E = E(:).'; F = F(:).'; Q = Q(:).';
assert(abs(Q(1)) > 0, "Q(1)=0; cannot solve for u(k).");

fprintf("\nE(q^-1) = "); disp(E);
fprintf("F(q^-1) = "); disp(F);
fprintf("Q(q^-1) = "); disp(Q);

%% -------------------- USER: operating point + reference ---------------
% Station actuator limits:
u_min = 1.0;  u_max = 5.0;

% True pre-step station input (ABSOLUTE volts, 1..5)
u_init = 1.4;                 % <-- EDIT (you said true u is 1.4V)

% True pre-step station output (ABSOLUTE volts, 1..5)
% If unknown, use saved y0 from identification (but better to set your real initial output)
y_init = iff(isfield(S,"y0"), S.y0, 1.262);   % <-- EDIT if you know it

% Desired output (absolute station output volts)
y_ref_abs = y_init + 0.30;    % <-- EDIT desired step size
t_step_ref = 10;              % seconds when reference changes

% Simulation horizon
Tsim = 3600;                  % seconds (thermal is slow)

% Reference mode:
%  - "preview": uses r(k+d) (you know the command ahead of time)
%  - "causal" : uses r(k-d) (strictly causal)
ref_mode = "preview";         % recommended

% Optional slew-rate limit on actuator (V/sample). Set Inf to disable.
du_max_per_sample = Inf;      % e.g., 0.02

% How much saturation allowed during tuning (fraction of samples)
satFrac_max = 0.10;           % 10%

%% -------------------- Tuning grids ------------------------------------
% tau_ref: smaller => faster but more aggressive
tau_grid = [2 5 10 20 30 60 120 180 300];

% lambda_u: bigger => gentler control
lambda_grid = [0, 1e-10, 1e-9, 1e-8, 1e-7, 1e-6, 1e-5, 1e-4, 1e-3];

%% -------------------- Build reference (absolute & deviation) ----------
N = round(Tsim/Ts) + 1;
t = (0:N-1)' * Ts;

r_abs = y_init*ones(N,1);
r_abs(t >= t_step_ref) = y_ref_abs;

r_dev = r_abs - y_init; % deviation reference

%% -------------------- Grid search (fastest settling with constraints) -
results = [];
best = struct("tau",NaN,"lambda",NaN,"settle",Inf,"rise",Inf,"overshoot",Inf, ...
              "IAE",Inf,"satFrac",Inf,"uMin",NaN,"uMax",NaN);

fprintf("\nGrid search (SIM FIXED, mode=%s)...\n", ref_mode);

for tau_ref = tau_grid
    % Prefilter reference (1st order)
    alpha = Ts/(tau_ref + Ts);
    r_f = zeros(N,1);
    for k = 2:N
        r_f(k) = r_f(k-1) + alpha*(r_dev(k) - r_f(k-1));
    end

    r_ctrl = make_r_ctrl(r_f, d, ref_mode);

    for lambda_u = lambda_grid
        opts = struct("lambda_u",lambda_u, "u_min",u_min, "u_max",u_max, ...
                      "du_max_per_sample",du_max_per_sample);

        [u_abs, y_abs] = sim_mvc_fixed(A, Bbar, d, Q, C, F, Ts, u_init, y_init, r_ctrl, opts);

        satFrac = mean((u_abs <= u_min+1e-9) | (u_abs >= u_max-1e-9));
        uMin = min(u_abs); uMax = max(u_abs);

        e = y_abs - r_abs;
        IAE = sum(abs(e))*Ts;

        [riseT, settleT, overshoot] = step_metrics(t, y_abs, r_abs, t_step_ref);

        results = [results; tau_ref, lambda_u, riseT, settleT, overshoot, IAE, satFrac, uMin, uMax]; %#ok<AGROW>

        if satFrac <= satFrac_max
            if settleT < best.settle
                best.tau = tau_ref;
                best.lambda = lambda_u;
                best.settle = settleT;
                best.rise = riseT;
                best.overshoot = overshoot;
                best.IAE = IAE;
                best.satFrac = satFrac;
                best.uMin = uMin;
                best.uMax = uMax;
            end
        end
    end
end

resultsTbl = array2table(results, 'VariableNames', ...
    {'tau_ref_s','lambda_u','rise_s','settle_s','overshoot_pct','IAE','satFrac','uMin','uMax'});
resultsTbl = sortrows(resultsTbl, 'settle_s');
disp(resultsTbl(1:min(12,height(resultsTbl)),:));

if isfinite(best.settle)
    fprintf("\nBEST (sat<=%.1f%%): tau_ref=%.1f s, lambda=%g\n", satFrac_max*100, best.tau, best.lambda);
    fprintf("  rise=%.1f s, settle=%.1f s, overshoot=%.1f%%, satFrac=%.2f%%, u=[%.3f..%.3f]\n", ...
        best.rise, best.settle, best.overshoot, best.satFrac*100, best.uMin, best.uMax);
else
    warning("No candidate met satFrac<=%.1f%%. Consider allowing more saturation or adding slew limits.", satFrac_max*100);
    best.tau = resultsTbl.tau_ref_s(1);
    best.lambda = resultsTbl.lambda_u(1);
end

%% -------------------- Simulate best and plot --------------------------
tau_ref = best.tau;
lambda_u = best.lambda;

alpha = Ts/(tau_ref + Ts);
r_f = zeros(N,1);
for k = 2:N
    r_f(k) = r_f(k-1) + alpha*(r_dev(k) - r_f(k-1));
end
r_ctrl = make_r_ctrl(r_f, d, ref_mode);

opts = struct("lambda_u",lambda_u, "u_min",u_min, "u_max",u_max, ...
              "du_max_per_sample",du_max_per_sample);
[u_abs, y_abs, u_dev, y_dev] = sim_mvc_fixed(A, Bbar, d, Q, C, F, Ts, u_init, y_init, r_ctrl, opts);

figure('Name','MVC/GMV output (fixed sim)');
plot(t, y_abs, 'LineWidth', 1.2); hold on;
plot(t, r_abs, '--', 'LineWidth', 1.2);
grid on; xlabel('Time (s)'); ylabel('y_{st} (V)');
legend('y','r','Location','best');
title(sprintf('MVC/GMV (%s): tau_ref=%.1fs, lambda=%g, d=%d', ref_mode, tau_ref, lambda_u, d));

figure('Name','MVC/GMV input (fixed sim)');
plot(t, u_abs, 'LineWidth', 1.2); grid on;
yline(u_min,'--'); yline(u_max,'--');
xlabel('Time (s)'); ylabel('u_{st} (V)');
title('Control input (saturated)');

%% -------------------- Save controller data ----------------------------
mvc = struct();
mvc.Ts = Ts;
mvc.d  = d;
mvc.A  = A;
mvc.B  = Bbar;
mvc.C  = C;
mvc.E  = E;
mvc.F  = F;
mvc.Q  = Q;
mvc.ref_mode = ref_mode;
mvc.tau_ref  = tau_ref;
mvc.lambda_u = lambda_u;
mvc.u_min = u_min;
mvc.u_max = u_max;
mvc.u_init_used = u_init;
mvc.y_init_used = y_init;
mvc.du_max_per_sample = du_max_per_sample;

save("mvc_data2a.mat","mvc");
fprintf("\nSaved: mvc_data2a.mat\n");

fprintf("\nIf u still sticks at 5V, it means your reference step is too large/fast\n");
fprintf("for your constraints, OR the identified gain/delay is not accurate.\n");

%% ===================== Local functions ===============================

function out = iff(cond, a, b)
if cond, out = a; else, out = b; end
end

function r_ctrl = make_r_ctrl(r_f, d, mode)
N = length(r_f);
switch string(mode)
    case "preview"
        r_ctrl = zeros(N,1);
        for k = 1:N
            idx = k + d;
            if idx > N, idx = N; end
            r_ctrl(k) = r_f(idx);
        end
    case "causal"
        r_ctrl = [zeros(d,1); r_f(1:end-d)];
    otherwise
        error("Unknown ref_mode. Use 'preview' or 'causal'.");
end
end

function [E,F] = diophantine_mv(A,C,d)
% Solve: C = conv(A,E) + q^{-d} F, with deg(E)=d-1
A = A(:).'; C = C(:).';
na = length(A)-1;
nc = length(C)-1;

degE = max(d-1, 0);
degTarget = max(nc, na+degE);
degF = max(degTarget - d, 0);

lenE = degE+1;
lenF = degF+1;
L = degTarget+1;

M = zeros(L, lenE+lenF);

for j = 0:degE
    col = zeros(1,L);
    idx = (j+1):(j+length(A));
    idx = idx(idx <= L);
    col(idx) = A(1:numel(idx));
    M(:, j+1) = col(:);
end

for j = 0:degF
    row = d + j + 1;
    if row <= L
        M(row, lenE + j + 1) = 1;
    end
end

Cp = zeros(L,1);
Cp(1:length(C)) = C(:);

x = M \ Cp;
E = x(1:lenE).';
F = x(lenE+1:end).';
end

function [u_abs, y_abs, u_dev, y_dev] = sim_mvc_fixed(A, B, d, Q, C, F, Ts, u_init, y_init, r_ctrl, opts)
% FIXED causality:
%   At each sample k:
%     1) compute y(k) from past u (plant, delayed)
%     2) compute u(k) using AVAILABLE y(k)
%
% Plant: A y = q^{-d} B u
% Controller: Q u = C r_ctrl - F y
%
% Signals are deviation around (u_init, y_init).

N = length(r_ctrl);

A=A(:).'; B=B(:).'; Q=Q(:).'; C=C(:).'; F=F(:).';
na = length(A)-1;
nb = length(B)-1;
nq = length(Q)-1;
nc = length(C)-1;
nf = length(F)-1;

u_dev = zeros(N,1);
y_dev = zeros(N,1);

u_abs = u_init*ones(N,1);
y_abs = y_init*ones(N,1);

q0 = Q(1);
lambda_u = opts.lambda_u;
u_min = opts.u_min;
u_max = opts.u_max;
du_max = opts.du_max_per_sample;

for k = 1:N
    % ---- 1) Plant output y_dev(k) from known past u_dev ----
    yk = 0;

    for i = 1:na
        if (k-i) >= 1
            yk = yk - A(i+1)*y_dev(k-i);
        end
    end

    for j = 0:nb
        idx_u = k - d - j;
        if idx_u >= 1
            yk = yk + B(j+1)*u_dev(idx_u);
        end
    end

    y_dev(k) = yk;
    y_abs(k) = y_init + yk;

    % ---- 2) Controller uses current y_dev(k) (measured) ----
    rhs = 0;

    for i = 0:nc
        if (k-i) >= 1
            rhs = rhs + C(i+1)*r_ctrl(k-i);
        end
    end

    for i = 0:nf
        if (k-i) >= 1
            rhs = rhs - F(i+1)*y_dev(k-i);   % includes i=0 -> y_dev(k) is NOW valid
        end
    end

    for i = 1:nq
        if (k-i) >= 1
            rhs = rhs - Q(i+1)*u_dev(k-i);
        end
    end

    if lambda_u > 0
        u_k = (rhs*q0)/(q0*q0 + lambda_u);
    else
        u_k = rhs/q0;
    end

    u_cmd = u_init + u_k;

    if isfinite(du_max) && k > 1
        u_cmd = min(u_cmd, u_abs(k-1) + du_max);
        u_cmd = max(u_cmd, u_abs(k-1) - du_max);
    end

    u_cmd = min(max(u_cmd, u_min), u_max);

    u_abs(k) = u_cmd;
    u_dev(k) = u_cmd - u_init;
end
end

function [riseT, settleT, overshoot] = step_metrics(t, y, r, t0)
idx0 = find(t >= t0, 1, "first");
if isempty(idx0), idx0 = 1; end

y_seg = y(idx0:end);
t_seg = t(idx0:end);

y0 = y(idx0);
r_final = r(end);
step = r_final - y0;
if abs(step) < 1e-9
    riseT = Inf; settleT = Inf; overshoot = 0; return;
end

y10 = y0 + 0.1*step;
y90 = y0 + 0.9*step;

i10 = find((step>0 & y_seg >= y10) | (step<0 & y_seg <= y10), 1, "first");
i90 = find((step>0 & y_seg >= y90) | (step<0 & y_seg <= y90), 1, "first");
if isempty(i10) || isempty(i90)
    riseT = Inf;
else
    riseT = t_seg(i90) - t_seg(i10);
end

if step > 0
    y_peak = max(y_seg);
    overshoot = max(0, (y_peak - r_final)/abs(step))*100;
else
    y_min = min(y_seg);
    overshoot = max(0, (r_final - y_min)/abs(step))*100;
end

band = 0.02*abs(step);
err = abs(y_seg - r_final);
inside = err <= band;

settleT = Inf;
for k = 1:length(inside)
    if inside(k) && all(inside(k:end))
        settleT = t_seg(k) - t_seg(1);
        break;
    end
end
end
