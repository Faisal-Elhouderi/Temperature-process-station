%% PI Controller Validation on G1 and G3
% User-defined Kp and Ki

clear; clc; close all;

%% ================= USER INPUT =================
Kp = 3.9;        % <-- put your desired Kp here
Ki = 0.001250;      % <-- put your desired Ki here
% ===============================================

%% Identified plants
G1 = tf(0.648808, [1688.11 1]);    % Dataset 1
G3 = tf(0.460407,  [1508.15 1]);    % Dataset 3

%% PI Controller
C = pid(Kp, Ki);

%% Closed-loop systems
T1_cl = feedback(C * G1, 1);
T3_cl = feedback(C * G3, 1);

%% Open-loop step responses (plant only)
figure;
subplot(2,1,1)
step(G1, 8000)
grid on
title('Open-Loop Step Response — G1')
ylabel('\DeltaT (°C)')

subplot(2,1,2)
step(G3, 8000)
grid on
title('Open-Loop Step Response — G3')
xlabel('Time (s)')
ylabel('\DeltaT (°C)')

%% Closed-loop step responses (with PI)
figure;
subplot(2,1,1)
step(T1_cl, 8000)
grid on
title('Closed-Loop Step Response with PI — G1')
ylabel('\DeltaT (°C)')

subplot(2,1,2)
step(T3_cl, 8000)
grid on
title('Closed-Loop Step Response with PI — G3')
xlabel('Time (s)')
ylabel('\DeltaT (°C)')

%% Overlay comparison (before vs after)
figure;
subplot(2,1,1)
step(G1, 8000); hold on;
step(T1_cl, 8000);
grid on
title('G1: Open-Loop vs Closed-Loop')
legend('Open Loop', 'Closed Loop (PI)', 'Location', 'best')

subplot(2,1,2)
step(G3, 8000); hold on;
step(T3_cl, 8000);
grid on
title('G3: Open-Loop vs Closed-Loop')
legend('Open Loop', 'Closed Loop (PI)', 'Location', 'best')
xlabel('Time (s)')

%% Display controller gains
fprintf('\nPI Controller Parameters:\n');
fprintf('Kp = %.6f\n', Kp);
fprintf('Ki = %.6f\n', Ki);
