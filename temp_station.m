%% Load and Plot Step Response Data
data = readtable('./data/data.csv');

time = data.timestamp_ms / 1000;  % Convert to seconds
voltage = (data.voltage + 0.15) .*(1.5);

% Normalize time to start from 0
time = time - time(1);

figure(1);
plot(time, voltage, 'b-', 'LineWidth', 1.5);
xlabel('Time (s)');
ylabel('Sensor Voltage (V)');
title('Temperature Step Response');
grid on;

% Extract key values from data
y_initial = voltage(1);           % Initial value


% DC Gain (needs setpoint temperature value)
setpoint_temp = input('Enter setpoint temperature (°C): ');



%% Alternative: Use System Identification Toolbox (if available)
% This gives more accurate results

% Create iddata object
Ts = mean(diff(time));  % Sampling time
step_input = setpoint_temp * ones(size(voltage));  % Step input
id_data = iddata(voltage - y_initial, step_input, Ts);

% Estimate 2nd order transfer function
sys_est = tfest(id_data, 3);  % 2nd order
disp('System ID Toolbox Result:');
sys_est

figure(2);
compare(id_data, sys_est);
title('System Identification Toolbox Comparison');