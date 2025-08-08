clear; clc;

PID = "(1.6,0.9,0.7)";
file_name = sprintf('./Pressure_Data/PID_Control_' + PID + '_1.mat');
record1 = load(file_name);

figure(3);
plot(record1.t_record, record1.pressure_record);
hold on;
grid on;
ylabel('Absolute Pressure (Pa)');
title('LPS22HHTR Sensor Data');


figure(4);
subplot(2,1,1);
hold on;
time = record1.t_record;
start_time = time(find(~isnan(time), 1));
time = time - start_time;
plot(time, record1.setPWM_record, 'r');
plot(time, record1.measuredPWM_record, 'b');
yline(127, '--r', 'PWM = 127', 'LabelVerticalAlignment', 'top');
yline(127 + 127*0.05, '--', 'Color', [0.5 0.5 0.5]);
yline(127 - 127*0.05, '--', '5% Bound', 'LabelVerticalAlignment', 'bottom', 'Color', [0.5 0.5 0.5]);
xline(15.8, '--', 'Color', [0.5 0.5 0.5]);
text(16, 50, 't = 15.8s', ...
    'HorizontalAlignment', 'left', ...
    'Rotation', 0, ...
    'Color', [0.5 0.5 0.5]);

grid on;
hold off;
xlabel('Time (s)'); ylabel('Actuator PWM (0-255)');
% title_name = sprintf('PID Control ' + PID +' on Vibration Amplitude');
title_name = 'Proposed PID Control (1.6, 0.9, 0.7) on Vibration Amplitude';
title(title_name);
legend('Set PWM', 'Measured PWM');


%% Pressure Data Processing

clear; clc;
batch = 35; index = 1;

pressure_filename = sprintf('./Pressure_Data/Batch%d_LPS_%d.mat', batch, index);
pressure_data = load(pressure_filename);
pressure = pressure_data.pressure_record;
time = pressure_data.t_record;
figure(4);
plot(time, pressure);
xlabel('Time (s)'); ylabel('Absolute Pressure (Pa)');
title('Example LPS22HHTR Pressure Data');
grid on;

pressure_filename = sprintf('./Pressure_Data/Batch%d_LPS_%d_cropped.mat', batch, index);
pressure_data = load(pressure_filename);
pressure = pressure_data.pressure_record;
time = pressure_data.t_record;
pressure = pressure(~isnan(pressure) & ~isinf(pressure));   % Remove NaN and Inf, shift valid values left
time = time(~isnan(time) & ~isinf(time));
deltaP = diff(pressure);  % p(i+1) - p(i) for all i
deltaP = abs([0, deltaP]);

figure(5);
plot(time, deltaP);
hold on; grid on;

amplitude = [];
for i = 1:length(deltaP)
    if i <= 50
        window = deltaP(1:i);
    else
        window = deltaP(i-50:i);
    end
    
    window = sort(window, 'descend');
    if length(window) < 10
        top10 = window;
    else
        top10 = window(1:10);
    end
    amplitude = [amplitude, mean(top10)];
end

plot(time, amplitude, 'r');


fs = 200;                   % Sampling frequency
fc = 1;                     % Cutoff frequency in Hz (adjust as needed)
Wn = fc / (fs/2);           % Normalized cutoff frequency

[b, a] = butter(2, Wn, 'low')  % 2nd-order low-pass filter
smooth_amplitude = filtfilt(b, a, amplitude);
plot(time, smooth_amplitude, 'b');
yline(mean(smooth_amplitude), 'g');
xlabel('Time (s)'); ylabel('Change in Pressure (Pa)');
title('Example of Change in Pressure Data Processing');
legend('Change in Pressure', 'Sliding Window Average Amplitude', 'Low-Pass Filter Amplitude', 'Filtered Average Amplitude');
hold off;



%% Force Data

batch = 50; index = 1;
force_filename = sprintf('./Force_Data/Batch%d_LPS_Force_%d.log', batch, index);
force_data = importdata(force_filename);
force_load = force_data.data(:,2);
travel = force_data.data(:,3);
force_time = force_data.data(:,4);
pressure_filename = sprintf('./Pressure_Data/Batch%d_LPS_%d.mat', batch, index);
pressure_data = load(pressure_filename);
pressure = pressure_data.pressure_record;
temperature = pressure_data.temperature_record;
pressure_time = pressure_data.t_record;
start_time = pressure_time(find(~isnan(pressure_time), 1));
pressure_time = pressure_time - start_time;

delta_F = compute_average_change(force_load);
delta_P = compute_average_change(pressure);

figure(6);
subplot(2,1,1);
plot(force_time, force_load);
% hold on;
% yline(delta_F, '--r', sprintf('F = %.4f N', delta_F), 'LabelVerticalAlignment', 'bottom');
% yline(0, '--b', 'Baseline', 'LabelVerticalAlignment', 'bottom');
% hold off;
grid on;
ylabel('Load (N)');
title('Applied Force and Measured Pressure with Travel Distance');
subplot(2,1,2);
plot(pressure_time, pressure);
% hold on;
% yline(101345 + delta_P, '--r', sprintf('p = %.0f Pa', 101345 + delta_P), 'LabelVerticalAlignment', 'bottom');
% yline(101345, '--b', 'Baseline', 'LabelVerticalAlignment', 'bottom');
% hold off;
grid on;
xlabel('Time (s)');
ylabel('Absolute Pressure (Pa)');


%% Pressure Sensor Characterisation

clear; clc;

function mean_change = compute_average_change(data)
    
    % Apply a moving average to smooth out noise
    data_smooth = movmean(data, 5);
    % Use findpeaks to get local maxima
    peak_to_peak = max(data_smooth) - min(data_smooth);
    prominence_thresh = 0.2 * peak_to_peak;  % 20% of signal range
    [peaks, locs] = findpeaks(data_smooth, 'MinPeakProminence', prominence_thresh, 'MinPeakDistance', 200);
    % Estimate baseline as the local minimum before each peak
    deltas = zeros(size(peaks));
    for i = 1:length(peaks)
        if i == 1
            search_start = 1;
        else
            search_start = locs(i-1);
        end
        search_end = locs(i);
        baseline = prctile(data(search_start:search_end), 20);  % use low percentile (20%)
        deltas(i) = peaks(i) - baseline;
    end
    mean_change = mean(deltas);
    % disp(['Average Change: ', num2str(mean(deltas))]);
end

rows = 41:50;
cols = 1:3;
[R, C] = meshgrid(rows, cols);
datasets_index = [R(:), C(:)];
buffer = [];

for dataset = datasets_index'
    batch = dataset(1);
    index = dataset(2);
    if batch == 41 && index == 1, continue; end
    force_filename = sprintf('./Force_Data/Batch%d_LPS_Force_%d.log', batch, index);
    force_data = importdata(force_filename);
    force_load = force_data.data(:,2);
    pressure_filename = sprintf('./Pressure_Data/Batch%d_LPS_%d.mat', batch, index);
    pressure_data = load(pressure_filename);
    pressure = pressure_data.pressure_record;
    % Calculate average change
    delta_F = compute_average_change(force_load);
    delta_P = compute_average_change(pressure);
    buffer = [buffer; delta_F, delta_P];
end

figure(7);
scatter(buffer(:,1), buffer(:,2), 'marker', '+');
grid on;
xlabel('Applied Force (N)'); ylabel('Average Change in Pressure (Pa)');
title('Pressure Sensor Characterisation');

% Best fit curve
best_fit = fit(buffer(:,1), buffer(:,2), 'poly1');
coeffs = coeffvalues(best_fit);
p1 = coeffs(1);
p2 = coeffs(2);
fit_label = sprintf('y = %.3f*x %.3f\n', p1, p2);  
fprintf(fit_label);

figure(8);
plot(best_fit, buffer(:,1), buffer(:,2));
xlabel('Applied Force (N)'); ylabel('Average Change in Pressure (Pa)');
title('Pressure Sensor Characterisation');
x_pos = 0.0488;  % example x-position
y_pos = best_fit(x_pos);  % corresponding y on the line
text(x_pos, y_pos-500, fit_label, 'FontSize', 10, 'Color', 'red');
grid on;

% x: Applied Force to air chamber, y: Change in Pressure
% y = 18445.223*x - 275.698

% Theoretical Lines
hold on;
r = 0.002;
surface_area = 2*pi*r^2 + pi*r^2;
m = 1 / surface_area;
x = 0:0.01:0.12;
y = m * x;
% plot(x, y, 'g-');
% fit_label = sprintf('y = %.3f*x\n', m); 
% text(x_pos-0.01, m*x_pos + 500, fit_label, 'FontSize', 10, 'Color', 'green');
hold off;

legend('Measured Data', 'Best-Fit Line', 'Location', 'northwest');

%% Actuator Characterisation

clear; clc;

start_batch = 60; end_batch = 70;
batch = start_batch;
buffer = [];
while batch <= end_batch
    index = 1;
    while index <= 3
        pressure_filename = sprintf('./Pressure_Data/Batch%d_LPS_%d.mat', batch, index);
        index = index + 1;
        if ~isfile(pressure_filename), continue; end
        pressure_data = load(pressure_filename);
        pressure = pressure_data.pressure_record;
        pressure = pressure(~isnan(pressure) & ~isinf(pressure));   % Remove NaN and Inf, shift valid values left
        deltaP = diff(pressure);  % p(i+1) - p(i) for all i
        deltaP = abs([0, deltaP]);

        % Sliding window average & Low-pass filter
        smoothed_amplitude = [];
        window_size = 100;
        for i = 1:length(deltaP)
            window_start = max(1, i - window_size + 1);
            window = deltaP(window_start:i);
            window = sort(window, 'descend');
            top10 = window(1:min(10, length(window)));
            smoothed_amplitude(end+1) = mean(top10);
        end
        b = [0.0000212, 0.00004239, 0.0000212];
        a = [1.0, -1.9869, 0.987];
        smoothed_amplitude = filtfilt(b, a, smoothed_amplitude);
        mean_amplitude = mean(smoothed_amplitude);
        std_amplitude = std(smoothed_amplitude, 0, 2);
        buffer = [buffer; 10*((batch-start_batch)*10)/100, mean_amplitude, std_amplitude];
    end
    batch = batch + 1;
end

figure(9);
% errorbar(buffer(:,1), buffer(:,2), buffer(:,3), 'x', 'LineStyle', 'none');
scatter(buffer(:,1), buffer(:,2), 'marker', '+');
grid on;
xlabel('Applied Peak-to-Peak Voltage (V)'); ylabel('Change in Pressure (Pa)');
title('Change in Pressure Distribution');

figure(10);
best_fit_deltaP = fit(buffer(:,1), buffer(:,2), 'poly2');
plot(best_fit_deltaP, buffer(:,1), buffer(:,2));
xlabel('Applied Peak-to-Peak Voltage (V)'); ylabel('Change in Pressure (Pa)');
title('Best-fit Curve of Change in Pressure Distribution');
legend('Measured Data', 'DeltaP-Voltage Best-Fit Curve', 'Location', 'northwest');
grid on;

% Best fit curve
coeffs = coeffvalues(best_fit_deltaP);
if length(coeffs) == 3
    p1 = coeffs(1);
    p2 = coeffs(2);
    p3 = coeffs(3);
    fit_label = sprintf('y = %.6f*x^2 + %.6f*x + %.6f\n', p1, p2, p3);
else
    p1 = coeffs(1);
    p2 = coeffs(2);
    fit_label = sprintf('y = %.6f*x + %.6f\n', p1, p2);
end
fprintf('DeltaP-Voltage: ');
fprintf(fit_label);
text(2, 20, fit_label, 'FontSize', 10, 'Color', 'red');


figure(11);
calculated_force = (buffer(:,2) + 275.698) / 18445.223;
best_fit_force = fit(buffer(:,1), calculated_force, 'poly2');
plot(best_fit_force, buffer(:,1), calculated_force);
xlabel('Applied Peak-to-Peak Voltage (V)'); ylabel('Actuator Force (N)');
title('Actuator Characterisation');
legend('Measured Data', 'Force-Voltage Best-Fit Curve', 'Location', 'northwest');
grid on;

% Best fit curve
coeffs = coeffvalues(best_fit_force);
if length(coeffs) == 3
    p1 = coeffs(1);
    p2 = coeffs(2);
    p3 = coeffs(3);
    fit_label = sprintf('y = %.6f*x^2 + %.6f*x + %.6f\n', p1, p2, p3);
else
    p1 = coeffs(1);
    p2 = coeffs(2);
    fit_label = sprintf('y = %.6f*x + %.6f\n', p1, p2);
end
fprintf('Force-Voltage: ');
fprintf(fit_label);
text(2, 0.016, fit_label, 'FontSize', 10, 'Color', 'red');

% DeltaP-Voltage: y = -0.249281*x^2 + 10.043143*x + 9.408743
% Force-Voltage: y = -0.000014*x^2 + 0.000544*x + 0.015457


%% Pressure Data Cropping

batch = 40; index = 3;
start_time = 188; end_time = 217;

pressure_filename = sprintf('./Pressure_Data/Batch%d_LPS_%d.mat', batch, index);
data = load(pressure_filename);
t_record = data.t_record;
pressure_record = data.pressure_record;
temperature_record = data.temperature_record;
setPWM_record = data.setPWM_record;

valid_idx = ~isnan(t_record) & ~isnan(pressure_record) &  ~isnan(temperature_record) & ~isnan(setPWM_record);
t_record = t_record(valid_idx);
pressure_record = pressure_record(valid_idx);
temperature_record = temperature_record(valid_idx);
setPWM_record = setPWM_record(valid_idx);

% Remove time range between 0 and 20 seconds
keep_idx = (t_record >= start_time) & (t_record <= end_time);
t_record = t_record(keep_idx);
pressure_record = pressure_record(keep_idx);
temperature_record = temperature_record(keep_idx);
setPWM_record = setPWM_record(keep_idx);

figure(999);
plot(data.t_record, data.pressure_record);
hold on;
plot(t_record, pressure_record, 'r');  % cropped version
legend('Original', 'Cropped');
hold off;
% Save to new file or overwrite
new_filename = sprintf('./Pressure_Data/Batch%d_LPS_%d_cropped.mat', batch, index);
save(new_filename, 't_record', 'pressure_record', 'temperature_record', 'setPWM_record');