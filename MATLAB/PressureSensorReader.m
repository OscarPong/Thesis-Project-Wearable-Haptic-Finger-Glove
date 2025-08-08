clear; clc;

global connection_mode c_actuator;

% Mode 0: USBC, Mode 1: BLE
connection_mode = 1;

%%%%%%%%%% Serial Port, BLE, UDP SETUP %%%%%%%%%%

if connection_mode == 0
    port = 'COM3';
    baudRate = 9600;
    s = serialport(port, baudRate);
    configureTerminator(s, "LF");   % Set terminator to match Arduino's Serial.println
    flush(s);                       % Clear the serial buffer
end

% BLE Setup for ESP32
% BLE_DEVICE_ADDRESS = 'dc:06:75:32:e3:26';     % LPS Prototype 1 (Rigid)
% BLE_DEVICE_ADDRESS = 'dc:06:75:32:4e:1e';     % LPS Prototype 2 (Flexible)
% BLE_DEVICE_ADDRESS = 'dc:06:75:35:0a:12';     % ICP Prototype 2 (Flexible)
BLE_DEVICE_ADDRESS = '28:37:2f:76:e9:36';       % LPS Prototype 3 (Final)
SERVICE_UUID = '12345678-1234-1234-1234-123456789abc';
S_CHARACTERISTIC_UUID = '87654321-4321-4321-4321-abcdefabcdef';
A_CHARACTERISTIC_UUID = '87654321-4321-4321-4321-abcdefabcd01';

% Connect to BLE device
device = ble(BLE_DEVICE_ADDRESS);
% Access the characteristics
c_sensor = characteristic(device, SERVICE_UUID, S_CHARACTERISTIC_UUID);
c_actuator = characteristic(device, SERVICE_UUID, A_CHARACTERISTIC_UUID);

% UDP Port
u = udpport("datagram", "LocalPort", 5022);
% Set callback to trigger when 1 datagram is available
configureCallback(u, "datagram", 1, @udpCallback);

%%%%%%%%%% Dynamic Plot Setup %%%%%%%%%%

duration = 60;                  % Duration in seconds
sampling_rate = 10 / 1000;       % Sampling rate (from Arduino Serial print delay = 10ms)
global N;
N = duration / sampling_rate;   % Number of samples to show
xData = nan(1, N);
y1Data = nan(1, N);
y2Data = nan(1, N);
figure(1);

% --- First subplot: Pressure ---
ax1 = subplot(2,1,1);
set(ax1, 'Position', [0.13, 0.6, 0.78, 0.35]);  % [left, bottom, width, height]
hPlot1 = plot(xData, y1Data, 'b');
ylabel('Absolute Pressure (Pa)');
title('Real-Time Sensor Data');
grid on;
xlim([0, duration]);

% --- Second subplot: Temperature ---
ax2 = subplot(2,1,2);  % 2 rows, 1 column, 2nd plot
set(ax2, 'Position', [0.13, 0.15, 0.78, 0.35]);  % Adjust vertical spacing
hPlot2 = plot(xData, y2Data, 'r');
xlabel('Time (s)');
ylabel('Temperature (°C)');
grid on;
xlim([0, duration]);

hButtons = CreateControlsInFigure(1);

% --- Control subplots ---
controlXData = nan(1, N);
controlY1Data = nan(1, N);
controlY2Data = nan(1, N);
setPressure = nan(1, N);
setForce = nan(1, N);
figure(2);

controlAx1 = subplot(2,1,1);
controlHPlot1 = plot(controlXData, controlY1Data, 'b');
hold(controlAx1, 'on');
hPressurePlot = plot(controlXData, setPressure, 'r');
hold(controlAx1, 'off');
ylabel('Change in Pressure (Pa)');
title('Pressure and Force Control');
grid on;
xlim([0, duration]);

controlAx2 = subplot(2,1,2);
controlHPlot2 = plot(controlXData, controlY2Data, 'b');
hold(controlAx2, 'on');
hForcePlot = plot(controlXData, setForce, 'r');
hold(controlAx2, 'off');
xlabel('Time (s)');
ylabel('Actuator PWM (0-255)');
grid on;
xlim([0, duration]);

%%%%%%%%%% Main Loop %%%%%%%%%%


global FFF t_record pressure_record temperature_record setPWM_record measuredPWM_record save_index setPWM;
FFF = zeros(4, 1);  % FFF(1): program start; FFF(2): clear graph; (3): record data; (4): record start time once
t_record = [];
pressure_record = [];
temperature_record = [];
setPWM_record = [];
measuredPWM_record = [];
setPWM = 0;
save_index = 1;
index = 1;
t0 = 0;
previous_pressure = 0;
serialBuffer = '';

while ishandle(hPlot1) && ishandle(hPlot2)
    
    % Flag to end program
    if FFF(1), break; end

    % Read data based on connection mode
    if connection_mode == 0
        % Check if serial monitor is avaliable
        if s.NumBytesAvailable <= 0, continue; end
        % Read all available characters as char
        newChars = char(read(s, s.NumBytesAvailable, 'char'));
        serialBuffer = [serialBuffer, newChars];

        % Look for newline characters
        lfIdx = strfind(serialBuffer, newline);
        while ~isempty(lfIdx)
            % Extract the first full line
            line = serialBuffer(1:lfIdx(1)-1);
            serialBuffer = serialBuffer(lfIdx(1)+1:end);  % Remove processed part
            % Parse and process
            values = strsplit(strtrim(line), ';');
            % Update lfIdx (in case there are more newlines in buffer)
            lfIdx = strfind(serialBuffer, newline);
        end

    elseif connection_mode == 1
        % Read BLE data
        data = read(c_sensor);  % read as uint8
        strData = char(data);   % convert to char array
        values = strsplit(strtrim(strData), ';');
    end
    
    if length(values) <= 2, continue; end

    t = str2double(values{1});
    pressure = str2double(values{2});
    temperature = str2double(values{3});
    if pressure <= 10, continue; end    % Reject abnormal data <= 10 Pa
    if temperature <= 0, continue; end    % Reject abnormal data <= 0 deg

    % Record start time for displaying data
    if FFF(4) == 0
        t0 = t;
        FFF(4) = 1;
    end
    
    % Add to display plot
    graph_t = t - t0;
    xData(index) = graph_t;
    y1Data(index) = pressure;
    y2Data(index) = temperature;
    % Add to control plot
    if index > 1
        controlXData(index) = graph_t;
        deltaP = abs(pressure - previous_pressure);
        controlY1Data(index) = deltaP; 
    end
    % Calculate average deltaP in a window in real time
    window_size = 50;
    window_start = max(1, index - window_size);
    window = controlY1Data(window_start:index);
    window = sort(window, 'descend');
    top10 = window(1:min(10, length(window)));
    setPressure(index) = mean(top10);
    % Compare the calculated Vpp to set Vpp
    controlY2Data(index) = str2double(values{4});
    setForce(index) = 255*(setPWM / 100);
    
    previous_pressure = pressure;
    index = index + 1;

    % Record data
    if FFF(3)
        t_record(end+1) = t;
        pressure_record(end+1) = pressure;
        temperature_record(end+1) = temperature;
        setPWM_record(end+1) = 255*(setPWM / 100);
        measuredPWM_record(end+1) = str2double(values{4});
    end

    set(hPlot1, 'XData', xData, 'YData', y1Data);
    set(hPlot2, 'XData', xData, 'YData', y2Data);
    set(controlHPlot1, 'XData', controlXData, 'YData', controlY1Data);
    set(controlHPlot2, 'XData', controlXData, 'YData', controlY2Data);
    set(hPressurePlot, 'XData', controlXData, 'YData', setPressure);
    set(hForcePlot, 'XData', controlXData, 'YData', setForce);

    % Reset plot if t exceed display duration or FFF(2) is True
    if graph_t > duration || FFF(2)
        FFF(2) = 0;
        FFF(4) = 0;
        xData = nan(1, N);
        y1Data = nan(1, N);
        y2Data = nan(1, N);
        controlXData = nan(1, N);
        controlY1Data = nan(1, N);
        controlY2Data = nan(1, N);
        setPressure = nan(1, N);
        setForce = nan(1, N);
        index = 1;
        set(hPlot1, 'XData', nan(1, N), 'YData', nan(1, N));
        set(hPlot2, 'XData', nan(1, N), 'YData', nan(1, N));
        set(controlHPlot1, 'XData', nan(1, N), 'YData', nan(1, N));
        set(controlHPlot2, 'XData', nan(1, N), 'YData', nan(1, N));
    end

    drawnow;
end

disp("END of Program");
if connection_mode == 0, clear s; end
delete(hButtons);



function MyCallback(src, event, x)
    global FFF t_record pressure_record temperature_record setPWM_record measuredPWM_record N save_index c_actuator setPWM;
    switch(x)
        case 1
            % End program
            FFF(1) = 1; return; 
        case 2
            % Clear graph
            if FFF(3)
                disp("Cannot clear graph because it is recording data!");
            else
                FFF(2) = 1;
                disp("Clear Graph Successfully!");
            end
            return;
        case 3
            FFF(2) = 1; % Clear graph
            if FFF(3) == 0
                FFF(3) = 1;
                disp("Recording data...");
            else
                filename = sprintf("./Pressure_Data/temp_%d.mat", save_index);
                save(filename, "t_record", "pressure_record", "temperature_record", "setPWM_record", "measuredPWM_record");
                t_record = nan(1, N);
                pressure_record = nan(1, N);
                temperature_record = nan(1, N);
                setPWM_record = nan(1, N);
                measuredPWM_record = nan(1, N);
                save_index = save_index + 1;
                fprintf("Successfully saved data into %s\n", filename);
                FFF(3) = 0;
            end
            return;
        case 4
            % Set PWM percentage of actuator
            popupObj = src;  % popup menu handle
            selected_index = popupObj.Value;
            selected_string = popupObj.String{selected_index};
            disp(['Actuator PWM percentage is set to: ', selected_string]);
            setPWM = str2double(erase(selected_string, '%'));
            % Send PWM percentage to ESP32
            write(c_actuator, setPWM, "uint8");
            return;
    end
end


function hh = CreateControlsInFigure(figu)
    figure(figu);
    left = 350; bottom = 7; width = 100; height = 20; dx = width*1.1;
    
    hh(1)= CreateMyButton('END', [left, bottom, width, height], {@MyCallback,1}); left = left - dx;
    hh(2)= CreateMyButton('Clear Graph', [left, bottom, width, height], {@MyCallback,2}); left = left - dx;
    hh(3)= CreateMyButton('Record Data', [left, bottom, width, height], {@MyCallback,3}); left = left - dx;
    popupOptions = {'0%', '10%', '20%', '30%', '40%', '50%', '60%', '70%', '80%', '90%', '100%'};
    hh(4) = uicontrol('Style', 'popupmenu', 'String', popupOptions, 'Position', [left, bottom, width, height], 'Callback', {@MyCallback, 4});

    set(hh(1),'BackgroundColor',[1,0.2,0]); % Red
    set(hh(2),'BackgroundColor',[1,1,0]);   % Yellow
    set(hh(3),'BackgroundColor',[0,1,0]);   % Green
    set(hh(4),'BackgroundColor',[0.5,0.8,0.9]);   % Light Blue
end


function h = CreateMyButton(strBla, position, CallbackFunction)
    h = uicontrol('Style', 'pushbutton', 'String', strBla, 'Position', position, 'Callback', CallbackFunction); 
end


function udpCallback(src, ~)
    global c_actuator setPWM;

    data = read(src, src.NumDatagramsAvailable);

    for i = 1:numel(data)
        % Extract raw bytes from the Datagram object
        rawBytes = data(i).Data;  % This is uint8 array

        % Convert bytes to char array (assuming ASCII/UTF-8 text)
        d = char(rawBytes');  % transpose because Data is column vector
        d = strtrim(d); % remove whitespace

        try
            setPWM = str2double(d);
            write(c_actuator, uint8(setPWM), "uint8");
            disp("Sent BLE PWM: " + d);
        catch ME
            disp("Invalid UDP input: " + d);
            disp("Error: " + ME.message);
        end
    end
end