% Try to make a fall detection algorithm, based on collected data
% Idea: look at error signal of angle controller. 
% Two ideas: threshold for error signal, count how long over threshold
% Or, integrate error signal
% Or, integrate absolute of error signal

clear all; close all; clc;

set(0,'DefaultAxesXGrid','on','DefaultAxesYGrid','on')

dataPath = 'data/';

fieldNames = {'accAngle','filterAngle',...
    'setpoint1','input1','output1',...
    'setpoint2','input2','output2',...
    'setpoint3','input3','output3'};

%% Find all .csv files
files = dir([dataPath, '*.csv']);
fileNames = {files.name}.';

%% Read .csv file
file = fileNames{9}; fileDescription = 'Driving and falling, no step loss';
file = fileNames{10}; fileDescription = 'Driving and forcing step loss';
file = fileNames{11}; fileDescription = 'Standing without interaction';
dRaw = csvread([dataPath, file], 1, 0);

t = dRaw(:,1) - dRaw(1,1);
d = cell2struct(num2cell(dRaw(:,2:end),1), fieldNames,2);

dT = mean(diff(t));
Fs = 1/dT;

errorIntThreshold = 30;

% Plot some time domain data
setpoint = d.setpoint1;
input = d.input1;
error = setpoint - input;

errorInt = cumtrapz(t, error);

% Detect where robot re-starts
controllerActive = ~d.output1==0;
controllerRestart = diff(controllerActive) == 1;

% Value of integrator at restart
% First value is 0, as index array has zero values
errorIntRestartValue = [0; errorInt(controllerRestart)];
% Offset 1, because indexing starts at 1
errorIntRestartIndex = cumsum(controllerRestart) + 1; 
% Add a single 0, as array has become 1 element too short
errorIntSubtractValue = [0; errorIntRestartValue(errorIntRestartIndex)];
errorIntCorrected = errorInt - errorIntSubtractValue;
errorIntCorrected(~controllerActive) = nan;

fallDetected = abs(errorIntCorrected)>errorIntThreshold;

figure(2)
clf

ax = [];
ax(end+1) = subplot(221);
plot(t, [setpoint input], 'LineWidth', 1)
xlabel('Time [s]')
ylabel('Angle [^o]')
legend('Setpoint', 'Input')


ax(end+1) = subplot(223);
plot(t,error)
xlabel('Time [s]')
ylabel('Angle [^o]')
legend('Error')

ax(end+1) = subplot(222);
% plot(t, [d.output1 d.output2 d.output3])
plot(t, controllerActive)
ylim([-0.1 1.1])
legend('Controller active')

ax(end+1) = subplot(224);
plot(t, errorIntCorrected)
hold on
yMax = max(errorIntCorrected);
plot(t, fallDetected*yMax, 'g')
xLim = get(gca, 'XLim');
plot(xLim, [1 1]*errorIntThreshold, 'k--')
plot(xLim, -[1 1]*errorIntThreshold, 'k--')
hold off
xlabel('Time [s]')
ylabel('Angle*s [^o]')
legend('Error integral', 'Fall detected', 'Threshold')

linkaxes(ax, 'x')



