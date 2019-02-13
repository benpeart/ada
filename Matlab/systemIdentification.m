clear all; close all; clc;

set(0,'DefaultAxesXGrid','on')

dataPath = 'data/';

fieldNames = {'accAngle','filterAngle',...
    'setpoint1','input1','output1',...
    'setpoint2','input2','output2',...
    'setpoint3','input3','output3'};

%% Find all .csv files
files = dir([dataPath, '*.csv']);
fileNames = {files.name}.';

%% Read .csv file
file = fileNames{1};
dRaw = csvread([dataPath, file], 1, 0);

t = dRaw(:,1) - dRaw(1,1);
d = cell2struct(num2cell(dRaw(:,2:end),1), fieldNames,2);

dT = mean(diff(t));

%% Plot some data
figure(1)
clf

subplot(221)
plot(t, d.accAngle)
hold on
plot(t, d.filterAngle)
plot(t, d.setpoint1, 'g--')
hold off

subplot(222)
plot(t, d.output1)

%% Calculate some FFTs
X = fft(d.filterAngle);
Y = fft(d.output1);

l = floor(length(X)/2)*2;
X = X(1:l/2+1);
Y = Y(1:l/2+1);
f = (1/dT)*(1:l/2+1)/l;

figure(2)
clf
subplot(211)
loglog(f, abs(X))
hold on
loglog(f, abs(Y))
hold off

subplot(212)
semilogx(f, angle(X)*180/pi)
hold on
semilogx(f, angle(Y)*180/pi)
hold off

xlabel('Frequency [Hz]')

%% Run FRF
[txy, f] = tfestimate(d.output1,d.filterAngle);

figure(2)
clf
subplot(211)
loglog(abs(txy))