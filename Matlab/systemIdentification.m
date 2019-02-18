clear all; close all; clc;

set(0,'DefaultAxesXGrid','on')

dataPath = 'data/';

fieldNames = {'accAngle','filterAngle',...
    'setpoint1','input1','output1',...
    'setpoint2','input2','output2',...
    'setpoint3','input3','output3','dist'};

%% Find all .csv files
files = dir([dataPath, '*.csv']);
fileNames = {files.name}.';

%% Read .csv file
file = fileNames{7};
dRaw = csvread([dataPath, file], 1, 0);

t = dRaw(:,1) - dRaw(1,1);
d = cell2struct(num2cell(dRaw(:,2:end),1), fieldNames,2);

dT = mean(diff(t));
Fs = 1/dT;

%% Plot some time domain data
y = d.filterAngle;
u = d.output1;
e = d.setpoint1-d.input1;
ds = d.dist;

figure(1)
clf

subplot(221)
plot(t, u)

subplot(222)
plot(t,ds)


%% Try to do system ID
[S, f2] = tfestimate(ds, u, [],[],[],Fs);
PS = tfestimate(ds, y);
H = PS./S;
C = ((1./S)-1)./H;

Cs = mscohere(ds, u);
Cps = mscohere(ds, y);

figure(5)
clf
subplot(311)
loglog(f2, abs([S PS]))
legend('S', 'PS')
ylabel('Magnitude')

subplot(312)
semilogx(f2, angle([S PS])*180/pi)
ylabel('Phase [deg]')

subplot(313)
semilogx(f2, [Cs Cps])
ylabel('Coherence [-]')
xlabel('Frequency [Hz]')

figure(6)
subplot(211)
loglog(f2, abs([H C H.*C]))
legend('H_1', 'C_1', 'C_1H_1')
ylabel('Magnitude')

subplot(212)
semilogx(f2, angle([H C])*180/pi)
ylabel('Phase [deg]')


