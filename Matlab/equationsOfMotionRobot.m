% Derive equations and motion for a wheeled inverted pendulum (robot)
%
% Wouter Klop
% 23-02-2021
% wouter@elexperiment.nl

syms theta phi x
syms phid xd
syms phidd xdd
syms L g r m1 m2
syms fext

% To do:
% - Store movie (videowriter)
% - Prescribed motion profile, either linear translation or rotation
%   - Solve state equation for presribed profile
%   - Use outcome in xdd equation
%   - Estimate error

% Fix:
% - Compute acceleration from prescribed motion
%

%% Moving pendulum
T = [x;
    r;
    x+L*sin(phi); 
    r+L*cos(phi); 
    x/r]
 
q = [x;phi];
qd = [xd;phid];

M = diag([m1 m1 m2 m2 0]);
fi = [0 -m1*g 0 -m2*g fext].'; % External forces


Tjl = jacobian(T,q);                
Tik = Tjl.';

gj = jacobian(Tjl*qd,q)*qd; 	  % Convective accelerations term

Mbar = Tik*M*Tjl;       % Generalized mass matrix
fbar = Tik*(fi - M*gj); % Generalized forces matrix

qdd_eqn = simplify(Mbar\fbar)

state = [q; qd];
stated = [qd; qdd_eqn];
parSym = [L g r m1 m2 fext];

stated_fun = matlabFunction(stated, 'vars', {state.', parSym});
qdd_fun = matlabFunction(qdd_eqn.', 'vars', {state.', parSym});
x_fun = matlabFunction(T.', 'vars', {state.', parSym});
syms phidd
qdd_sym = [xdd phidd].';

xdd_sym = jacobian(Tjl*qd,q)*qd + Tjl*qdd_sym;
xdd_sym(2) = xdd_sym(1);
xdd_fun = matlabFunction(xdd_sym.','vars',{state.',qdd_sym.',parSym});
% qdd_func = matlabFunction(stated,'vars',{'time',state},'file','state_derivative');
% F_func = matlabFunction(simplify(qdd_eqn(n_q+1:n_q+n_c)),'vars',{state},'file','calc_F');

%%
R = [cos(phi) -sin(phi); sin(phi) cos(phi)];
acc_sym = simplify((R)*xdd_sym(3:4))

accx_sym = [g*sin(phi) xdd*cos(phi) L*phidd];
accy_sym = [g*cos(phi) xdd*sin(phi) -L*phid^2];
accxy_sym = [accx_sym accy_sym];
accxy_fun = matlabFunction(accxy_sym,'vars',{state.',qdd_sym.',parSym});

%% Solve
dtSim = 5e-3;
TSim = 3;

q0 = [0;pi*0.9]; 
qd0 = [0;0.0];
parVal = [0.2 9.81 0.04 0.1 0.5 0.05];

eom_fun = @(t,s) stated_fun(s.', parVal);
[t,y] = ode45(eom_fun, [0:dtSim:TSim], [q0;qd0]);

figure(1); clf;
plot(t,y)
xlabel('Time [s]')
legend('$\phi$', '$\dot{\phi}$', 'Interpreter', 'latex')
grid on

qddVal = qdd_fun(y, parVal);

%% Prescribed motion
% Case 1: pendulum at fixed position, linear acceleration profile

% xddMax = 1; % [m/s^2] Linear acceleration
% xddT = 1; % [s] Acceleration duration
% 
% xdVal = linspace(0,xddMax, floor(xddT/dtSim)).';
% xdVal = [xdVal; xdVal(end)*ones(size(xdVal)); flip(xdVal)];
% 
% t = (1:length(xdVal)).'*dtSim;
% xVal = cumtrapz(t,xdVal);
% 
% phiVal = 10/180*pi * ones(size(t));
% phidVal = 0 * ones(size(t));

% Case 2: robot at fixed position, acceleration profile in rotation
phiddMax = 120/180*pi;
phiddT = 0.5;

phidVal = linspace(0,phiddMax, floor(phiddT/dtSim)).';
phidVal = [phidVal; phidVal(end)*ones(size(phidVal)); flip(phidVal)];

t = (1:length(phidVal)).'*dtSim;
phiVal = cumtrapz(t,phidVal);

xVal = zeros(size(t));
xdVal = zeros(size(t));

y = [xVal phiVal xdVal phidVal];

qddVal = [diff(xdVal)/dtSim diff(phidVal)/dtSim];
qddVal(end+1,:) = qddVal(end,:);

figure(1)
clf
plot(t, y)


%% Animate
% 
% statedVal = stated_fun2(y,parVal);
% qddVal = qdd_fun(y, parVal);
xddVal = xdd_fun(y, qddVal, parVal);
accVal = accxy_fun(y, qddVal, parVal);

figure(5)
plot(t,xddVal(:,1:4))
plot(t,qddVal)
% plot(t,accVal)

%%
% 
% figure(2); clf;
% plot(t,qddVal)
% legend('$\dot{\phi}$','$\ddot{\phi}$', 'Interpreter', 'Latex')
% 
% figure(3); clf
% plot(t,xddVal)
% legend('$\ddot{x}_1$','$\ddot{y}_1$', 'Interpreter', 'Latex')


phiEstimate = atan2(accVal(:,1),accVal(:,4));
phiEstimate2 = atan2(sum(accVal(:,[1 2 3]),2), sum(accVal(:,[4 5 6]),2));

figure(4); clf
subplot(221)
plot(t,accVal(:,1:3))
xlabel('Time [s]')
ylabel('x acceleration [m/s^2]')
legend('g*cos(phi)', 'xdd*cos(phi)', 'L*phidd')
subplot(222)
plot(t,accVal(:,4:6))
ylabel('y acceleration [m/s^2]')
legend('g*sin(phi)', 'xdd*sin(phi)', '-L*phid^2')

subplot(223)
plot(t, y(:,2)*180/pi, 'b', 'LineWidth', 2)
hold on
plot(t, unwrap(phiEstimate)*180/pi, 'r--', 'LineWidth', 2)
plot(t, unwrap(phiEstimate2)*180/pi, 'g--', 'LineWidth', 2)
hold off



%%
figure(123); clf;

xy = x_fun(y, parVal.*ones(length(t),1));
xyPlot = xy(1,:);
% I'd like to know XY coordinates of CoMs, makes drawing a lot easier. 
% Draw line and circle, animate / update over time
h1 = plot(xyPlot([1 3]), xyPlot([2 4]), '-o', 'LineWidth', 2, 'MarkerSize', 10);
hold on
rv = parVal(3);
% Draw wheel with spokes
% Spoke is line from center to radial position

hWheel = rectangle('Position',[xyPlot(1)-rv xyPlot(2)-rv rv*2 rv*2],'Curvature',[1,1], 'LineWidth', 3);
nSpoke = 5;
spokeAngle = (0:nSpoke-1).'/nSpoke*2*pi;
hSpoke = plot(xyPlot(1)+[zeros(nSpoke,1) cos(spokeAngle)*rv].', xyPlot(2)+[zeros(nSpoke,1) sin(spokeAngle)*rv].', 'LineWidth', 2);

hold off


xlim([-0.3 2.3])
ylim([-0.3 0.3])
% axis equal
daspect([1 1 1])
grid on

%
animStep = round(1/25/dtSim);
% animStep = 1;
dtAnim = animStep*dtSim;
t2 = zeros(size(t));
tStart = tic;
for k = 1:animStep:length(t)
    while toc(tStart)<t(k)
    end
    t2(k) = toc(tStart);
    xyPlot = xy(k,:);
    h1.XData = xyPlot([1 3]);
    h1.YData = xyPlot([2 4]);
    hWheel.Position(1:2) = xyPlot([1 2])-rv;
    set(hSpoke, {'XData'}, num2cell(xyPlot(1)+[zeros(nSpoke,1) cos(-xyPlot(5)+spokeAngle)*rv],2),...
        {'YData'}, num2cell(xyPlot(2)+[zeros(nSpoke,1) sin(-xyPlot(5)+spokeAngle)*rv],2))
    drawnow;
end

% Idea: place balancing robot in stable equilibrium so that I don't need
% PID control but can still simulate effect of accelerometer :)

% Also: add rotation to frame of accelerometer!

% Also: does xdd represent measured acceleration? Don't think so!

% To do: create function that generates all functions needed for
% solving/plotting/animating (qdd_fun, x_fun, xdd_fun etc)
% - Draw wheel
% - Rotate wheel

% Check sim correctness
% - Energy (lagrange?)
% - length of pendulum

% - Record video! Good to have initial versions
% -- Videowriter
% - Implement sensor algorithm
% -- Gyro: noise + integration --> show drift
% -- Accelerometer: lots of noise
% -- Show error due to "disturbance" acceleration
