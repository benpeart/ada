syms theta phi x
syms phid xd
syms phidd xdd
syms L g r m1 m2

%% Moving pendulum
T = [x;
    r;
    x+L*sin(phi); 
    r+L*cos(phi); 
    x/r]
 
q = [x;phi];
qd = [xd;phid];

M = diag([m1 m1 m2 m2 0]);
fi = [0 -m1*g 0 -m2*g 0].'; % External forces


Tjl = jacobian(T,q);                
Tik = Tjl.';

gj = jacobian(Tjl*qd,q)*qd; 	  % Convective accelerations term

Mbar = Tik*M*Tjl;       % Generalized mass matrix
fbar = Tik*(fi - M*gj); % Generalized forces matrix

qdd_eqn = simplify(Mbar\fbar)

state = [q; qd];
stated = [qd; qdd_eqn];
parSym = [L g r m1 m2];

stated_fun = matlabFunction(stated, 'vars', {state.', parSym});
stated_fun2 = matlabFunction(stated.', 'vars', {state.', parSym});
x_fun = matlabFunction(T.', 'vars', {state.', parSym});
syms phidd
qdd_sym = [xdd phidd].';

xdds = jacobian(Tjl*qd,q)*qd + Tjl*qdd_sym;
xdd_fun = matlabFunction(xdds.','vars',{state.',qdd_sym.',parSym});
% qdd_func = matlabFunction(stated,'vars',{'time',state},'file','state_derivative');
% F_func = matlabFunction(simplify(qdd_eqn(n_q+1:n_q+n_c)),'vars',{state},'file','calc_F');

%%
R = [cos(phi) -sin(phi); sin(phi) cos(phi)];
simplify(inv(R)*xdds(3:4))

%% Solve
dtSim = 1e-2;
TSim = 3;
% q0 = [pi/2;0]; 

q0 = [0;0]; 
qd0 = [0;0.05];
parVal = [0.3 9.81 0.1 1000 1];

eom_fun = @(t,s) stated_fun(s.', parVal);
[t,y] = ode45(eom_fun, [0:dtSim:TSim], [q0;qd0]);

figure(1); clf;
plot(t,y)
xlabel('Time [s]')
legend('$\phi$', '$\dot{\phi}$', 'Interpreter', 'latex')
grid on

%% Animate
% 
% qddVal = stated_fun2(y,parVal);
% xddVal = xdd_fun(y, qddVal(:,2:2:end), parVal);
% 
% figure(2); clf;
% plot(t,qddVal)
% legend('$\dot{\phi}$','$\ddot{\phi}$', 'Interpreter', 'Latex')
% 
% figure(3); clf
% plot(t,xddVal)
% legend('$\ddot{x}_1$','$\ddot{y}_1$', 'Interpreter', 'Latex')

%%
dtAnim = 4*dtSim;
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
nSpoke = 3;
spokeAngle = (0:nSpoke-1).'/nSpoke*2*pi;
hSpoke = plot(xyPlot(1)+[zeros(nSpoke,1) cos(spokeAngle)*rv].', xyPlot(2)+[zeros(nSpoke,1) sin(spokeAngle)*rv].', 'LineWidth', 2);

hold off


xlim([-1 1])
ylim([-1 1])
% axis equal
daspect([1 1 1])
grid on
%
for k = 1:length(t)
    xyPlot = xy(k,:);
    h1.XData = xyPlot([1 3]);
    h1.YData = xyPlot([2 4]);
    hWheel.Position(1:2) = xyPlot([1 2])-rv;
    set(hSpoke, {'XData'}, num2cell(xyPlot(1)+[zeros(nSpoke,1) cos(xyPlot(5)+spokeAngle)*rv],2),...
        {'YData'}, num2cell(xyPlot(2)+[zeros(nSpoke,1) sin(xyPlot(5)+spokeAngle)*rv],2))
%         'xyPlot(2)+[zeros(nSpoke,1) sin(spokeAngle)*rv].', 'LineWidth', 2);
    drawnow;
    pause(0.01)
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
