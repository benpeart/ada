syms theta phi x
syms phid xd
syms phidd xdd
syms L g r m1 m2

%% Moving pendulum
T = [x;
    r;
    x+L*sin(phi); 
     -L*cos(phi)]
 
q = [x;phi];
qd = [xd;phid];

M = diag([m1 m1 m2 m2]);
fi = [0 0 0 -m2*g].'; % External forces


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
TSim = 5;
% q0 = [pi/2;0]; 

q0 = [0;pi]; 
qd0 = [0;0.01];
parVal = [1 9.81 0.2 1 1];

eom_fun = @(t,s) stated_fun(s.', parVal);
[t,y] = ode45(eom_fun, [0:dtSim:TSim], [q0;qd0]);

figure(1); clf;
plot(t,y)
xlabel('Time [s]')
legend('$\phi$', '$\dot{\phi}$', 'Interpreter', 'latex')
grid on

%% Animate

qddVal = stated_fun2(y,parVal);
xddVal = xdd_fun(y, qddVal(:,2:2:end), parVal);

figure(2); clf;
plot(t,qddVal)
legend('$\dot{\phi}$','$\ddot{\phi}$', 'Interpreter', 'Latex')

figure(3); clf
plot(t,xddVal)
legend('$\ddot{x}_1$','$\ddot{y}_1$', 'Interpreter', 'Latex')

%%
dtAnim = 4*dtSim;
figure(123); clf;

xy = x_fun(y, parVal.*ones(length(t),1));
% I'd like to know XY coordinates of CoMs, makes drawing a lot easier. 
% Draw line and circle, animate / update over time
h1 = plot(xy(1,[1 3]), xy(1,[2 4]), '-o', 'LineWidth', 2, 'MarkerSize', 10);

% xlim([-1 1])
ylim([-1 1])
axis equal
grid on

for k = 1:length(t)
    h1.XData = xy(k,[1 3]);
    h1.YData = xy(k,[2 4]);
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

% - Record video! Good to have initial versions
% -- Videowriter
% - Implement sensor algorithm
% -- Gyro: noise + integration --> show drift
% -- Accelerometer: lots of noise
% -- Show error due to "disturbance" acceleration
