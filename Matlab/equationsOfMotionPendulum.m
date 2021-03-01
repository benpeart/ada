syms theta phi
syms L g m
syms qd

%% Start with simple pendulum. 
T = [L*sin(phi); 
     -L*cos(phi)];
 
q = [phi];

M = [m];
fi = [0 -m*g].'; % External forces

Tjl = jacobian(T,q);                
Tik = Tjl.';

gj = jacobian(Tjl*qd,q)*qd; 	  % Convective accelerations term

Mbar = Tik*M*Tjl;       % Generalized mass matrix
fbar = Tik*(fi - M*gj); % Generalized forces matrix

qdd_eqn = Mbar\fbar;

state = [q; qd];
stated = [qd; simplify(qdd_eqn)];
parSym = [L g];

stated_fun = matlabFunction(stated, 'vars', {state.', parSym});
stated_fun2 = matlabFunction(stated.', 'vars', {state.', parSym});
x_fun = matlabFunction(T.', 'vars', {state.', parSym});
syms phidd
qdd_sym = [phidd].';

xdd = jacobian(Tjl*qd,q)*qd + Tjl*qdd_sym;
xdd_fun = matlabFunction((xdd.','vars',{state.',qdd_sym,parSym});
% qdd_func = matlabFunction(stated,'vars',{'time',state},'file','state_derivative');
% F_func = matlabFunction(simplify(qdd_eqn(n_q+1:n_q+n_c)),'vars',{state},'file','calc_F');
%%
R = [cos(phi) -sin(phi); sin(phi) cos(phi)];
simplify(inv(R)*xdd)

%% Solve
dtSim = 1e-2;
TSim = 3;
% q0 = [pi/2;0]; 

q0 = [0;0.1]; 
parVal = [1 9.81];

eom_fun = @(t,s) stated_fun(s.', parVal);
[t,y] = ode45(eom_fun, [0:dtSim:TSim], q0);

figure(1); clf;
plot(t,y)
xlabel('Time [s]')
legend('$\phi$', '$\dot{\phi}$', 'Interpreter', 'latex')
grid on

%% Animate
dtAnim = 4*dtSim;

qddVal = stated_fun2(y,parVal);
xddVal = xdd_fun(y, qddVal(:,2), parVal);

figure(2); clf;
plot(t,qddVal)
legend('$\dot{\phi}$','$\ddot{\phi}$', 'Interpreter', 'Latex')

figure(3); clf
plot(t,xddVal)
legend('$\ddot{x}_1$','$\ddot{y}_1$', 'Interpreter', 'Latex')

%%
figure(123); clf;

xy = x_fun(y, parVal);
% I'd like to know XY coordinates of CoMs, makes drawing a lot easier. 
% Draw line and circle, animate / update over time
h1 = plot([0 xy(1,1)], [0 xy(1,2)], '-o', 'LineWidth', 2);

% xlim([-1 1])
ylim([-1 1])
axis equal

for k = 1:length(t)
    h1.XData = [0 xy(k,1)];
    h1.YData = [0 xy(k,2)];
    drawnow;
end

% Idea: place balancing robot in stable equilibrium so that I don't need
% PID control but can still simulate effect of accelerometer :)

% Also: add rotation to frame of accelerometer!

% Also: does xdd represent measured acceleration? Don't think so!
