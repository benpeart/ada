
set(groot,'DefaultAxesXGrid','on','DefaultAxesYGrid','on')


syms x phi
syms phid xd
syms L g r m1 m2
syms fext

% Add: controller
% Simulate with controller
% Add later: motor model (inertia, gear ratio, gear direction, 


% Next steps: make control fun, with nonlinear elements (e.g. clipping)
% Add motor inertia and gear ratio


%%
T = [x;
    r;
    x+L*sin(phi); 
    r+L*cos(phi); 
    -x/r; % Counter-clockwise wheel rotation is positive
    phi];
 
q = [x;phi];

qd = [xd;phid];

M = diag([m1 m1 m2 m2 0 0]);
fi = [0 -m1*g 0 -m2*g fext fext].'; % External forces


Tjl = jacobian(T,q);                
Tik = Tjl.';

gj = jacobian(Tjl*qd,q)*qd; 	  % Convective accelerations term

Mbar = Tik*M*Tjl;       % Generalized mass matrix
fbar = Tik*(fi - M*gj); % Generalized forces matrix

qdd_eqn = simplify(Mbar\fbar);

% Controller
%  Inputs: reference, and feedback signal
%  Output: torque (for now, until I have a motor model)
syms phiRef Kp Kd fLP
e = phiRef - phi;
ed = -phid;

Cout = e*Kp + ed*Kd;
% qdd_eqn = simplify(subs(qdd_eqn, fext, Cout));

state = [q; qd];
stateDerivative = [qd; qdd_eqn];
parSym = [L g r m1 m2];

stateDerivativeFun = matlabFunction(stateDerivative, 'vars', {state.', parSym, fext}, 'File', 'stateDerivativeFun');
% Used for animation later on: find local coordinates from state variables
localCoordinateFun = matlabFunction(T.', 'vars', {state.', parSym, fext});
qddFun = matlabFunction(qdd_eqn.', 'vars', {state.', parSym, fext});
% controlFun = matlabFunction(Cout, 'vars', {state.', parSym});

%% Double checks
% What is (rotational) acceleration when upper body is horizontal? 
parVal = [.1 9.81 .04 .1 .2 0 0 0]; % Set controller gains to zero
qddFun([0 pi/2 0 0], parVal)

% Expected value: torque due to gravity is m2*g*L
% Acceleration is T/inertia, inertia is m2*L^2
% So, rotational acceleration is g/L = 98.1 --> correct!


% If the robot is leaning forward at a small angle, what acceleration do we
% expect, and what do we get?
phiStart = pi*0.5;
q0 = [0 phiStart 0 0];
parVal = [.1 9.81 0.04 .1 0.1 1 0.05 phiStart];

% The robot leaning forward gives torque T = sin(phi)*L*m2*g
% The wheels will transfer this T to a force on the ground F
% F propels both m1 and m2
% So, expected acceleration is 
T = sin(q0(2))*parVal(1)*parVal(5)*parVal(2)
F = T/parVal(3)
aLin = F/(parVal(4)+parVal(5))

qddFun(q0, parVal)

%%

% %% Try to linearize
% qLin = [0; pi]; % Point at which to linearize (upright positions)
% % q2 = subs(qdd_eqn, fext, m2*g*L*sin(phi));
% J = jacobian(qdd_eqn,q)
% 
% 
% qdd_eqn_lin = simplify(subs(J, q, qLin)*(q-qLin))
% 
% stateDerivativeLin = [qd; qdd_eqn_lin];
% 
% % make state space
% % design PD controller, convert to state space
% 

% Simulate
dtSim = 1e-3;
TSim = 0.5;

phiStart = pi*0.5;

q0 = [0;phiStart]; 
qd0 = [0;0];
parVal = [.1 9.81 0.04 .05 0.1 .1 0.005 0.1];

controlPar.Kp = 0.1;
controlPar.Kd = 0.005;
controlPar.phiRef = 0.1;
controlPar.max = 0.1;
controlPar.min = -0.1;

eom_fun = @(t,s) odeFun(s.', parVal, controlPar);
tic
[t,y] = ode45(eom_fun, [0:dtSim:TSim], [q0;qd0]);
toc
CoutSim = controlFun(y, controlPar);
qddSim = odeFun(y, parVal.*ones(length(t),1), controlPar);

fprintf('Acceleration: %.2f m/s^2\n', qddSim(end,1))
fprintf('Controller output: %.2f mNm\n', CoutSim(end)*1e3)

% Plot state variables
figure(1); clf;
subplot(221)
plot(t,y(:,1:2))
legend('$x$', '$\phi$', 'Interpreter', 'latex')
xlabel('Time [s]')
ylabel('State')

subplot(222)
plot(t, y(:,3:4))
legend('$\dot{x}$', '$\dot{\phi}$', 'Interpreter', 'latex')
xlabel('Time [s]')
ylabel('State derivative')

subplot(223)
plot(t, qddSim)
legend('$\ddot{x}$', '$\ddot{\phi}$', 'Interpreter', 'latex')
xlabel('Time [s]')
ylabel('State acceleration')

subplot(224)
grid on
plot(t, CoutSim)
xlabel('Time [s]')
ylabel('Controller output')

% Animate
figure(123); clf;
xy = localCoordinateFun(y, parVal.*ones(length(t),1));
xyPlot = xy(1,:);
% I'd like to know XY coordinates of CoMs, makes drawing a lot easier. 
% Draw line and circle, animate / update over time
h1 = plot(xyPlot([1 3]), xyPlot([2 4]), 'r-o', 'LineWidth', 5, 'MarkerSize', 10);
hold on
rv = parVal(3);
% Draw wheel with spokes
% Spoke is line from center to radial position
hWheel = rectangle('Position',[xyPlot(1)-rv xyPlot(2)-rv rv*2 rv*2],'Curvature',[1,1], 'LineWidth', 3); % Tire is a rounded rectangle ^^
nSpoke = 5;
spokeAngle = (0:nSpoke-1).'/nSpoke*2*pi;
hSpoke = plot(xyPlot(1)+[zeros(nSpoke,1) cos(spokeAngle)*rv].', xyPlot(2)+[zeros(nSpoke,1) sin(spokeAngle)*rv].', 'LineWidth', 2);
hold off


xlim([-0.3 1.3])
ylim([-0.1 0.3])
% axis equal
daspect([1 1 1])
grid on

%
slowMotionFactor = 1;
animationFPS = 50;
animStep = round(1/animationFPS/dtSim/slowMotionFactor);
% animStep = 1;
dtAnim = animStep*dtSim;
t2 = zeros(size(t));
tStart = tic;
for k = 1:animStep:length(t)
    while toc(tStart)<t(k)*slowMotionFactor
    end
    t2(k) = toc(tStart);
    xyPlot = xy(k,:);
    h1.XData = xyPlot([1 3]);
    h1.YData = xyPlot([2 4]);
    hWheel.Position(1:2) = xyPlot([1 2])-rv;
    set(hSpoke, {'XData'}, num2cell(xyPlot(1)+[zeros(nSpoke,1) cos(xyPlot(5)+spokeAngle)*rv],2),...
        {'YData'}, num2cell(xyPlot(2)+[zeros(nSpoke,1) sin(xyPlot(5)+spokeAngle)*rv],2))
    drawnow;
end

% tStop = toc(tStart)


% Something is off in the EOMs...
% Is it the torque being applied only on the wheel, and not the reaction
% torque on the frame? 

% Study question: what if I make m1 close to zero, m2 relatively large, and
% add a large reaction torque due to motor inertia? 

% Can we accelerate faster than 1*g, or is this a bug in the simulator? 

% Findings:
%  Motor mass shouldn't be too low, otherwise we'll get a very fast
%  accelerating robot (for a given upper body angle)

% Idea: run simulation with remote controller as input :))


