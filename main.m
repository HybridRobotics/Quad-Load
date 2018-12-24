%% Quadrotor Attitude Pendulum SO(3)x S(2)x R - Dynamics Simulation and Geometry Control
close all;

%% Parameters
data.params.mQ = 0.5;
% data.params.mL = 0.087;
data.params.mL = 0.5;
data.params.J = [2.32e-3,0,0;0,2.32e-3,0;0,0,4e-3];
data.params.g = 9.81;
data.params.e1 = [1;0;0];
data.params.e2 = [0;1;0];
data.params.e3 = [0;0;1];
data.params.l = 1;

%% Get initial condition from the nominal trajectory
xL = [-3;-3;2];
vL = zeros(3,1);
th = 90*pi/180;
q = [-sin(th);0;cos(th)];
omega = [0;0;0];
R = eye(3,3);
Omega = [0;0;0];

%% In-Sanity checks on initial condition
if(abs(norm(q)-1) > 1e-2)
    disp('Error in q') ; keyboard ;
end

x_0 = [xL; vL; q; omega; reshape(R, 9,1); Omega];

%% Solving Dynamical Equations
odeopts = odeset('RelTol',1e-6,'AbsTol',1e-6);
[t, x] = ode45(@odefun_control, [0 10], x_0, odeopts, data);


%% Compute various quantities

disp('Computing State Variables and Configuration Errors ') ;
ind = round(linspace(1, length(t), round(0.1*length(t)))) ;
for j=ind
    [~, xLd_, Rd, qd_, f_, M_] = odefun_control(t(j), x(j,:)', data) ;
    [phi_d(j),theta_d(j),psi_d(j)] = RotToRPY_ZXY(Rd) ;
    [phi(j),theta(j),psi(j)] = RotToRPY_ZXY(reshape(x(j,13:21),3,3));
    xLd(j,:)=xLd_';
    M(j,:) = M_';
    f(j,:) = f_';
    qd(j,:) = qd_';
    err_q(j) = 1 - qd_'*x(j, 7:9)';
    err_R(j) = 0.5*trace(eye(3,3) - Rd'*reshape(x(j,13:21),3,3));
    err_xL(j) = norm(x(j,1:3)-xLd_');
    err_xL1(j) = x(j,1)-xLd_(1);
    err_xL2(j) = x(j,2)-xLd_(2);
    err_xL3(j) = x(j,3)-xLd_(3);
end

%% Plotting various states/outputs/inputs
disp('Computing Plotting Figures') ;
ind = round(linspace(1, length(t), round(0.1*length(t)))) ;

% Input Moment
figure;
plot(t(ind), M(ind,1),t(ind), M(ind,2),t(ind), M(ind,3)) ;
legend('M(1)','M(2)','M(3)');title('Quad-Moment');
grid on ; xlabel('Time (s)') ;

% Input Thrust
figure;
plot(t(ind), f(ind,1)) ;
legend('f'); title('Quad-Thrust');
grid on ; xlabel('Time (s)')

% Configuration Errors
figure ; plot(t(ind), err_R(ind)) ;
grid on ; xlabel('Time (s)') ; title('psi-R')
figure ; plot(t(ind), err_q(ind)) ;
grid on ; xlabel('Time (s)') ; title('psi-q')
figure; plot(t(ind),err_xL(ind));
grid on; xlabel('Time (s)'); title('position error');
figure; plot(t(ind),err_xL1(ind),t(ind),err_xL2(ind),t(ind),err_xL3(ind))
grid on; xlabel('Time (s)'); title('Position Error');
% Load Position
figure ; plot(t, x(:, 1:3)) ;
grid ; title('xL') ; legend('x','y','z') ; xlabel('Time (s)') ;

% Quadrotor Attitude
figure ; plot(t(ind), phi_d(ind)*180/pi, 'b:', t(ind), theta_d(ind)*180/pi, 'g:', t(ind), psi_d(ind)*180/pi, 'r:') ;
hold on ; plot(t(ind), phi(ind)*180/pi, 'b', t(ind), theta(ind)*180/pi, 'g', t(ind), psi(ind)*180/pi, 'r') ;
grid ; xlabel('Time (s)') ; ylabel('deg') ; title('Quad Atttitude') ;
legend('phi_d','theta_d','psi_d','phi','theta','psi');

% Load Attitude
figure ; plot(t(ind), qd(ind,1), 'b:',t(ind), qd(ind,2), 'g:',t(ind), qd(ind,3), 'r:') ;
hold on ; plot(t(ind), x(ind, 7),'b',t(ind), x(ind, 8),'g',t(ind), x(ind, 9),'r') ;
xlabel('Time (s)');title('Load Attitude') ; grid ;
legend('q_d1','q_d2','q_d3','q_1','q_2','q_3');

%% Animation
animate_3dquad_load(t, x, t(ind), xLd(ind,:));