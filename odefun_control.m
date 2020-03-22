%% Geometric control design for quadrotor suspended load model
function[dx, xLd, Rd, qd, f, M] = odefun_control(t,x,data)
%% Constants
mL = data.params.mL;
g = data.params.g;
mQ = data.params.mQ;
J = data.params.J;
e1 = data.params.e1;
e2 = data.params.e2;
e3 = data.params.e3;
l = data.params.l;

%% Desired States
%---------------%

% Case 1: Testing
[xLd,vLd,aLd,qd,dqd,d2qd,~,Omegad,dOmegad] = get_nom_traj(data.params, get_load_traj(t));

%% Extracting States
xL = x(1:3);
vL = x(4:6);
q = x(7:9);
omega = x(10:12);
dq = vec_cross(omega, q);
R = reshape(x(13:21), 3,3);
Omega = x(22:24);
b3 = R(:,3);
b1 = R(:,1);

% LOAD POSITION TRACKING

% Position errors
eL = xL - xLd;
deL = vL - vLd;

epsilon_bar = 0.8;
kp_xy = 0.3/epsilon_bar^2; kd_xy = 0.6/epsilon_bar;
k1 = diag([kp_xy kp_xy 2]); k2 = diag([kd_xy kd_xy 1.5]);

% PD force to track trajectory for Load with
% feedforward
A = (-k1*eL - k2*deL + (mQ+mL)*(aLd+g*e3) + mQ*l*vec_dot(dq,dq)*q);
qd = -A/norm(A);

epsilon_q = 0.5;
kp = -1.5/epsilon_q^2; kom = -0.8/epsilon_q;
err_q = hat_map(q)^2*qd;
err_om = dq - vec_cross(vec_cross(qd, dqd), q);

F_pd = -kp*err_q-kom*err_om;
F_ff = (mQ*l)*vec_dot(q, vec_cross(qd,dqd))*vec_cross(q,dq)+...
    (mQ*l)*vec_cross( vec_cross(qd, d2qd), q);
F_n = vec_dot(A,q)*q;

F = F_pd - F_ff + F_n;

b3c = F/norm(F);

f = vec_dot(F, R(:,3));

% Load position
xL_dot = vL;
vL_dot = 1/(mQ+mL)*((vec_dot(q,f*b3)-mQ*l*vec_dot(dq,dq))*q-(mQ+mL)*g*e3);

if(abs(norm(qd)-1) > 1e-2)
    disp('Error in pd'); keyboard;
end

% Load Attitude
q_dot = dq;
omega_dot = 1/(mQ*l) * vec_cross(-q, f*b3);

% DESIRED YAW DIRECTION
b1d = e1;
b1c = -vec_cross(b3c,vec_cross(b3c,b1d))/norm(vec_cross(b3c,b1d));
Rc = [b1c, vec_cross(b3c,b1c),b3c];
Rd = Rc;
if(norm(Rd'*Rd-eye(3)) > 1e-2)
    disp('Error in R'); keyboard;
end
kR = 4 ; kOm = 4 ;
epsilon = 0.1 ; %.5 ; %0.01 ;

err_R = 1/2 * vee_map(Rd'*R - R'*Rd);
err_Om = Omega - R'*Rd*Omegad;
M = -kR/epsilon^2*err_R - kOm/epsilon*err_Om + vec_cross(Omega, J*Omega)...
    - J*(hat_map(Omega)*R'*Rd*Omegad - R'*Rd*dOmegad);

% Quadrotor Attitude
R_dot = R*hat_map(Omega);
Omega_dot = J\( -vec_cross(Omega, J*Omega) + M );

%% Output
dx = [xL_dot; vL_dot; q_dot; omega_dot; reshape(R_dot, 9,1); Omega_dot];

if nargout <= 1
   fprintf('Simulation time %0.4f seconds \n',t);
end

end