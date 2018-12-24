% function [xLd vLd aLd daLd d2aLd d3aLd d4aLd p dp d2p d3p d4p b1 b2 b3 db1 db2 db3 d2b1 d2b2 d2b3] = get_nom_traj(t)
% Use differential flatness to compute various system quantities, given
% load trajectory and higher order time deriviatives.
function [xLd,vLd,aLd,p,dp,d2p,...
    R,omega,domega,Omega,dOmega,daLd,d2aLd,d3p,dR,d2R,f,M] = get_nom_traj(params, load_traj)
%% Constants
mQ = params.mQ ;
mL = params.mL ;
J = params.J ;
g = params.g ;
e1 = params.e1 ;
e2 = params.e2 ;
e3 = params.e3 ;
l = params.l;

%% Derivative of Load Position
xLd = load_traj.xL;
vLd = load_traj.dxL;
aLd = load_traj.d2xL;
daLd = load_traj.d3xL;
d2aLd = load_traj.d4xL;
d3aLd = load_traj.d5xL;
d4aLd = load_traj.d6xL;

%% Cable Tension and Directional Vector
Tp = -mL*(aLd + g*e3) ;
norm_Tp = norm(Tp);
p = Tp / norm_Tp;
xQ = xLd - l*p;

dTp = -mL*daLd ;
dnorm_Tp = 1/norm_Tp * vec_dot(Tp, dTp) ;
dp = (dTp - p*dnorm_Tp) / norm_Tp ;


d2Tp = -mL*d2aLd ;
d2norm_Tp = ( vec_dot(dTp, dTp) + vec_dot(Tp, d2Tp) - dnorm_Tp^2 ) / norm_Tp ;
d2p = ( d2Tp - dp*dnorm_Tp - p*d2norm_Tp - dp*dnorm_Tp) / norm_Tp ;

d3Tp = -mL*d3aLd ;
d3norm_Tp = ( 2*vec_dot(d2Tp, dTp) + vec_dot(dTp, d2Tp)+vec_dot(Tp, d3Tp) - 3*dnorm_Tp*d2norm_Tp) / norm_Tp ;
d3p = (d3Tp - d2p*dnorm_Tp-dp*d2norm_Tp - dp*d2norm_Tp-p*d3norm_Tp - d2p*dnorm_Tp-dp*d2norm_Tp - d2p*dnorm_Tp) / norm_Tp ;

d4Tp = -mL*d4aLd;
d4norm_Tp = ( 2*vec_dot(d3Tp, dTp)+2*vec_dot(d2Tp, d2Tp) + vec_dot(d2Tp, d2Tp)+vec_dot(dTp, d3Tp) + vec_dot(dTp, d3Tp)+vec_dot(Tp, d4Tp) - 3*d2norm_Tp^2-3*dnorm_Tp*d3norm_Tp ...
    - d3norm_Tp*dnorm_Tp) / norm_Tp ;
d4p = ( d4Tp - d3p*dnorm_Tp-d2p*d2norm_Tp - d2p*d2norm_Tp-dp*d3norm_Tp - d2p*d2norm_Tp-dp*d3norm_Tp - dp*d3norm_Tp-p*d4norm_Tp ...
    - d3p*dnorm_Tp-d2p*d2norm_Tp - d2p*d2norm_Tp-dp*d3norm_Tp - d3p*dnorm_Tp-d2p*d2norm_Tp - d3p*dnorm_Tp ) / norm_Tp ;

%% Derivatives of Load Angular Velocity
omega = vec_cross(p,dp);
domega = vec_cross(dp,dp)+vec_cross(p,d2p);

%% Derivatives of Quadrotor's Position
vxQ = vLd - l*dp;
axQ = aLd - l*d2p;
daxQ = daLd - l*d3p;
d2axQ = d2aLd - l*d4p;

b1d = e1;
db1d = zeros(3,1);
d2b1d = zeros(3,1);

fb3 = mQ*(axQ+g*e3) - Tp;
norm_fb3 = norm(fb3);
f = norm_fb3;
b3 = fb3 / norm_fb3;
b3_b1d = vec_cross(b3, b1d);
norm_b3_b1d = norm(b3_b1d);
b1 = - vec_cross(b3, b3_b1d) / norm_b3_b1d;
b2 = vec_cross(b3, b1);
R = [b1 b2 b3];

dfb3 = mQ*(daxQ) - dTp;
dnorm_fb3 = vec_dot(fb3, dfb3) / norm_fb3;
db3 = (dfb3*norm_fb3 - fb3*dnorm_fb3) / norm_fb3^2;
db3_b1d = vec_cross(db3, b1d) + vec_cross(b3, db1d);
dnorm_b3_b1d = vec_dot(b3_b1d, db3_b1d) / norm_b3_b1d;
db1 = (-vec_cross(db3,b3_b1d)-vec_cross(b3,db3_b1d) - b1*dnorm_b3_b1d) / norm_b3_b1d;
db2 = vec_cross(db3, b1) + vec_cross(b3, db1);
dR = [db1 db2 db3];
Omega = vee_map(R'*dR);

d2fb3 = mQ*(d2axQ) - d2Tp ;
d2norm_fb3 = (vec_dot(dfb3, dfb3)+vec_dot(fb3, d2fb3) - dnorm_fb3*dnorm_fb3) / norm_fb3 ;
d2b3 = ( (d2fb3*norm_fb3+dfb3*dnorm_fb3 - dfb3*dnorm_fb3-fb3*d2norm_fb3)*norm_fb3^2 - db3*norm_fb3^2*2*norm_fb3*dnorm_fb3 ) / norm_fb3^4 ;
d2b3_b1d = vec_cross(d2b3, b1d)+vec_cross(db3, db1d) + vec_cross(db3, db1d)+vec_cross(b3, d2b1d) ;
d2norm_b3_b1d = ( (vec_dot(db3_b1d,db3_b1d)+vec_dot(b3_b1d,d2b3_b1d))*norm_b3_b1d - vec_dot(b3_b1d, db3_b1d)*dnorm_b3_b1d ) / norm_b3_b1d^2 ;
d2b1 = ( (-vec_cross(d2b3,b3_b1d)-vec_cross(db3,db3_b1d) - vec_cross(db3,db3_b1d)-vec_cross(b3,d2b3_b1d) - db1*dnorm_b3_b1d-b1*d2norm_b3_b1d )*norm_b3_b1d - db1*norm_b3_b1d*dnorm_b3_b1d ) / norm_b3_b1d^2 ;
d2b2 = vec_cross(d2b3, b1)+vec_cross(db3, db1) + vec_cross(db3, db1)+vec_cross(b3, d2b1) ;
d2R = [d2b1 d2b2 d2b3];
dOmega = vee_map( dR'*dR + R'*d2R ) ; %vee_map( dR'*dR + R'*d2R, true ) ;
M = J*dOmega + vec_cross(Omega, J*Omega);
end

% Test-case Function to test the above function.
function test_nom_traj()
t = linspace(0, 10, 1000)' ;
for j=1:length(t)
    [xLd(:,j) vLd(:,j) aLd(:,j) daLd(:,j) d2aLd(:,j) d3aLd(:,j) d4aLd(:,j) p(:,j) dp(:,j) d2p(:,j) d3p(:,j) d4p(:,j) ...
        b1(:,j) b2(:,j) b3(:,j) db1(:,j) db2(:,j) db3(:,j) d2b1(:,j) d2b2(:,j) d2b3(:,j)] = get_nom_traj(t(j)) ;
end

xLd_comp = cumtrapz_ic(t, vLd', xLd(:,1))' ;
vLd_comp = cumtrapz_ic(t, aLd', vLd(:,1))' ;
aLd_comp = cumtrapz_ic(t, daLd', aLd(:,1))' ;
daLd_comp = cumtrapz_ic(t, d2aLd', daLd(:,1))' ;
d2aLd_comp = cumtrapz_ic(t, d3aLd', d2aLd(:,1))' ;
d3aLd_comp = cumtrapz_ic(t, d4aLd', d3aLd(:,1))' ;

p_comp = cumtrapz_ic(t, dp', p(:,1))' ;
dp_comp = cumtrapz_ic(t, d2p', dp(:,1))' ;
d2p_comp = cumtrapz_ic(t, d3p', d2p(:,1))' ;
d3p_comp = cumtrapz_ic(t, d4p', d3p(:,1))' ;

b1_comp = cumtrapz_ic(t, db1', b1(:,1))' ;
b2_comp = cumtrapz_ic(t, db2', b2(:,1))' ;
b3_comp = cumtrapz_ic(t, db3', b3(:,1))' ;
db1_comp = cumtrapz_ic(t, d2b1', db1(:,1))' ;
db2_comp = cumtrapz_ic(t, d2b2', db2(:,1))' ;
db3_comp = cumtrapz_ic(t, d2b3', db3(:,1))' ;

close all
figure ; plot(t, xLd, ':') ; hold on ; plot(t, xLd_comp) ;
figure ; plot(t, vLd, ':') ; hold on ; plot(t, vLd_comp) ;
figure ; plot(t, aLd, ':') ; hold on ; plot(t, aLd_comp) ;
figure ; plot(t, daLd, ':') ; hold on ; plot(t, daLd_comp) ;
figure ; plot(t, d2aLd, ':') ; hold on ; plot(t, d2aLd_comp) ;
figure ; plot(t, d3aLd, ':') ; hold on ; plot(t, d3aLd_comp) ;

figure ; plot(t, p, ':') ; hold on ; plot(t, p_comp) ;
figure ; plot(t, dp, ':') ; hold on ; plot(t, dp_comp) ;
figure ; plot(t, d2p, ':') ; hold on ; plot(t, d2p_comp) ;
figure ; plot(t, d3p, ':') ; hold on ; plot(t, d3p_comp) ;

figure ; plot(t, b1, ':') ; hold on ; plot(t, b1_comp) ;
figure ; plot(t, b2, ':') ; hold on ; plot(t, b2_comp) ;
figure ; plot(t, b3, ':') ; hold on ; plot(t, b3_comp) ;
figure ; plot(t, db1, ':') ; hold on ; plot(t, db1_comp) ;
figure ; plot(t, db2, ':') ; hold on ; plot(t, db2_comp) ;
figure ; plot(t, db3, ':') ; hold on ; plot(t, db3_comp) ;
end

function x = cumtrapz_ic(t, dx, ic)
x = cumtrapz(t, dx) ;
for j=1:size(ic)
    x(:,j) = x(:,j) + ic(j) ;
end
end