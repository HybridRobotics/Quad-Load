function load_traj = get_load_traj(t)
%% Global Test
f1 = 1/4;
f2 = 1/5;
f3 = 1/7;
f4 = 1/9;

a_x = 2;
a_y = 2.5;
a_z = 1.5;

load_traj.xL = [a_x*(1 - cos(2*pi*f1*t));
    a_y * sin(2*pi*f2*t);
    a_z * cos(2*pi*f3*t)];
load_traj.dxL = 2*pi*[f1*a_x*sin(2*pi*f1*t);
    f2*a_y * cos(2*pi*f2*t);
    f3*1.5 * -sin(2*pi*f3*t)];
load_traj.d2xL = (2*pi)^2*[f1^2*a_x*cos(2*pi*f1*t);
    f2^2*a_y * -sin(2*pi*f2*t);
    f3^2*a_z * -cos(2*pi*f3*t)];
load_traj.d3xL = (2*pi)^3*[f1^3*a_x*-sin(2*pi*f1*t);
    f2^3*a_y * -cos(2*pi*f2*t);
    f3^3*a_z * sin(2*pi*f3*t)];
load_traj.d4xL = (2*pi)^4*[f1^4*a_x*-cos(2*pi*f1*t);
    f2^4*a_y * sin(2*pi*f2*t);
    f3^4*a_z * cos(2*pi*f3*t)];
load_traj.d5xL = (2*pi)^5*[f1^5*a_x*sin(2*pi*f1*t);
    f2^5*a_y * cos(2*pi*f2*t);
    f3^5*a_z * -sin(2*pi*f3*t)];
load_traj.d6xL = (2*pi)^6*[f1^6*a_x*cos(2*pi*f1*t);
    f2^6*a_y * -sin(2*pi*f2*t);
    f3^6*a_z * -cos(2*pi*f3*t)];

end