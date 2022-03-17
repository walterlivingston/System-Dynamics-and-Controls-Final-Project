%% INITIALIZATION
clc, clear all, close all

dt = 0.01 ;
tspan = 0:dt:5;
yaw_0 = 0;
IC = [0 0 0 yaw_0];

syms d(t)
d(t) = 0.5*sin(2*t);

a = 1.257; % m
b = 1.593; % m
Cf = 120000; % N/rad
Cr = 184600; % N/rad
Iz = 4292; % kg*m^2
Vcar = 30; % m/s
m = 1856; % kg

%% PART A
% Statespace
W = [((-Cf-Cr)/(m*Vcar)) (((b*Cr - a*Cf)/(m*Vcar^2)) - 1)
     ((b*Cr - a*Cf)/(Iz)) ((-a^2 * Cf - b^2 * Cr)/(Iz*Vcar))];
X = [((Cf)/(m*Vcar))
     ((a*Cf)/(Iz))];
Y = [Vcar 0
     0 1
     ((-Cr-Cf)/(m)) ((b*Cr-a*Cf)/(m*Vcar))];
Z = [0
     0
     ((Cf)/(m))];

full_sys = ss(W,X,Y,Z);
[num, denom] = ss2tf(full_sys(2).A,full_sys(2).B,full_sys(2).C,full_sys(2).D);

% Coefficients for Yaw Rate EOM
A = denom(1); B = denom(2); C = denom(3); D = num(2); E = num(3); 

plant = tf(num, denom);
del_sys = tf(10, [1 10]);
yaw_sys = tf(1, [1 0]);

ol_yaw_rate = del_sys*plant;
ol_yaw = ol_yaw_rate*yaw_sys;

figure
pzplot(ol_yaw)
title('Part A: Eigenvalues for Open Loop System')

figure
step(ol_yaw)
grid on
title('Part A: Step Response of Open Loop System')
ylabel('Heading (rad)')

figure
bode(ol_yaw)
grid on
title('Part A: Bode Plot for Open Loop System')
setoptions(gcr,'MagUnits','abs')
setoptions(gcr,'MagScale','log')

%% PART B
figure
rlocus(ol_yaw)
sgrid(0.7,10.9524)
title('Part B: Root Locus Plot')

% Derived w/ root locus
Kdd = 1;
Kd = 19.06;
Kp = 93.62;
k = 0.0462;

controller = tf([Kdd Kd Kp],1);
c_ol_sys = ol_yaw*controller;
c_cl_sys = feedback(k*c_ol_sys,1);

figure
pzplot(c_cl_sys)
title('Part B: Eigenvalues for Closed Loop System')

figure
step(c_cl_sys)
grid on
title('Part B: Step Response for Closed Loop System')
ylabel('Heading (rad)')
stepinfo(c_cl_sys)

figure
bode(c_cl_sys)
grid on
title('Part B: Bode Plot for Closed Loop System')
setoptions(gcr,'MagUnits','abs')
setoptions(gcr,'MagScale','log')

%% PART C
save tmp.mat
clear all
load tmp.mat

tspan_c = 0:dt:2 ;
heading_des = zeros(1,length(tspan_c)) + 20*pi/180;
del_com(1) = 0;
for i=1:length(tspan_c)-1
    [GPS(i+1,:),yaw_gyro(i+1),del_meas(i+1),WP(i+1,:)]=run_MKZ_fp(del_com(i),Vcar,IC,0);
    e = heading_des(i) - GPS(i,3);
    e_dot = 0 - yaw_gyro(i);
    e_double_dot = 0 - ((yaw_gyro(i+1)-yaw_gyro(i))/dt);
    
    del_com(i+1) = (k*1)*e_double_dot + (k*19.06)*e_dot + (k*93.62)*e;
end

figure
plot(tspan_c, GPS(:,3),'k')
grid on, hold on
lsim(c_cl_sys, heading_des, tspan_c, 'r--')
title('Part C: Step Response on the Actual Vehicle at 30 m/s')
ylabel('Heading (rad)')
legend('Actual','Expected')

save tmp.mat
clear all
load tmp.mat

Vcar = 10; % m/s
heading_des = zeros(1,length(tspan_c)) + 20*pi/180;
del_com(1) = 0;
for i=1:length(tspan_c)-1
    [GPS(i+1,:),yaw_gyro(i+1),del_meas(i+1),WP(i+1,:)]=run_MKZ_fp(del_com(i),Vcar,IC,0);
    e = heading_des(i) - GPS(i,3);
    e_dot = 0 - yaw_gyro(i);
    e_double_dot = 0 - ((yaw_gyro(i+1)-yaw_gyro(i))/dt);
    
    del_com(i+1) = (k*1)*e_double_dot + (k*19.06)*e_dot + (k*93.62)*e;
end

figure
plot(tspan_c, GPS(:,3),'k')
grid on, hold on
lsim(c_cl_sys, heading_des, tspan_c, 'r--')
title('Part C: Step Response on the Actual Vehicle at 10 m/s')
ylabel('Heading (rad)')
legend('Actual','Expected')

%% PART D
save tmp.mat
clear all
load tmp.mat

Vcar = 30; % m/s
del_com(1) = 0;
for i=1:length(tspan)-1
    [GPS(i+1,:),yaw_gyro(i+1),del_meas(i+1),WP(i+1,:)]=run_MKZ_fp(del_com(i),Vcar,IC,0);
    heading_des(i+1) = sin(16.4*tspan(i+1));
    e = heading_des(i) - GPS(i,3);
    e_dot = 0 - yaw_gyro(i);
    e_double_dot = 0 - ((yaw_gyro(i+1)-yaw_gyro(i))/dt);
    
    del_com(i+1) = (k*1)*e_double_dot + (k*19.06)*e_dot + (k*93.62)*e;
end

figure
plot(tspan, 0.5*sin(16.4*tspan),'r--', tspan, GPS(:,3),'k')
grid on
title('Part D: Time Response from a Sinusoidal Input of 16.4 rad/s')
xlabel('Time (seconds)')
ylabel('Heading (rad)')
legend('Expected','Actual')

save tmp.mat
clear all
load tmp.mat

Vcar = 30; % m/s
del_com(1) = 0;
for i=1:length(tspan)-1
    [GPS(i+1,:),yaw_gyro(i+1),del_meas(i+1),WP(i+1,:)]=run_MKZ_fp(del_com(i),Vcar,IC,0);
    heading_des(i+1) = sin(10*tspan(i+1));
    e = heading_des(i) - GPS(i,3);
    e_dot = 0 - yaw_gyro(i);
    e_double_dot = 0 - ((yaw_gyro(i+1)-yaw_gyro(i))/dt);
    
    del_com(i+1) = (k*1)*e_double_dot + (k*19.06)*e_dot + (k*93.62)*e;
end

figure
plot(tspan, 0.5*sin(10*tspan),'r--', tspan, GPS(:,3),'k')
grid on
title('Part D: Time Response from a Sinusoidal Input of 10 rad/s')
xlabel('Time (seconds)')
ylabel('Heading (rad)')
legend('Expected','Actual')

save tmp.mat
clear all
load tmp.mat

Vcar = 30; % m/s
del_com(1) = 0;
for i=1:length(tspan)-1
    [GPS(i+1,:),yaw_gyro(i+1),del_meas(i+1),WP(i+1,:)]=run_MKZ_fp(del_com(i),Vcar,IC,0);
    heading_des(i+1) = sin(5*tspan(i+1));
    e = heading_des(i) - GPS(i,3);
    e_dot = 0 - yaw_gyro(i);
    e_double_dot = 0 - ((yaw_gyro(i+1)-yaw_gyro(i))/dt);
    
    del_com(i+1) = (k*1)*e_double_dot + (k*19.06)*e_dot + (k*93.62)*e;
end

figure
plot(tspan, 0.5*sin(5*tspan),'r--', tspan, GPS(:,3),'k')
grid on
title('Part D: Time Response from a Sinusoidal Input of 5 rad/s')
xlabel('Time (seconds)')
ylabel('Heading (rad)')
legend('Expected','Actual')

%% PART E
save tmp.mat
clear all
load tmp.mat
clear GPS
clear WP

dt = 0.01;
t_f = 500;
tspan = 0:dt:t_f;
X0 = [0 0 pi 0];
Vel = 10;
WP_FILE = 1;

for i=1:length(tspan)-1
    [GPS(i+1,:),yaw_gyro(i+1),del_meas(i+1),WP(i+1,:)]=run_MKZ_fp(del_com(i),Vel,X0,WP_FILE);
    heading_des(i+1) = atan2((WP(i+1,1)-GPS(i+1,1)),(WP(i+1,2)-GPS(i+1,2)));
    e = wrap_angle(heading_des(i) - GPS(i,3),[-pi pi]);
    e_dot = 0 - yaw_gyro(i);
    e_double_dot = 0 - ((yaw_gyro(i+1)-yaw_gyro(i))/dt);
    
    del_com(i+1) = (k*1)*e_double_dot + (k*19.06)*e_dot + (k*93.62)*e;
    if isnan(WP(i+1)) == 1
       break 
    end
end

figure
hold on
plot(WP(:,1),WP(:,2),"og")
plot(GPS(:,1),GPS(:,2),"-k")
legend('Track','Car')
xlabel('East')
ylabel('North')
axis equal
title('Part E: Path Tracking of Track 1 at 10 m/s (Lane Change)')

save tmp.mat
clear all
load tmp.mat
clear GPS
clear WP

dt = 0.01;
t_f = 500;
tspan = 0:dt:t_f;
X0 = [0 0 pi 0];
Vel = 20;
WP_FILE = 1;

for i=1:length(tspan)-1
    [GPS(i+1,:),yaw_gyro(i+1),del_meas(i+1),WP(i+1,:)]=run_MKZ_fp(del_com(i),Vel,X0,WP_FILE);
    heading_des(i+1) = atan2((WP(i+1,1)-GPS(i+1,1)),(WP(i+1,2)-GPS(i+1,2)));
    e = wrap_angle(heading_des(i) - GPS(i,3),[-pi pi]);
    e_dot = 0 - yaw_gyro(i);
    e_double_dot = 0 - ((yaw_gyro(i+1)-yaw_gyro(i))/dt);
    
    del_com(i+1) = (k*1)*e_double_dot + (k*19.06)*e_dot + (k*93.62)*e;
    if isnan(WP(i+1)) == 1
       break 
    end
end

figure
hold on
plot(WP(:,1),WP(:,2),"og")
plot(GPS(:,1),GPS(:,2),"-k")
legend('Track','Car')
xlabel('East')
ylabel('North')
axis equal
title('Part E: Path Tracking of Track 1 at 20 m/s (Lane Change)')

save tmp.mat
clear all
load tmp.mat
clear GPS
clear WP

Vcar = 30; % m/s

WP_FILE = 2;
for i=1:length(tspan)-1
    [GPS(i+1,:),yaw_gyro(i+1),del_meas(i+1),WP(i+1,:)]=run_MKZ_fp(del_com(i),Vcar,X0,WP_FILE);
    heading_des(i+1) = atan2((WP(i+1,1)-GPS(i+1,1)),(WP(i+1,2)-GPS(i+1,2)));
    e = wrap_angle(heading_des(i) - GPS(i,3),[-pi pi]);
    e_dot = 0 - yaw_gyro(i);
    e_double_dot = 0 - ((yaw_gyro(i+1)-yaw_gyro(i))/dt);
    
    del_com(i+1) = (k*1)*e_double_dot + (k*19.06)*e_dot + (k*93.62)*e;
    if isnan(WP(i+1)) == 1
       break 
    end
end

figure
hold on
plot(WP(:,1),WP(:,2),"og")
plot(GPS(:,1),GPS(:,2),"-k")
legend('Track','Car')
xlabel('East')
ylabel('North')
axis equal
title('Part E: Path Tracking of Track 2 at 30 m/s(Indianapolis 500)')

save tmp.mat
clear all
load tmp.mat
clear GPS
clear WP

tspan_e = 0:dt:60;
Vcar = 80; % m/s
WP_FILE = 2;
for i=1:length(tspan_e)-1
    [GPS(i+1,:),yaw_gyro(i+1),del_meas(i+1),WP(i+1,:)]=run_MKZ_fp(del_com(i),Vcar,X0,WP_FILE);
    heading_des(i+1) = atan2((WP(i+1,1)-GPS(i+1,1)),(WP(i+1,2)-GPS(i+1,2)));
    e = wrap_angle(heading_des(i) - GPS(i,3),[-pi pi]);
    e_dot = 0 - yaw_gyro(i);
    e_double_dot = 0 - ((yaw_gyro(i+1)-yaw_gyro(i))/dt);
    
    del_com(i+1) = (k*1)*e_double_dot + (k*19.06)*e_dot + (k*93.62)*e;
    if isnan(WP(i+1)) == 1
       break 
    end
end

figure
hold on
plot(WP(:,1),WP(:,2),"og")
plot(GPS(:,1),GPS(:,2),"-k")
legend('Track','Car')
xlabel('East')
ylabel('North')
axis equal
title('Part E: Path Tracking of Track 2 at 80 m/s (Indianapolis 500)')

%% FUNCTIONS
function [GPS, yaw_rate, delta_meas, WP] = runpcode(vel_in, ...
    d, tspan, IC, wpfile)
% Used to set initial values for outputs from p-code and generate
% output data
% *Must be paired with a clear all and either resetting the necessary
% function inputs manually, or by saving the previous variables (or a
% preconfigured set) to a .mat file and loading them after the clear
% is executed 
    len = length(tspan);
    GPS = zeros(len, 3);
    yaw_rate = zeros(len, 1);
    delta_meas = zeros(len, 1);
    GPS(1,:) = IC(1,1:3);
    yaw_rate(1,1) = IC(4);
    delta_meas(1,1) = d(0);
    for k = 1:(length(tspan)-1)
    [GPS(k+1, :), yaw_rate(k+1, 1), delta_meas(k+1, 1), WP] = ...
            run_MKZ_fp(eval(d(tspan(k))), vel_in, IC, wpfile);
    end
end