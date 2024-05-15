%% State feedback control for a quadrotor
% Francisco Salas Porras A01177893

% Clean workspace
close all;
clear;
clc;

%% Drone parameters

m = 0.068; % Mass {kg}
g = 9.81; % Gravity {m/s^2}
I_xx = 0.0686*exp(-3); % Inertia in x {kg*m^2}
I_yy = 0.092*exp(-3); % Inertia in y {kg*m^2}
I_zz = 0.1366*exp(-3); % Inertia in z {kg*m^2}
k_t = 0.0107; % Thrust coefficient {N*s^2}
t = linspace(0,70,1000); % Time {s}

% Initial conditions
x = zeros(12,1); % State vector

% State at hover
x_bar = zeros(12,1); % State vector
x_bar(5) = 2; % Initial z position

% Control
omega = sqrt(m*g/(4*k_t)); % Angular velocity {rad/s}
u_bar = [omega; omega; omega; omega]; % Control vector

% Desired trajectory
x_r = 2*cos(0.2*t); % Desired x position
y_r = 2*sin(0.2*t); % Desired y position
z_r = 0.2*t; % Desired z position
psi_r = 0; % Desired psi position

%% Space state model

% State matrix
A = zeros(12,12);
A(1,2) = 1; % x
A(2,9) = g; % x_dot
A(3,4) = 1; % y
A(4,7) = -g; % y_dot
A(5,6) = 1; % z
A(7,8) = 1; % phi
A(9,10) = 1; % theta
A(11,12) = 1; % psi

% Input matrix
B = zeros(12,4);
B(6,1) = 1/m; % z_dot
B(8,2) = 1/I_xx; % phi_dot
B(10,3) = 1/I_yy; % theta_dot
B(12,4) = 1/I_zz; % psi_dot

% Output matrix
C = zeros(4,12);
C(1,1) = 1; % x
C(2,3) = 1; % y
C(3,5) = 1; % z
C(4,11) = 1; % psi

% Analyze controllability
C_ctr = ctrb(A,B);
rank_ctr = rank(C_ctr);

% State feedback control
P = [-11, -11, -9, -9, -7, -7, -8, -4, -15, -15, -3, -3]; % Desired poles
K = place(A,B,P); % Control gain matrix

% Closed loop matrix
A_c = A - B*K;

% Evaluate the system
%sys = ss(A_c,B,C,0);
%[y,t,x] = lsim(sys,zeros(size(t)),t,x_bar);

% Plot
plot3(x_r,y_r,z_r)
grid on
