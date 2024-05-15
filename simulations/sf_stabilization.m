%% State feedback control for a quadrotor
% Francisco Salas Porras A01177893

% Clean workspace
close all;
clear;
clc;

%% State definition

d = [0; 0; 0]; % Lineal position [x; y; z]
v = [0; 0; 0]; % Lineal velocity [u; v; w]
theta = [0; 0; 0]; % Angular position [phi; theta; psi]
omega = [0; 0; 0]; % Angular velocity [p; q; r]
R_1 = 0; % Rotation matrix
R_2 = 0; % Kinematic operator
g = [0; 0; 9.81]; % Gravity vector [g_x; g_y; g_z] {m/s^2}
m = 0.068; % Mass {kg}
f = [0; 0; 0]; % External forces vector [f_x; f_y; f_z]
n = [0; 0; 0]; % External moment vector [l; m; n]
I = diag([0.0686*exp(-3), 0.092*exp(-3), 0.1366*exp(-3)]); % Inertia tensor [I_xx, I_yy, I_zz] {kg*m^2}

%% Linearized model

% a3x3 matrix
a3x3 = [g(1,3)*cos(psi_0), g(1,3)*sin(psi_0), 0;
        g(1,3)*sin(psi_0), -g(1,3)*cos(psi_0), 0;
        0, 0, 0];

% State matrix
A = [zeros(3), eye(3), zeros(3,6);
     zeros(3,6), a3x3, zeros(3);
     zeros(3), zeros(3,6), eye(3);
     zeros(3), zeros(3,6), zeros(3)];

% Input matrix
B  = [zeros(4,5);
      1/m, 0, 0, 0;
      zeros(1,3), I];

% Output matrix
C = zeros(4,12); % 4 outputs
C(1,1) = 1; % x
C(2,3) = 1; % y
C(3,5) = 1; % z
C(4,10) = 1; % psi

% Feedthrough matrix
D = 0;

% Motor lift
F = m*g(1,3)/4;

% Linear acceleration
% d_theta = theta(1,2) - theta_0;
% d_phi = theta(1,1) - phi_0;
% d_F = F - F_0;
% xdd = g(1,3)*(d_theta*cos(theta(1,3)) + d_phi*sin(theta(1,3)));
% ydd = g(1,3)*(d_theta*sin(theta(1,3)) - d_phi*cos(theta(1,3)));
% zdd = d_F;

% Angular acceleration
% pd = n(1,1)/I(1,1) * (F_1 - F_4);
% qd = n(1,2)/I(2,2) * (F_3 - F_1);
% rd = n(1,3)/I(3,3) * (F_1 - F_2 + F_3 - F_4);

%% State feedback control

% Controller
% r = 0; % Reference input (regulator)
% u = r - K*x;
% xd = (A - B*K)*x + B*r;



