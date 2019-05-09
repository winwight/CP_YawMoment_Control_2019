%% Main M-File

clc
close all

% Load Protean PD18 Motor Data
load('Protean_PD18_motormap.mat')
maxonmot = xlsread('Maxon Motor Specs.xlsx', 'Sheet1', 'I3:J614'); 

% Real-time sampling rate
Ts  = 0.015;    % Trajectory reconstruction sampling rate

% % Case Study Vehicle Parameters
% m  = 1400; % Vehicle mass [kg]
% m_s = 0.9*m; % Sprung mass  [kg]
% 
% % Moments of inertia
% Ix = 900;    % Roll inertia [kg-m^2]
% Iy = 3200;   % Pitch inertia
% Iz = 2000;   % Yaw inertia
% Ixz = 0;  
% J   = 0.9;   % Tire polar moment of inertia [kg-m^2] from cartech.com
% 
% % Vehicle geometry
% WB  = 2.88;     % Wheelbase [m]
% l_f = 1.1907;   % Distance from CG to front axle
% l_r = WB-l_f;   % Distance from CG to rear axle
% w1 = 1.565;     % Front track width [m]
% w2 = 1.565;     % Rear track width [m]
% hc = 0.60;       % Height of the CG
% Cd = 0.32;      % Cd, coefficient of drag
% Af = 2.0;       % A, frontal area
% R  = 0.35;       % Tire radius, [m]
% i_steer = 16.19; % Steering ratio
% 
% % Roll parameters
% h_rc_f = 0.00;      % Height of the front axle roll center [m]
% h_rc_r = 0.05;   % Height of the rear axle roll center [m]
% h_s    = 0.05;    % Height of sprung mass CG above unsprung mass CG [m]
% K_f    =  105000; % Front roll stiffness [Nm/rad]
% K_r    =  55000; % Rear roll stiffness  [Nm/rad]
% K_roll =  K_f + K_r;
% C_f    =   2000; % Front roll damping   [Nm-s/rad]
% C_r    =   1500; % Rear roll damping    [Nm-s/rad]
% C_roll =  C_f + C_r;
% 
% % Tire Stiffness
% C_af = 45784;    % Front cornering stiffness [N/rad]
% C_lf = 1500;    % Front longitudinal stiffness [N]
% C_ar = 32270;    % Rear cornering stiffness [N/rad]
% C_lr = 1500;    % Rear longitudinal stiffness [N]

% CPSSIV Parameters
m  = 3.832;  % Vehicle mass [kg]
m_s = 0.9*m; % Sprung mass  [kg]

% Moments of inertia
Ix = 0.061;    % Roll inertia [kg-m^2]
Iy = 0.218;   % Pitch inertia
Iz = 0.136;   % Yaw inertia
Ixz = 0;  
J   = 0.9;   % Tire polar moment of inertia [kg-m^2] from cartech.com

% Vehicle geometry
WB  = 0.406;     % Wheelbase [m]
l_f = 0.1968;   % Distance from CG to front axle
l_r = 0.2042;   % Distance from CG to rear axle
w1 = 0.2824;     % Front track width [m]
w2 = 2.929;     % Rear track width [m]
hc = 0.0751;       % Height of the CG
Cd = 0.32;      % Cd, coefficient of drag
Af = 0.2;       % A, frontal area
R  = 0.055;       % Tire radius, [m]
i_steer = 0; % Steering ratio

% Roll parameters (Assume Jato suspension params from https://traxxas.com/pitpass/howto/news/Jato-Performance-Guide-Suspension-Tuning
h_rc_f = 0.00;      % Height of the front axle roll center [m]
h_rc_r = 0.00;   % Height of the rear axle roll center [m]
h_s    = 0.05;    % Height of sprung mass CG above unsprung mass CG [m]
l_cs   = 0.008;   % front-center to bottom suspension pin distance [m]
K_lin  = 1500;    % linear suspension spring rate [N/m]
phi    = 10;     % spring angle measured from vertical [degrees]
K_f    = K_lin*l_cs^2*cos(phi*pi/180); % Front roll stiffness [Nm/rad] calculated
                                       % from center-to-spring distance * 1 radian
                                       % *cos(spring angle from vertical) * 
                                       % spring rate [N/mm]
K_r    =  K_lin*l_cs^2*cos(phi*pi/180); % Rear roll stiffness  [Nm/rad]
K_roll =  K_f + K_r;
C_f    =   0; % Front roll damping   [Nm-s/rad]
C_r    =   0; % Rear roll damping    [Nm-s/rad]
C_roll =  C_f + C_r;

% Tire Stiffness
C_af = 60;    % Front cornering stiffness [N/rad]
C_ar = 32270;    % Rear cornering stiffness [N/rad]

% Environmental conditions
mu  = 0.8;    % Tire/road friction coefficient [-]
f   = 0.015;
rho = 1.2037; % Air density at standard temp, pressure [kg/m^3]
g = 9.81;     % Acceleration due to gravity [m/s^2] 

% Static weight distribution
F_z1_s   = l_r * m * g / 2 / WB;
F_z2_s   = F_z1_s;
F_z3_s   = l_f * m * g / 2 / WB;
F_z4_s   = F_z3_s;

% Controller parameters
xi       = .01;
k_SMC    = 1;
phi_1    = 100;
phi_2    = 100;
Ki_SMC   = 100;

Kp       =  600;      % Speed controller proportional gain
Ki       =  75;      % Speed controller integral gain
Kd       =  0;      % Speed controller derivative gain

Kp_yaw   =  15000;
Ki_yaw   =  1500;
Kd_yaw   =  0;

k1       =   2000;
k2       =   1000;
boundary =   3000;
eta      =   0;

K_SMC    =  1.0;

% Reference conditions
V_x0     =   40;                % Reference velocity [km/h]
V_x0     = (1000/3600)*V_x0;    % Reference velocity [m/s]
beta_ref =    .001;                % Reference sideslip angle

% Extended Kalman Filter initialization
x_0       = [0 0 0 0 0]'; % Initial state vector
P_0       = 10000*randi(5,5); % Initial covariance matrix

sig_ax_m  = 0;     % Measured longitudinal acceleration standard dev
sig_ay_m  = 0;    % Measured lateral acceleration standard dev
sig_r_m   = 0;    % Measured yaw-rate standard dev
sig_p_m   = 0;  % Measured roll rate standard dev
sig_phi_m = 0;   % Measured roll angle standard dev
sig_Vx_p  = 0;     % Process longitudinal velocity standard dev
sig_Vy_p  = 0;    % Process lateral velocity standard dev
sig_r_p   = 0;    % Process yaw-rate standard dev
sig_p_p   = 0;  % Process roll rate standard dev
sig_phi_p = 0;   % Process roll angle standard dev

Q_EKF = [sig_Vx_p^2 0 0 0 0;
         0 sig_Vy_p^2 0 0 0;
         0 0 sig_r_p^2  0 0;
         0 0 0 sig_p_p^2 0;
         0 0 0 0 sig_phi_p^2];
 
R_EKF = [sig_ax_m^2 0 0 0 0;
         0 sig_ay_m^2 0 0 0;
         0 0 sig_r_m^2 0 0;
         0 0 0 sig_p_m^2 0;
         0 0 0 0 sig_phi_m^2];

%% Vertical Tire Force Estimator Parameters
%  Linear observer design
A_vte = [1 0 -2*Ts*m_s/WB*(l_r*h_rc_f/w1+l_f*h_rc_r/w2) 0 -2*Ts*(K_f/w1+K_r/w2);
         0 1 Ts 0 0;
         0 0 1 0 0;
         0 0 0 1 Ts;
         0 Ts*m_s*h_s/Ix 0 Ts*(m_s*g*h_s-K_r)/Ix 1-Ts*C_r/Ix];
C_vte = [0 1 0 g 0;
         0 0 0 1 0;
         0 0 0 0 1;
         1 0 0 0 0];
    
A_vte_p2 = [1 0 0 0 0 -Ts * m * hc / 2 / WB 0 -Ts * l_r * m * hc / WB / w1;
            0 1 0 0 0 -Ts * m * hc / 2 / WB 0 -Ts * l_r * m * hc / WB / w1;
            0 0 1 0 0 -Ts * m * hc / 2 / WB 0 -Ts * l_f * m * hc / WB / w2;
            0 0 0 1 0 -Ts * m * hc / 2 / WB 0 -Ts * l_f * m * hc / WB / w2;
            0 0 0 0 1  Ts 0 0;
            0 0 0 0 0  1 0 0;
            0 0 0 0 0  0 1 Ts;
            0 0 0 0 0  0 0 1];

C_vte_p2 = [1, -1, 1, -1, 0, 0, 0, 0;
            1, 1, 0, 0, 0, 0, 0, 0;
            0, 0, 0, 0, 1, 0, 0, 0;
            0, 0, 0, 0, 0, 0, 1, 0;
            1, 1, 1, 1, 0, 0, 0, 0];

var_ay_m_p   = 1e-3;
var_phi_p    = 1e-4;
var_p_p      = 1e-5;
var_del_Fzl_p = 1e1;

var_ay_m_m   = 1e-3;
var_ay_dot_m = 1e-4;
var_phi_m    = 1e-4;
var_p_m      = 1e-5;
var_del_Fzl_m = 1e1;

Q_proc_susp_p1 = [var_del_Fzl_m^2 0 0 0 0;
                  0 var_ay_m_m^2 0 0 0;
                  0 0 var_ay_dot_m^2 0 0;
                  0 0 0 var_phi_m^2 0;
                  0 0 0 0 var_p_m^2];

Q_meas_susp_p1 = [var_ay_m_p^2 0 0 0;
                  0 var_phi_p^2 0 0;
                  0 0 var_p_p^2 0;
                  0 0 0 var_del_Fzl_p^2];


var_Fz  = 10;
var_ax  = .01;
var_ax_dot = .1;
var_ay  = .01;
var_ay_dot = .1;

Q_proc_susp_p2 = 1e0*[var_Fz^2 0 0 0 0 0 0 0;
                     0 var_Fz^2 0 0 0 0 0 0;
                     0 0 var_Fz^2 0 0 0 0 0;
                     0 0 0 var_Fz^2 0 0 0 0;
                     0 0 0 0 var_ax^2 0 0 0;
                     0 0 0 0 0 var_ax_dot^2 0 0;
                     0 0 0 0 0 0 var_ay^2 0;
                     0 0 0 0 0 0 0 var_ay_dot^2];

var_del_Fz = 1e0;
var_Fz_sum  = 1e1;
var_ax_susp = 1e-3;
var_ay_susp = 1e-3;
var_Fz_tot  = 1e3;

Q_meas_susp_p2 = [var_del_Fz^2 0 0 0 0;
                  0 var_Fz_sum^2 0 0 0;
                  0 0 var_ax_susp^2 0 0;
                  0 0 0 var_ay_susp^2 0;
                  0 0 0 0 var_Fz_tot^2];

var_r = .001;
var_Vx = .01;
var_Vy = .1;
var_F = 10;

Q_lat_process = 1E0*[var_r^2  0 0 0 0 0 0 0 0;
                      0 var_Vx^2 0 0 0 0 0 0 0;
                      0 0 var_Vy^2 0 0 0 0 0 0;
                      0 0 0 var_F^2 0 0 0 0 0;
                      0 0 0 0 var_F^2 0 0 0 0;
                      0 0 0 0 0 var_F^2 0 0 0;
                      0 0 0 0 0 0 var_F^2 0 0;
                      0 0 0 0 0 0 0 var_F^2 0;
                      0 0 0 0 0 0 0 0 var_F^2];

var_r_s  = .05;
var_Vx_s = .1;
var_ax_s = .01;
var_ay_s = .01;

Q_lat_sensor = 1E-2*[var_r_s^2 0 0 0;
                     0 var_Vx_s^2 0 0;
                     0 0 var_ax_s^2 0;
                     0 0 0 var_ay_s^2];

% Optimization vector
b1 = ones(4,1)/R;

% Start simulation
set_param('DYC_controller', 'SimulationCommand', 'start')