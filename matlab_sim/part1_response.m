clc; clear; close all;

%% ================== SIMULATION CONFIGURATION ==================
dt = 0.01;                % time step [s]
T_final = 30;             % total simulation time [s]
enable_collision    = true;
enable_perturbation = true;
enable_obs_noise    = true;
enable_animation    = true; % live animation
target_theta = deg2rad(0);  % target [rad]

%% ================== SYSTEM PARAMETERS ==================
global I_total lever2motor b_bearing K_T K_tau g motor_weight lever2imuss imuss_weight

% Gravity
g = 9.81492;  % [m/s^2]

% Component geometry (x: length, y: height, z: width) [m]
lever_size = [0.430 0.005 0.030];   % lever
motor_size = [0.015 0.015];         % motor (width, height)
propl_size = [0.130 0.005];         % propeller (diameter?, thickness)
imuss_size = [0.015 0.005];         % imu sensor (width, height)

% Offsets
L = 0.2;  % lever half length [m]
lever2motor = [L (lever_size(2)+motor_size(2))/2];  % lever->motor [m]
motor2propl  = [0 (motor_size(2)+propl_size(2))/2]; % motor->prop [m]
lever2propl  = lever2motor + motor2propl;           % lever->prop [m]
lever2imuss = [-L (lever_size(2)+imuss_size(2))/2]; % lever->imu [m]

% Mass & inertia
lever_density = 600;                              % [kg/m^3]
lever_weight  = prod(lever_size) * lever_density; % [kg]
motor_weight  = 0.0145;                           % [kg]
imuss_weight  = 0.002;                            % [kg]

I_lever  = lever_weight * (lever_size(1)^2 + lever_size(2)^2) / 12; % [kg*m^2]
I_motor  = motor_weight * (lever2motor(1)^2 + lever2motor(2)^2);    % [kg*m^2]
I_imuss  = imuss_weight * (lever2imuss(1)^2 + lever2imuss(2)^2);    % [kg*m^2]
I_total  = I_lever + I_motor + I_imuss;                             % total inertia [kg*m^2]

% Fulcrum & friction
fulcr_angle = deg2rad(15);                                 % fulcrum half-angle [rad]
fulcr_leg   = lever2motor(1) + propl_size(1)/2 + 0.03;     % fulcrum leg length [m]
b_bearing   = 0.001;                                       % bearing damping [N*m/(rad/s)]

% Motor & thrust gains
K_v   = 4000; 
K_tau = (3/2)*(1/sqrt(3))*(60/(2*pi))*(1/K_v); % torque constant [N*m/A]
K_T   = 121.7556;                               % thrust-to-torque gain [N/(N*m)]
K     = K_tau*K_T;

%% ================== RESPONSE PLOT ==================
t_end = 10;                     % total simulation time [s]
x0 = [0; 0];                    % initial state [theta, theta_dot]

% Choose input type: 'step' or 'ramp'
inputType = 'step';             % <-- change this to 'ramp' if desired

%% === Input Function (u must be >= 0) ===
switch inputType
    case 'step'
        u_func = @(t) (t >= 1) * 0.1;  % step from 0 to 1 at t = 1s
    case 'ramp'
        u_func = @(t) max(0, 0.01 * t); % ramp increasing at 0.01 per sec
    otherwise
        error('Unknown input type: %s', inputType);
end

%% === Define ODE for Simulation ===
dynamicsWrapper = @(t, x) dynamics(x, u_func(t));

%% === Run Simulation ===
[t, x] = ode45(dynamicsWrapper, [0 t_end], x0);

% Compute control input over time for plotting
u_vals = arrayfun(u_func, t);

%% === Plot Results ===
figure('Color','w','Position',[100 100 700 400]);

subplot(3,1,1)
plot(t, u_vals, 'r', 'LineWidth', 1.5);
ylabel('Input u(t)');
title(sprintf('System Response to %s Input', upper(inputType)));
grid on;

subplot(3,1,2)
plot(t, x(:,1), 'b', 'LineWidth', 1.5);
ylabel('\theta [rad]');
grid on;

subplot(3,1,3)
plot(t, x(:,2), 'k', 'LineWidth', 1.5);
ylabel('\theta\_dot [rad/s]');
xlabel('Time [s]');
grid on;