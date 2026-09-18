clc; clear; close all;

%% ================== SIMULATION CONFIGURATION ==================
dt = 0.01;                
T_final = 3;             
inputType = 'ramp';       

%% ================== SYSTEM PARAMETERS ==================
global K L I_total b_bearing g m_motor lever2motor m_imuss lever2imuss

g = 9.81492;

lever_size  = [0.430 0.005 0.030];
motor_size  = [0.015 0.015];
imuss_size  = [0.015 0.005];
L           = 0.2;

lever2motor = [L, (lever_size(2)+motor_size(2))/2];
lever2imuss = [-L, (lever_size(2)+imuss_size(2))/2];

lever_density = 600;
m_lever  = prod(lever_size) * lever_density;
m_motor  = 0.0145;
m_imuss  = 0.002;

I_lever = m_lever * (lever_size(1)^2 + lever_size(2)^2) / 12;
I_motor = m_motor * (lever2motor(1)^2 + lever2motor(2)^2);
I_imuss = m_imuss * (lever2imuss(1)^2 + lever2imuss(2)^2);
I_total = I_lever + I_motor + I_imuss;

b_bearing = 0.001;
K_T   = 121.7556;
K_tau = (3/2)*(1/sqrt(3))*(60/(2*pi))*(1/4000);
K     = K_T * K_tau;

%% ================== INPUT DEFINITION ==================
t_vec = 0:dt:T_final;  % <-- renamed from t
switch inputType
    case 'step'
        u = (t_vec >= 1) * 0.1;
    case 'ramp'
        u = max(0, 0.01 * t_vec);
    otherwise
        error('Unknown input type: %s', inputType);
end

%% ================== NONLINEAR SIMULATION ==================
x0 = [0; 0];  
dynamicsWrapper = @(t, x) dynamics(x, interp1(t_vec, u, t, 'previous', 0));
[t_nl, x_nl] = ode45(dynamicsWrapper, [0 T_final], x0);

theta_nl = x_nl(:,1);
thetadot_nl = x_nl(:,2);

%% ================== LINEARIZED TRANSFER FUNCTION ==================
linearized_tf = linearizedTF();
[y_lin, t_lin] = lsim(linearized_tf, u, t_vec);
thetadot_lin = gradient(y_lin, dt);

%% ================== PLOTS ==================
figure('Color','w','Position',[100 100 800 500]);

subplot(3,1,1)
plot(t_vec, u, 'r', 'LineWidth', 1.5);
ylabel('Input u(t)');
title(sprintf('Nonlinear vs Linearized System Response (%s Input)', upper(inputType)));
grid on;

subplot(3,1,2)
plot(t_nl, theta_nl, 'k', 'LineWidth', 1.6); hold on;
plot(t_lin, y_lin, '--b', 'LineWidth', 1.5);
ylabel('\theta [rad]');
legend('Nonlinear','Linearized','Location','Best');
grid on;

subplot(3,1,3)
plot(t_nl, thetadot_nl, 'k', 'LineWidth', 1.6); hold on;
plot(t_lin, thetadot_lin, '--b', 'LineWidth', 1.5);
ylabel('\theta\_dot [rad/s]');
xlabel('Time [s]');
legend('Nonlinear','Linearized','Location','Best');
grid on;
