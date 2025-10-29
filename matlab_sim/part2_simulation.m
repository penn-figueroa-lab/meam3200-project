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
global I_total lever2motor b_bearing K_T K_tau g m_motor lever2imuss m_imuss

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
lever_density = 600;                         % [kg/m^3]
m_lever  = prod(lever_size) * lever_density; % [kg]
m_motor  = 0.0145;                           % [kg]
m_imuss  = 0.002;                            % [kg]

I_lever  = m_lever * (lever_size(1)^2 + lever_size(2)^2) / 12; % [kg*m^2]
I_motor  = m_motor * (lever2motor(1)^2 + lever2motor(2)^2);    % [kg*m^2]
I_imuss  = m_imuss * (lever2imuss(1)^2 + lever2imuss(2)^2);    % [kg*m^2]
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

% Initial state: [theta; theta_dot] (rad, rad/s)
state = [0; 0];

%% ================== PID PARAMETERS ==================
% Initial parameters
Kp = 1.0; Ki = 0.0; Kd = 0.0;

pid = PIDController(Kp, Ki, Kd);
linearized_tf = linearizedTF();
pid.runTuner(1/tf('s'))


%% ================== SHAPES FOR ANIMATION ==================
rect = @(lx, ly, cx, cy) polyshape(lx*[-.5 .5 .5 -.5]+cx, ly*[-.5 -.5 .5 .5]+cy);
diamond = @(lx, ly, cx, cy) polyshape(lx*[0 .5 0 -.5]+cx, ly*[-.5 0 .5 0]+cy);

lever_poly  = rect(lever_size(1), lever_size(2), 0, 0);
motor_poly  = rect(motor_size(1), motor_size(2), lever2motor(1), lever2motor(2));
imuss_poly  = rect(imuss_size(1), imuss_size(2), lever2imuss(1), lever2imuss(2));
propl_poly  = diamond(propl_size(1), propl_size(2), lever2propl(1), lever2propl(2));
fulcr_poly  = polyshape([0 fulcr_leg*sin(fulcr_angle) -fulcr_leg*sin(fulcr_angle)], ...
                        [0 -fulcr_leg*cos(fulcr_angle) -fulcr_leg*cos(fulcr_angle)]);

if enable_animation
    figure('Color','w'); hold on; axis equal;
    xlim([-0.3 0.3]); ylim([-0.3 0.3]);
    plot(fulcr_poly,'FaceColor',[0 0.45 0.74]);
    lever_plot  = plot(lever_poly,'FaceColor',[0.55 0.27 0.07]);
    motor_plot  = plot(motor_poly,'FaceColor',[0.85 0.33 0.1]);
    imuss_plot  = plot(imuss_poly,'FaceColor',[0.5 0.5 1.0]);
    propl_plot  = plot(propl_poly,'FaceColor',[1 0.6 0.2]);
    perturb_plot = quiver(0,0,0,0,0,'-r','LineWidth',3,'ShowArrowHead','on');
    title_handle = title('Time = 0.00 s, Input = 0.00 A');
    xlabel('X [m]'); ylabel('Y [m]');
    set(gca,'FontSize',10,'Box','on','LineWidth',1);
end

%% ================== DATA LOGGING ==================
steps = T_final/dt;
log_data = zeros(steps,5); % [t, u, theta, theta_dot, perturb_flag]

%% ================== PERTURBATION VARIABLES ==================
perturb_timer = 0;      % steps remaining
perturb_strength = 0;   % current perturbation magnitude
perturb_flag = 0;       % current perturbation activation

%% ================== SIMULATION LOOP ==================
for k = 1:steps
    t = k*dt;
    
    % ---------------- PID CONTROL ----------------
    obs_noise = enable_obs_noise * sqrt(deg2rad(0.001)) * randn;
    error = target_theta - (state(1)+obs_noise);
    [u, pid] = pid.computeControl(error, dt);
    u = clip(u, 0, 3.2);
    
    % ---------------- DYNAMICS INTEGRATION ----------------
    state = state + dt*dynamics(state,u);
    
    % ---------------- PERTURBATION ----------------
    if enable_perturbation
        % Start a new perturbation if timer expired
        if perturb_timer <= 0 && rand < 0.01  % chance to start
            perturb_timer = randi([10,100]);    % duration in steps (0.1–1.0 s)
            perturb_strength = randn*0.1;      % force magnitude
        end
        if perturb_timer > 0
            state = state + dt*[0; L*perturb_strength/I_total];
            perturb_flag = 1;
            perturb_timer = perturb_timer - 1;
            if enable_animation
                perturb_plot.UData = perturb_strength*-sin(state(1));
                perturb_plot.VData = perturb_strength*cos(state(1));
            end
        else
            perturb_flag = 0;
            if enable_animation
                perturb_plot.UData = 0; perturb_plot.VData = 0;
            end
        end
    end
    if enable_animation
        perturb_plot.XData = L*cos(state(1));
        perturb_plot.YData = L*sin(state(1));
    end
    
    % ---------------- COLLISION ----------------
    if enable_collision
        restitution = 0.6;
        if state(1) > deg2rad(90)-fulcr_angle
            state(2) = state(2) - (1+restitution)*state(2)*(state(2)>0);
        elseif state(1) < deg2rad(-90)+fulcr_angle
            state(2) = state(2) - (1+restitution)*state(2)*(state(2)<0);
        end
    end
    
    % ---------------- ANIMATION UPDATE ----------------
    if enable_animation
        theta_deg = rad2deg(state(1));
        lever_plot.Shape = rotate(lever_poly,theta_deg,[0 0]);
        motor_plot.Shape = rotate(motor_poly,theta_deg,[0 0]);
        imuss_plot.Shape = rotate(imuss_poly,theta_deg,[0 0]);
        propl_plot.Shape = rotate(propl_poly,theta_deg,[0 0]);
        title_handle.String = sprintf('Time = %.2f s, Input = %.2f A, Target = %.2f deg, Current = %.2f deg', ...
                                      t,u,rad2deg(target_theta),theta_deg);
        % drawnow limitrate;
        pause(0.001)
    end
    
    % ---------------- LOG DATA ----------------
    log_data(k,:) = [t, u, state(1), state(2), perturb_flag];
end

%% ================== PLOT TIME-SERIES ==================
figure('Color','w');
subplot(3,1,1); plot(log_data(:,1), log_data(:,3)*180/pi); ylabel('\theta [deg]'); grid on; title('Lever Angle');
subplot(3,1,2); plot(log_data(:,1), log_data(:,4)); ylabel('\theta dot [rad/s]'); grid on; title('Angular Velocity');
subplot(3,1,3); plot(log_data(:,1), log_data(:,2)); hold on;
area(log_data(:,1), log_data(:,5)*max(log_data(:,2)),'FaceColor','r','FaceAlpha',0.1,'EdgeColor','none');
ylabel('Input [A]'); xlabel('Time [s]'); grid on;
title('Control Input + Perturbation Events');
