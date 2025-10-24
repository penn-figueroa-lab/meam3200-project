function dx = dynamics(x, u)
    % dynamics - Computes the angular dynamics of the system
    %
    % Inputs:
    %   x : [theta; theta_dot]  -> angular position and velocity [rad, rad/s]
    %   u : control input (motor current)
    %
    % Output:
    %   dx : [theta_dot; theta_ddot]  -> state derivatives
    %
    % Globals required:
    %   I_total, lever2motor, b_bearing, K_T, K_tau, g,
    %   motor_weight, imuss_weight, lever2imuss

    % === Global parameters ===
    global I_total lever2motor b_bearing K g ...
           motor_weight imuss_weight lever2imuss

    % === State unpacking ===
    theta     = x(1);   % angular position [rad]
    theta_dot = x(2);   % angular velocity [rad/s]

    % === Geometry: Lever angles ===
    psi_motor = atan2(lever2motor(2), lever2motor(1));
    psi_imuss = atan2(lever2imuss(2), lever2imuss(1));

    % === Torques acting on the system ===
    tau_motor   = -motor_weight * g * norm(lever2motor) * cos(theta + psi_motor); % gravitational torque [N·m]
    tau_imuss   =  imuss_weight * g * norm(lever2imuss) * cos(theta - psi_imuss); % gravitational torque [N·m]
    tau_bearing = -b_bearing * theta_dot;                                         % bearing friction torque [N·m]
    tau_thrust  =  K * u * lever2motor(1);                                        % motor thrust torque [N·m]

    % === Dynamics equation ===
    theta_ddot = (tau_motor + tau_imuss + tau_bearing + tau_thrust) / I_total;    % angular acceleration [rad/s²]

    % === State derivative ===
    dx = [theta_dot; theta_ddot];
end

