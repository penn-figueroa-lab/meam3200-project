function dx = dynamics(x, u)
    % dynamics - Computes the angular dynamics of the system
    %
    % Inputs:
    %   x : [theta; theta_dot]  -> angular position and velocity [rad, rad/s]
    %   u : control input (motor current)
    %
    % Output:
    %   dx : [theta_dot; theta_ddot]  -> state derivatives

    % === Global parameters ===
    global I_total lever2motor b_bearing K g ...
           m_motor m_imuss lever2imuss

    % === State unpacking ===
    theta     = x(1);   % angular position [rad]
    theta_dot = x(2);   % angular velocity [rad/s]

    % === TODO ===
    theta_ddot = 0;    % angular acceleration [rad/s²]

    % === State derivative ===
    dx = [theta_dot; theta_ddot];
end

