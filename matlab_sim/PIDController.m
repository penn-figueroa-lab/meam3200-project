classdef PIDController
    % PIDController - Simple Proportional-Integral-Derivative controller class
    %
    % Example usage:
    %   pid = PIDController(1.0, 0.1, 0.05);
    %   [u, pid] = pid.computeControl(error, dt);
    %
    % Students: Complete the sections marked with "TODO".

    properties
        % PID gains
        Kp = 0;   % Proportional gain
        Ki = 0;   % Integral gain
        Kd = 0;   % Derivative gain

        % Internal states
        integral = 0;         % Integrated error
        previousError = 0;    % Error from previous step
        startFlag = false;    % Indicates first iteration
    end

    methods
        function obj = PIDController(Kp, Ki, Kd)
            % Constructor: initialize PID gains
            if nargin == 3
                obj.Kp = Kp;
                obj.Ki = Ki;
                obj.Kd = Kd;
            end
        end

        function runTuner(obj, sys_tf)
            % Launch MATLAB PID tuner for given transfer function
            pidTuner(sys_tf, 'pid');
        end

        function obj = setGains(obj, Kp, Ki, Kd)
            % Set PID gains manually
            obj.Kp = Kp;
            obj.Ki = Ki;
            obj.Kd = Kd;
        end

        function [u, obj] = computeControl(obj, error, dt)
            % Compute control output using PID formula
            %
            % Inputs:
            %   error - current control error
            %   dt    - time step
            %
            % Outputs:
            %   u     - control effort
            %   obj   - updated PID object

            % === TODO: Implement the integral term ===
            obj.integral = 0;

            % === TODO: Implement the derivative term ===
            if ~obj.startFlag
                derivative = 0;
                obj.startFlag = true;
            else
                derivative = 0;
            end

            % === TODO: Implement the PID control law ===
            u = 0;

            % Store current error
            obj.previousError = error;
        end

        function obj = reset(obj)
            % Reset all internal states (but keep gains)
            obj.integral = 0;
            obj.previousError = 0;
            obj.startFlag = false;
        end
    end
end