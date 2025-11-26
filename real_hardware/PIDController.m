classdef PIDController
    properties
        Kp = 0;
        Ki = 0;
        Kd = 0;

        integral = 0;
        previousError = 0;
        startFlag = false;
    end

    methods
        function obj = PIDController(Kp, Ki, Kd)
            if nargin == 3
                obj.Kp = Kp;
                obj.Ki = Ki;
                obj.Kd = Kd;
            end
        end

        function obj = setGains(obj, Kp, Ki, Kd)
            obj.Kp = Kp;
            obj.Ki = Ki;
            obj.Kd = Kd;
        end

        function [u, obj] = computeControl(obj, error, dt)
            obj.integral = obj.integral + 0.5 * (error + obj.previousError) * dt;

            if ~obj.startFlag
                derivative = 0;
                obj.startFlag = true;
            else
                derivative = (error - obj.previousError) / dt;
            end

            u = obj.Kp * error + obj.Ki * obj.integral + obj.Kd * derivative;

            obj.previousError = error;
        end

        function obj = reset(obj)
            obj.integral = 0;
            obj.previousError = 0;
            obj.startFlag = false;
        end
    end
end
