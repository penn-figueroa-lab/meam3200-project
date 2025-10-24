function linearized_tf = linearizedTF()
    % linearizedTF - Computes the linearized transfer function for the system.
    %
    % Uses global parameters for physical constants and geometry.
    % Students: Review how torque contributions are linearized and combined.
    %
    % Output:
    %   linearized_tf : Linearized transfer function (tf object)

    % === Global parameters ===
    global K L I_total b_bearing g motor_weight imuss_weight lever2motor lever2imuss

    % Define Laplace variable
    s = tf('s');

    % === Torque contributions due to motor and IMU ===
    % TODO: Verify torque direction signs and geometry before using in simulation
    motor_torque = -motor_weight * g * norm(lever2motor) * ...
        sin(atan2(lever2motor(2), lever2motor(1)));
    imuss_torque = imuss_weight * g * norm(lever2imuss) * ...
        sin(atan2(lever2imuss(2), lever2imuss(1)));

    % === Linearized Transfer Function ===
    linearized_tf = (K * L) / (I_total * s^2 + b_bearing * s + motor_torque + imuss_torque);
end
