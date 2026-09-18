function linearized_tf = linearizedTF()
    % linearizedTF - Constructs the linearized transfer function of the system.
    %
    % Output:
    %   linearized_tf : Linearized transfer function (tf object)
    %
    % Example:
    %   G = linearizedTF();
    %   step(G);    % Visualize step response

    % === Global parameters ===
    global K L I_total b_bearing g m_motor m_imuss lever2motor lever2imuss

    % Define Laplace variable
    s = tf('s');

    % === TODO ===
    linearized_tf = s;
end
