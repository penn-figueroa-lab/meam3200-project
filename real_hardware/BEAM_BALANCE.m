port = "COM4";      
baud = 115200;
dt = 0.05;         
run_time = 30;
N_steps = round(run_time/dt);

Kp = 1;   
Ki = 0;    
Kd = 0;    
    
pid = PIDController(Kp, Ki, Kd);

throttle_mid = 1050;   
tmin = 1000;
tmax = 2000;
max_step = 15;         
throttle = throttle_mid;

% ---- Angle settings ----
theta_fail = 30 * pi/180;   % 30 degrees (kept for later)
theta_ref  = 0;             % rad, level is the target

%% ======================= SERIAL SETUP =============================
s = serialport(port, baud);
pause(1);

while s.NumBytesAvailable > 0
    readline(s);
end

disp("Hold your beam LEVEL for calibration...");

%% ======================= ACCEL ZERO CALIBRATION ===================
N = 200;
accel_sum = 0;

for i = 1:N
    line = readline(s);
    vals = str2num(line); 
    if numel(vals) ~= 3
        continue;
    end
    ax = vals(1);
    az = vals(2);

    accel_angle = atan2(ax, az);
    accel_sum   = accel_sum + accel_angle;
end

theta_offset = accel_sum / N;
disp("Calibration complete.");

%% ======================= KALMAN SETUP =============================
angle = 0;
bias  = 0;
P = zeros(2);

Q_angle   = 0.001;
Q_bias    = 0.003;
R_measure = 0.03;

disp("Starting control loop...");

%% ======================= LOGGING ARRAYS ===========================
t_hist       = zeros(N_steps,1);
theta_hist   = zeros(N_steps,1);   % Kalman angle (rad)
error_hist   = zeros(N_steps,1);
throttle_hist= zeros(N_steps,1);
u_hist       = zeros(N_steps,1);   % raw PID effort

%% ======================= MAIN LOOP ================================
for k = 1:N_steps
    t_hist(k) = (k-1)*dt;

    %% ---- 1. READ SERIAL ----
    line = readline(s);
    vals = str2num(line); 
    if numel(vals) ~= 3
        pause(dt);
        continue;
    end

    ax = vals(1);
    az = vals(2);
    gy = vals(3);

    %% ---- 2. ACCEL ANGLE ----
    accel_angle = atan2(ax, az) - theta_offset;

    %% ---- 3. KALMAN FILTER ----
    rate       = gy - bias;
    angle_pred = angle + dt * rate;

    P00 = P(1,1); P01 = P(1,2);
    P10 = P(2,1); P11 = P(2,2);

    P(1,1) = P00 + dt*(dt*P11 - P01 - P10) + Q_angle;
    P(1,2) = P01 - dt*P11;
    P(2,1) = P10 - dt*P11;
    P(2,2) = P11 + Q_bias;

    S  = P(1,1) + R_measure;
    K0 = P(1,1) / S;
    K1 = P(2,1) / S;

    y     = accel_angle - angle_pred;
    angle = angle_pred + K0 * y;
    bias  = bias + K1 * y;

    P00 = P(1,1); P01 = P(1,2);
    P10 = P(2,1); P11 = P(2,2);

    P(1,1) = P00 - K0*P00;
    P(1,2) = P01 - K0*P01;
    P(2,1) = P10 - K1*P00;
    P(2,2) = P11 - K1*P01;

    theta = angle;   % rename for clarity


    %% ---- 4. FAILSAFE (kept commented for now) ----
    % if abs(theta) > theta_fail
    %     throttle = tmin;
    %     writeline(s, num2str(throttle));
    %     fprintf("FAILSAFE! θ = %.1f deg\n", theta*180/pi);
    %     pause(dt);
    %     continue;
    % end

    %% ---- 5. PID CONTROL ----
    error = theta_ref - theta;
    [u, pid] = pid.computeControl(error, dt);

    throttle_cmd = throttle_mid + u;

    % optional hard clamp before slew (you can uncomment later)
    % if throttle_cmd < tmin
    %     throttle_cmd = tmin;
    % elseif throttle_cmd > tmax
    %     throttle_cmd = tmax;
    % end

    %% ---- 6. SLEW RATE LIMIT ----
    delta = throttle_cmd - throttle;

    if delta > max_step
        delta = max_step;
    elseif delta < -max_step
        delta = -max_step;
    end

    throttle = throttle + delta;

    %% ---- 7. SEND TO ARDUINO ----
    writeline(s, num2str(round(throttle)));

    %% ---- 8. SAVE TO LOGS ----
    theta_hist(k)    = theta;
    error_hist(k)    = error;
    throttle_hist(k) = throttle;
    u_hist(k)        = u;

    %% ---- 9. DEBUG PRINT ----
    fprintf("θ=%6.2f deg | err=%6.2f | thr=%4d | u=%6.2f\n", ...
            theta*180/pi, error, round(throttle), u);

    pause(dt);
end

%% ======================= SHUTDOWN & PLOTS =========================
% Cut throttle to idle when run ends
writeline(s, num2str(tmin));
disp("Run complete. Generating plots...");

% Convert angle to deg for plotting
theta_deg_hist = theta_hist * 180/pi;

figure;
subplot(3,1,1);
plot(t_hist, theta_deg_hist, 'LineWidth', 1.5);
grid on;
ylabel('\theta (deg)');
title('Beam Angle vs Time');

subplot(3,1,2);
plot(t_hist, throttle_hist, 'LineWidth', 1.5);
grid on;
ylabel('Throttle (\mus)');

subplot(3,1,3);
plot(t_hist, u_hist, 'LineWidth', 1.5);
grid on;
ylabel('u (PID output)');
xlabel('Time (s)');
title('PID Control Effort');

%% ======================= EXPORT TO EXCEL ==========================
% Combine all logged data into a table
T = table( ...
    t_hist, ...
    theta_deg_hist, ...
    throttle_hist, ...
    u_hist, ...
    'VariableNames', {'time_s','theta_deg','throttle_us','pid_output'});

% Write to Excel file in the current folder
filename = 'balancing_run_log.xlsx';   % change name per run if you want
writetable(T, filename);

disp(['Data saved to ', filename]);
