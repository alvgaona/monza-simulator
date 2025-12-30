clear; close all; clc;

set(groot, 'defaultTextInterpreter', 'latex');
set(groot, 'defaultLegendInterpreter', 'latex');
set(groot, 'defaultAxesTickLabelInterpreter', 'latex');
set(groot, 'defaultFigureColor', 'white');
set(groot, 'defaultAxesColor', 'white');

%% Difficulty Level: 'very_easy', 'easy', 'medium', 'hard', 'very_hard'
difficulty = 'hard';

%% Create Platform
geom = MonzaGeometry();
m = MonzaPlatform(geom);

% Set difficulty level
fprintf('Difficulty level: %s\n', difficulty);
original_floor_x_ranges = m.setDifficulty(difficulty);

%% Initial Conditions - Start on floor 1
x0_R = 0.03;      % Start at center of floor 1
vx0_R = 0.0;      % Zero velocity
theta0 = 0.0;     % No initial tilt

% Set platform state
m.x_R = x0_R;
m.vx_R = vx0_R;
m.y_R = geom.k * x0_R^2 + geom.C_tracks(1);
m.vy_R = 2 * geom.k * x0_R * vx0_R;
m.geometry.theta = theta0;

[m.x_I, m.y_I, m.vx_I, m.vy_I] = m.rotating_to_inertial(m.x_R, m.y_R, m.vx_R, m.vy_R);

m.on_floor = true;
m.current_floor = 1;

fprintf('Initial state on floor %d:\n', m.current_floor);
fprintf('  x_R = %.3f m, vx_R = %.3f m/s\n\n', m.x_R, m.vx_R);

%% Setup MPC Controllers for each floor
fprintf('Setting up MPC controllers for all floors...\n');

num_floors = length(geom.C_tracks);
mpc_controllers = cell(num_floors, 1);
goals = zeros(3, num_floors);

for floor_idx = 1:num_floors
    x_min = geom.floor_x_ranges(floor_idx, 1);
    x_max = geom.floor_x_ranges(floor_idx, 2);

    x_min_orig = original_floor_x_ranges(floor_idx, 1);
    x_max_orig = original_floor_x_ranges(floor_idx, 2);

    % Floor edge interpretation (using ORIGINAL values):
    % x_min = -inf: ball can fall off left edge (no path to next floor)
    % x_max = inf: ball can fall off right edge (no path to next floor)
    % x_min = finite: there's a path to next floor at left edge (TARGET)
    % x_max = finite: there's a path to next floor at right edge (TARGET)
    if floor_idx < num_floors
        % Check which edge has a path to next floor (finite value in ORIGINAL)
        left_has_path = isfinite(x_min_orig);
        right_has_path = isfinite(x_max_orig);

        if left_has_path && ~right_has_path
            % Path to next floor is at left edge
            % Don't constrain left edge (allow ball to reach it and beyond)
            mpc_x_min = -inf;
            mpc_x_max = inf;  % Right side can fall off anyway
            % Goal: reach left edge with leftward velocity
            goal_x = x_min + 0.01;  % Just past the transition point
            goal_vx = -0.3;
            Q_vxR = 5.0;
        elseif right_has_path && ~left_has_path
            % Path to next floor is at right edge
            % Don't constrain right edge (allow ball to reach it and beyond)
            mpc_x_min = -inf;   % Left side can fall off anyway
            mpc_x_max = inf;
            % Goal: reach right edge with rightward velocity
            goal_x = x_max - 0.01;  % Just before the transition point
            goal_vx = 0.3;
            Q_vxR = 5.0;
        elseif left_has_path && right_has_path
            % Both edges have paths - need to decide which one to use
            % This shouldn't happen in normal geometry, but handle it
            % Default to left edge
            mpc_x_min = -inf;
            mpc_x_max = inf;
            goal_x = x_min + 0.01;
            goal_vx = -0.3;
            Q_vxR = 5.0;
        else
            error('Floor %d has no path to next floor (both edges are inf)!', floor_idx);
        end
    else
        % Last floor
        mpc_x_min = x_min;
        mpc_x_max = inf;
        goal_x = x_max + 0.01;
        goal_vx = 1.0;
        Q_vxR = 5.0;
    end

    % Create MPC controller
    mpc_controllers{floor_idx} = setup_mpc(m, ...
        'x_min', mpc_x_min, ...
        'x_max', mpc_x_max, ...
        'boundary_margin', 0.02, ...
        'Q_xR', 10.0, ...
        'Q_vxR', Q_vxR, ...
        'Q_theta', 0.0);

    % Store goal
    goals(:, floor_idx) = [goal_x; goal_vx; 0.0];

    % Debug output
    fprintf('Floor %d: orig=[%.3f, %.3f], actual=[%.3f, %.3f], goal_x=%.3f, goal_vx=%.2f\n', ...
        floor_idx, x_min_orig, x_max_orig, x_min, x_max, goal_x, goal_vx);
end

fprintf('MPC controllers ready for %d floors!\n\n', num_floors);

%% Simulation Parameters
Tsim = 10;        % Simulation time [s]
Ts = m.Ts;        % Sample time
N_sim = round(Tsim/Ts);

%% Storage
x_history = zeros(3, N_sim+1);
u_history = zeros(1, N_sim);
y_history = zeros(2, N_sim+1);
floor_history = zeros(1, N_sim+1);
disturbance_history = zeros(1, N_sim);
noise_history = zeros(3, N_sim);
mpc_time_history = zeros(1, N_sim);  % Control loop timing [s]

x_history(:, 1) = [m.x_R; m.vx_R; m.geometry.theta];
y_history(:, 1) = [m.x_I; m.y_I];
floor_history(1) = m.current_floor;

%% Disturbance Parameters
% Disturbance type: 'none', 'step', 'sinusoidal'
disturbance_type = 'sinusoidal';

% Step disturbance parameters (per floor at center)
step_amplitude = 1.0;         % rad/s
step_duration = 0.5;          % seconds (how long the disturbance lasts)
center_tolerance = 0.02;      % meters (how close to center triggers disturbance)

% Sinusoidal disturbance parameters
sin_amplitude = 0.75;          % rad/s (low amplitude)
sin_frequency = 2.0;          % Hz

% Measurement noise parameters (Gaussian noise on state measurements)
enable_measurement_noise = true;
noise_std_x_R = 0.002;        % m (position noise std dev)
noise_std_vx_R = 0.01;        % m/s (velocity noise std dev)
noise_std_theta = 0.001;      % rad (angle noise std dev)

% Track which floors have been disturbed (for step disturbance)
disturbed_floors = false(num_floors, 1);
disturbance_active = false;
disturbance_end_time = 0;

%% Main Simulation Loop
fprintf('Running multi-floor MPC simulation...\n');
fprintf('Disturbance type: %s\n', disturbance_type);
if enable_measurement_noise
    fprintf('Measurement noise: ENABLED (σ_x=%.1f mm, σ_vx=%.1f mm/s, σ_θ=%.2f mdeg)\n', ...
        noise_std_x_R*1000, noise_std_vx_R*1000, noise_std_theta*180/pi*1000);
else
    fprintf('Measurement noise: DISABLED\n');
end

for i = 1:N_sim
    if m.on_floor
        % Ball is on a floor - use MPC control
        x_current = [m.x_R; m.vx_R; m.geometry.theta];

        % Add measurement noise
        measurement_noise = zeros(3, 1);
        if enable_measurement_noise
            measurement_noise = [
                noise_std_x_R * randn();
                noise_std_vx_R * randn();
                noise_std_theta * randn()
            ];
        end
        x_measured = x_current + measurement_noise;

        % Select MPC controller and reference based on current floor
        if m.current_floor >= 1 && m.current_floor <= num_floors
            nlobj = mpc_controllers{m.current_floor};
            ref = goals(:, m.current_floor)';
        else
            error('Unexpected floor number: %d', m.current_floor);
        end

        % MPC optimization (uses noisy measurements)
        tic_mpc = tic;
        [u_optimal, info] = nlmpcmove(nlobj, x_measured, 0, ref, []);
        mpc_time_history(i) = toc(tic_mpc);
        u_current = u_optimal(1);
    else
        % Ball is in the air - no control (coast with zero omega)
        u_current = 0;
        measurement_noise = zeros(3, 1);
    end

    % Apply disturbance based on type
    u_disturbance = 0;

    switch disturbance_type
        case 'step'
            % Step disturbance when ball reaches center of each floor
            if m.on_floor && m.current_floor >= 1 && m.current_floor <= num_floors
                % Calculate center of current floor
                x_min = geom.floor_x_ranges(m.current_floor, 1);
                x_max = geom.floor_x_ranges(m.current_floor, 2);
                x_center = (x_min + x_max) / 2;

                % Check if ball is near center and hasn't been disturbed on this floor yet
                if abs(m.x_R - x_center) < center_tolerance && ~disturbed_floors(m.current_floor)
                    % Trigger disturbance
                    disturbance_active = true;
                    disturbance_end_time = m.t + step_duration;
                    disturbed_floors(m.current_floor) = true;
                    fprintf('  Disturbance triggered at floor %d center (x_R=%.3f)\n', m.current_floor, m.x_R);
                end

                % Apply disturbance if active
                if disturbance_active && m.t < disturbance_end_time
                    u_disturbance = step_amplitude;
                elseif m.t >= disturbance_end_time
                    disturbance_active = false;
                end
            end

        case 'sinusoidal'
            % Continuous sinusoidal disturbance (only when on floor)
            if m.on_floor
                u_disturbance = sin_amplitude * sin(2 * pi * sin_frequency * m.t);
            end

        case 'none'
            % No disturbance
            u_disturbance = 0;

        otherwise
            error('Unknown disturbance type: %s', disturbance_type);
    end

    % Apply control with disturbance and step forward
    m.step(u_current + u_disturbance);

    % Update history
    u_history(i) = u_current;
    disturbance_history(i) = u_disturbance;
    noise_history(:, i) = measurement_noise;
    x_history(:,i+1) = [m.x_R; m.vx_R; m.geometry.theta];
    y_history(:,i+1) = [m.x_I; m.y_I];
    floor_history(i+1) = m.on_floor * m.current_floor;  % 0 if in air

    % Check if ball has fallen
    if m.has_fallen
        if ~exist('fall_time', 'var')
            fall_time = m.t;  % Record when the ball fell
            fall_x_R = m.x_R;  % Record x position when fell

            % Check if ball fell to the right of the last floor (game won!)
            last_floor_x_max = geom.floor_x_ranges(num_floors, 2);
            if fall_x_R > last_floor_x_max - 0.1
                fprintf('\n*** GAME WON! Ball fell to the right of floor %d at t=%.2f s ***\n', num_floors, fall_time);
                game_won = true;

                % Trim arrays and stop immediately when goal is reached
                x_history = x_history(:, 1:i+1);
                u_history = u_history(1:i);
                y_history = y_history(:, 1:i+1);
                floor_history = floor_history(1:i+1);
                disturbance_history = disturbance_history(1:i);
                noise_history = noise_history(:, 1:i);
                mpc_time_history = mpc_time_history(1:i);
                break;
            else
                fprintf('\n*** Ball fell outside platform at t=%.2f s ***\n', fall_time);
                game_won = false;
            end
        end

        % Continue for 2 seconds after failing (not winning)
        if ~game_won && m.t - fall_time >= 2.0
            % Trim arrays
            x_history = x_history(:, 1:i+1);
            u_history = u_history(1:i);
            y_history = y_history(:, 1:i+1);
            floor_history = floor_history(1:i+1);
            disturbance_history = disturbance_history(1:i);
            noise_history = noise_history(:, 1:i);
            mpc_time_history = mpc_time_history(1:i);
            break;
        end
    end

    % Progress info
    if m.on_floor && mod(i, 20) == 0
        fprintf('t=%.2f s: Floor %d, x_R=%.3f m, vx_R=%.3f m/s\n', ...
            m.t, m.current_floor, m.x_R, m.vx_R);
    end
end

fprintf('\nSimulation complete!\n');

%% Control Loop Timing Statistics
% Filter out zero entries (when ball was in air)
% mpc_times_active = mpc_time_history(mpc_time_history > 0);
% if ~isempty(mpc_times_active)
%     fprintf('\n--- Control Loop Timing Statistics ---\n');
%     fprintf('  Mean:   %.2f ms\n', mean(mpc_times_active) * 1000);
%     fprintf('  Std:    %.2f ms\n', std(mpc_times_active) * 1000);
%     fprintf('  Min:    %.2f ms\n', min(mpc_times_active) * 1000);
%     fprintf('  Max:    %.2f ms\n', max(mpc_times_active) * 1000);
%     fprintf('  Median: %.2f ms\n', median(mpc_times_active) * 1000);
%     fprintf('  Sample time (Ts): %.2f ms\n', Ts * 1000);
%     if mean(mpc_times_active) > Ts
%         fprintf('  WARNING: Mean MPC time exceeds sample time!\n');
%     else
%         fprintf('  Real-time margin: %.1f%%\n', (1 - mean(mpc_times_active)/Ts) * 100);
%     end
%     fprintf('--------------------------------------\n\n');
% end

%% Plot Results
t = 0:Ts:(length(u_history))*Ts;

fig = figure('Position', [100, 100, 1600, 900]);
movegui(fig, 'center');

% State x_R
subplot(3,3,1);
plot(t, x_history(1, :), 'b-', 'LineWidth', 2);
xlabel('Time [s]');
ylabel('$x_R$ [m]');
title('Position on Floor (Rotating Frame)');
grid on;

% State x_Rdot
subplot(3,3,2);
plot(t, x_history(2, :), 'r-', 'LineWidth', 2);
xlabel('Time [s]');
ylabel('$\dot{x}_R$ [m/s]');
title('Velocity on Floor');
grid on;

% State theta
subplot(3,3,3);
plot(t, x_history(3, :)*180/pi, 'g-', 'LineWidth', 2);
xlabel('Time [s]');
ylabel('$\theta$ [deg]');
title('Platform Angle');
grid on;

% Control omega
subplot(3,3,4);
stairs(t(1:end-1), u_history, 'k-', 'LineWidth', 2);
xlabel('Time [s]');
ylabel('$\omega$ [rad/s]');
title('Control Input (Angular Velocity)');
grid on;

% Disturbance
subplot(3,3,9);
stairs(t(1:end-1), disturbance_history, 'm-', 'LineWidth', 2);
xlabel('Time [s]');
ylabel('$\omega_{dist}$ [rad/s]');
title(sprintf('Disturbance (%s)', disturbance_type));
grid on;

% Floor indicator
subplot(3,3,5);
stairs(t, floor_history, 'LineWidth', 2);
xlabel('Time [s]');
ylabel('Floor Number');
title('Current Floor (0 = in air)');
grid on;
ylim([-0.5, num_floors+0.5]);

% Output x_I
subplot(3,3,6);
plot(t, y_history(1, :), 'b-', 'LineWidth', 2);
xlabel('Time [s]');
ylabel('$x_I$ [m]');
title('X Position (Inertial Frame)');
grid on;

% Output y_I
subplot(3,3,7);
plot(t, y_history(2, :), 'r-', 'LineWidth', 2);
hold on;
% Plot floor heights
for floor_idx = 1:num_floors
    plot([0, t(end)], [geom.C_tracks(floor_idx), geom.C_tracks(floor_idx)], ...
        '--', 'LineWidth', 1);
end
xlabel('Time [s]');
ylabel('$y_I$ [m]');
title('Y Position (Inertial Frame)');
grid on;

% Trajectory in inertial frame
subplot(3,3,8);
plot(y_history(1, :), y_history(2, :), 'b-', 'LineWidth', 2);
hold on;
plot(y_history(1, 1), y_history(2, 1), 'go', 'MarkerSize', 10, 'MarkerFaceColor', 'g');

% Draw all floors at different angles
colors = lines(num_floors);
for floor_idx = 1:num_floors
    x_floor = linspace(geom.floor_x_ranges(floor_idx,1), geom.floor_x_ranges(floor_idx,2), 100);
    for theta_snap = linspace(0, 2*pi, 8)
        y_floor = geom.k*x_floor.^2 + geom.C_tracks(floor_idx);
        x_rot = x_floor*cos(theta_snap) - y_floor*sin(theta_snap);
        y_rot = x_floor*sin(theta_snap) + y_floor*cos(theta_snap);
        plot(x_rot, y_rot, '--', 'LineWidth', 0.5, 'Color', colors(floor_idx,:)*0.7 + 0.3);
    end
end

axis equal;
xlabel('$x_I$ [m]');
ylabel('$y_I$ [m]');
title('Trajectory (Inertial Frame)');
grid on;

sgtitle('MPC Control - Multi-Floor (8 Floors)');

fprintf('Plot complete!\n');

%% Plot Measurement Noise (if enabled)
if enable_measurement_noise
    fig_noise = figure('Position', [150, 150, 1200, 600]);
    movegui(fig_noise, 'center');

    % Position noise
    subplot(2,3,1);
    plot(t(1:end-1), noise_history(1,:)*1000, 'b-', 'LineWidth', 1);
    xlabel('Time [s]');
    ylabel('Noise [mm]');
    title('Position Measurement Noise ($x_R$)');
    grid on;

    % Velocity noise
    subplot(2,3,2);
    plot(t(1:end-1), noise_history(2,:)*1000, 'r-', 'LineWidth', 1);
    xlabel('Time [s]');
    ylabel('Noise [mm/s]');
    title('Velocity Measurement Noise ($\dot{x}_R$)');
    grid on;

    % Angle noise
    subplot(2,3,3);
    plot(t(1:end-1), noise_history(3,:)*180/pi*1000, 'g-', 'LineWidth', 1);
    xlabel('Time [s]');
    ylabel('Noise [mdeg]');
    title('Angle Measurement Noise ($\theta$)');
    grid on;

    % Noise histograms
    subplot(2,3,4);
    histogram(noise_history(1,:)*1000, 30, 'FaceColor', 'b');
    xlabel('Noise [mm]');
    ylabel('Count');
    title(sprintf('Position Noise Dist ($\\sigma$=%.1f mm)', noise_std_x_R*1000));
    grid on;

    subplot(2,3,5);
    histogram(noise_history(2,:)*1000, 30, 'FaceColor', 'r');
    xlabel('Noise [mm/s]');
    ylabel('Count');
    title(sprintf('Velocity Noise Dist ($\\sigma$=%.1f mm/s)', noise_std_vx_R*1000));
    grid on;

    subplot(2,3,6);
    histogram(noise_history(3,:)*180/pi*1000, 30, 'FaceColor', 'g');
    xlabel('Noise [mdeg]');
    ylabel('Count');
    title(sprintf('Angle Noise Dist ($\\sigma$=%.2f mdeg)', noise_std_theta*180/pi*1000));
    grid on;

    sgtitle('Measurement Noise Analysis');
    fprintf('Noise plot complete!\n');
end

%% Plot MPC Timing
if ~isempty(mpc_times_active)
    fig_timing = figure('Position', [200, 200, 1000, 400]);
    movegui(fig_timing, 'center');

    % Time series of MPC computation time
    subplot(1,2,1);
    plot(t(1:end-1), mpc_time_history * 1000, 'b-', 'LineWidth', 1);
    hold on;
    yline(Ts * 1000, 'r--', 'LineWidth', 2, 'Label', 'Sample Time');
    yline(mean(mpc_times_active) * 1000, 'g--', 'LineWidth', 1.5, 'Label', 'Mean');
    xlabel('Time [s]');
    ylabel('Computation Time [ms]');
    title('MPC Computation Time per Step');
    legend('MPC Time', 'Sample Time', 'Mean', 'Location', 'best');
    grid on;

    % Histogram of MPC computation times
    subplot(1,2,2);
    histogram(mpc_times_active * 1000, 30, 'FaceColor', 'b');
    hold on;
    xline(Ts * 1000, 'r--', 'LineWidth', 2, 'Label', 'Sample Time');
    xline(mean(mpc_times_active) * 1000, 'g--', 'LineWidth', 1.5, 'Label', 'Mean');
    xlabel('Computation Time [ms]');
    ylabel('Count');
    title(sprintf('MPC Time Distribution (mean=%.2f ms, $\\sigma$=%.2f ms)', ...
        mean(mpc_times_active) * 1000, std(mpc_times_active) * 1000));
    legend('Distribution', 'Sample Time', 'Mean', 'Location', 'best');
    grid on;

    sgtitle('Control Loop Timing Analysis');
    fprintf('Timing plot complete!\n');
end

%% Animation
fprintf('\nCreating real-time animation...\n');

fig_anim = figure('Position', [200, 200, 800, 800]);
movegui(fig_anim, 'center');

% Real-time animation: no frame skipping, pause for actual simulation time
for i = 1:length(t)
    clf;

    % Update platform state to frame i
    m.x_R = x_history(1, i);
    m.vx_R = x_history(2, i);
    m.geometry.theta = x_history(3, i);
    m.x_I = y_history(1, i);
    m.y_I = y_history(2, i);
    m.on_floor = (floor_history(i) > 0);
    if m.on_floor
        m.current_floor = floor_history(i);
    else
        m.current_floor = 0;
    end

    % Call animate method with trajectory history
    m.animate('trajectory', y_history(:, 1:i), ...
              'time', t(i), ...
              'xlim', [-0.5, 0.5], ...
              'ylim', [-0.5, 0.5]);

    drawnow;

    % Pause for the actual sample time to achieve real-time playback
    pause(Ts);
end

fprintf('Animation complete!\n');
