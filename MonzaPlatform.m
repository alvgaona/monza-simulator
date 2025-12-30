classdef MonzaPlatform < handle
    % MonzaPlatform - Class for the Monza rotating platform system

    properties
        % Geometry
        geometry        % MonzaGeometry instance

        % Physical parameters
        g               % Gravitational acceleration [m/s^2]
        m               % Ball mass [kg]
        r_ball          % Ball radius [m]

        % Ball state (inertial frame)
        x_I          % Ball x position [m]
        y_I          % Ball y position [m]
        vx_I         % Ball x velocity [m/s]
        vy_I         % Ball y velocity [m/s]

        % Ball state (rotating frame)
        x_R          % Ball x position in rotating frame [m]
        y_R          % Ball y position in rotating frame [m]
        vx_R         % Ball x velocity in rotating frame [m/s]
        vy_R         % Ball y velocity in rotating frame [m/s]

        % Collision state
        on_floor                % true if ball is on a floor
        current_floor           % Floor number (1-N) if on_floor
        has_fallen              % true if ball fell outside inner circle

        % Collision tolerance
        collision_tolerance     % Distance tolerance for floor collision [m]

        % Time
        t                       % Current simulation time [s]
        Ts                      % Simulation sampling rate [s]
    end

    methods
        function obj = MonzaPlatform(geometry)
            % Constructor
            % geometry: MonzaGeometry instance (optional - creates default if not provided)

            if nargin < 1 || isempty(geometry)
                obj.geometry = MonzaGeometry();
            else
                obj.geometry = geometry;
            end

            obj.g = 9.8;
            obj.m = 0.027;                    % Default ball mass [kg]
            obj.r_ball = 0.015;               % Default ball radius [m]
            obj.collision_tolerance = 0.01;
            obj.Ts = 0.01;                    % Default sampling time [s]

            % Initialize ball state (inertial frame)
            obj.x_I = 0;
            obj.y_I = 0;
            obj.vx_I = 0;
            obj.vy_I = 0;

            % Initialize ball state (rotating frame)
            obj.x_R = 0;
            obj.y_R = 0;
            obj.vx_R = 0;
            obj.vy_R = 0;

            % Initialize collision state
            obj.on_floor = true;
            obj.current_floor = 1;
            obj.has_fallen = false;

            % Initialize time
            obj.t = 0;
        end

        function original_ranges = setDifficulty(obj, level)
            % Set the difficulty level by updating floor x-ranges
            %
            % Input:
            %   level - String: 'very_easy', 'easy', 'medium', 'hard', 'very_hard'
            %
            % Output:
            %   original_ranges - The original floor ranges before updateFloorRanges()
            %                     (needed for MPC controller setup logic)
            %
            % Usage:
            %   m = MonzaPlatform();
            %   original_ranges = m.setDifficulty('hard');

            % Get the difficulty level ranges
            original_ranges = MonzaGeometry.getDifficultyLevel(level);

            % Set them in geometry
            obj.geometry.floor_x_ranges = original_ranges;

            % Update ranges (replaces inf with computed intersections)
            obj.geometry.updateFloorRanges();
        end

        function step(obj, u)
            % Step the simulation forward by one time step
            %
            % Inputs:
            %   u - Angular velocity control input (omega) [rad/s]
            %
            % Updates both inertial and rotating frame states automatically.
            % Access states via: obj.x_I, obj.y_I, obj.vx_I, obj.vy_I (inertial)
            %                    obj.x_R, obj.y_R, obj.vx_R, obj.vy_R (rotating)

            if obj.on_floor
                % Ball is on a floor - use constrained dynamics
                x_current = [obj.x_R; obj.vx_R; obj.geometry.theta];
                x_next = obj.step_on_floor(x_current, u);

                % Extract new state in rotating frame
                obj.x_R = x_next(1);
                obj.vx_R = x_next(2);
                obj.geometry.theta = x_next(3);

                % Check if ball has left the floor edge
                floor_idx = obj.current_floor;
                x_min = obj.geometry.floor_x_ranges(floor_idx, 1);
                x_max = obj.geometry.floor_x_ranges(floor_idx, 2);

                % Velocity threshold to prevent oscillation at edge
                v_threshold = 0.05;  % m/s

                if (obj.x_R < x_min && obj.vx_R < -v_threshold) || ...
                   (obj.x_R > x_max && obj.vx_R > v_threshold)
                    % Ball has left the edge - switch to ballistic
                    obj.on_floor = false;
                    obj.current_floor = 0;

                    % Compute final position and velocity on edge
                    obj.y_R = obj.geometry.k * obj.x_R^2 + obj.geometry.C_tracks(floor_idx);
                    obj.vy_R = 2 * obj.geometry.k * obj.x_R * obj.vx_R;

                    % Update inertial frame state (with omega for correct velocity)
                    [obj.x_I, obj.y_I, obj.vx_I, obj.vy_I] = ...
                        obj.rotating_to_inertial(obj.x_R, obj.y_R, obj.vx_R, obj.vy_R, u);
                else
                    % Still on floor
                    % Compute y position and velocity on parabola
                    obj.y_R = obj.geometry.k * obj.x_R^2 + obj.geometry.C_tracks(floor_idx);
                    obj.vy_R = 2 * obj.geometry.k * obj.x_R * obj.vx_R;

                    % Update inertial frame state (with omega for correct velocity)
                    [obj.x_I, obj.y_I, obj.vx_I, obj.vy_I] = ...
                        obj.rotating_to_inertial(obj.x_R, obj.y_R, obj.vx_R, obj.vy_R, u);
                end
            else
                % Ball is in the air - use ballistic dynamics
                obj.step_ballistic(obj.Ts);
                obj.geometry.theta = obj.geometry.theta + u * obj.Ts;

                % Update rotating frame state (with omega for correct velocity)
                [obj.x_R, obj.y_R, obj.vx_R, obj.vy_R] = ...
                    obj.inertial_to_rotating(obj.x_I, obj.y_I, obj.vx_I, obj.vy_I, u);
            end

            % Check for collisions with floors
            obj.update_collision_state(obj.x_I, obj.y_I);

            % Update time
            obj.t = obj.t + obj.Ts;
        end

        function step_ballistic(obj, dt)
            % Ballistic flight dynamics (Euler integration)
            %
            % Free fall under gravity in inertial frame:
            %   ẍ = 0
            %   ÿ = -g
            %
            % Note: Mass cancels out in acceleration (F = ma => a = F/m)
            %       Mass will be needed when adding dissipative forces

            % Update velocities
            obj.vx_I = obj.vx_I + 0 * dt;
            obj.vy_I = obj.vy_I - obj.g * dt;

            % Update positions
            obj.x_I = obj.x_I + obj.vx_I * dt;
            obj.y_I = obj.y_I + obj.vy_I * dt;
        end

        function x_next = step_on_floor(obj, x_current, u)
            % On-floor dynamics (ODE45)
            %
            % Ball constrained to parabola y_R = k*x_R^2 + C in rotating frame
            % Dynamics: ẍ_R = [-g·sin(θ) - 2k·x_R·g·cos(θ) + ω²·x_R·(1 + 2k²·x_R² + 2k·C_k)] / (1 + 4k²·x_R²)
            %
            % Note: Mass cancels out (normal force adjusts to maintain constraint)
            %       Mass will be needed when adding friction forces
            
            floor_idx = obj.current_floor;
            params.g = obj.g;
            params.m = obj.m;
            params.k = obj.geometry.k;
            params.C_k = obj.geometry.C_tracks(floor_idx);

            % Simulate one step
            [~, x_traj] = ode45(@(t, x) obj.monza_state_fcn_wrapper(x, u, params), ...
                                [0, obj.Ts], x_current);
            x_next = x_traj(end, :)';
        end


        function update_collision_state(obj, x_I, y_I)
            % Update collision detection for ball at given position
            %
            % Inputs:
            %   x_I - Ball x position in inertial frame [m]
            %   y_I - Ball y position in inertial frame [m]
            %
            % Updates:
            %   obj.on_floor           - true ONLY if ball is in contact with a floor
            %   obj.current_floor      - Floor number ONLY when on_floor is true (0 otherwise)
            %   obj.has_fallen         - true ONLY if ball is outside inner circle
            %   obj.is_falling_between - true if ball is falling between floors
            %   obj.between_floors     - [lower_floor, upper_floor] when falling between
            %
            % States:
            %   - has_fallen=true: ball outside r_inner (lost)
            %   - on_floor=true: ball in contact with floor i
            %   - is_falling_between=true: ball falling between floors (not in contact)
            
            if obj.on_floor
                return;
            end

            % Check if ball is within the boundary
            % Ball center position
            r_center = sqrt(x_I^2 + y_I^2);
            % Ball has fallen if its edge goes beyond the inner radius
            if r_center + obj.r_ball > obj.geometry.r_inner
                obj.has_fallen = true;
                obj.on_floor = false;
                obj.current_floor = inf;
                return;
            end

           
            obj.has_fallen = false;

            [x_R, y_R, ~, ~] = obj.inertial_to_rotating(x_I, y_I, 0, 0);

            for i = 1:length(obj.geometry.C_tracks)
                if x_R < obj.geometry.floor_x_ranges(i, 1) || ...
                   x_R > obj.geometry.floor_x_ranges(i, 2)
                    continue;
                end

                % Check if on parabola surface (actual contact)
                % The ball surface touches the floor when the distance from
                % the ball center to the parabola equals the ball radius
                y_floor = obj.geometry.k * x_R^2 + obj.geometry.C_tracks(i);
                distance_to_floor = abs(y_R - y_floor);

                % Contact occurs when ball center is approximately r_ball above the floor
                if distance_to_floor < obj.r_ball + obj.collision_tolerance
                    obj.on_floor = true;
                    obj.current_floor = i;
                    return;
                end
            end
        end

        function xdot = monza_state_fcn_wrapper(~, x, u, params)
            k = params.k;
            C_k = params.C_k;
        
            x_R = x(1);
            x_Rdot = x(2);
            theta = x(3);
            omega = u;
        
            num = -params.g*sin(theta) - 2*k*x_R*params.g*cos(theta) + ...
                        omega^2*x_R*(1 + 2*k^2*x_R^2 + 2*k*C_k);
            den = 1 + 4*k^2*x_R^2;
        
            x_Rddot = num / den;
        
            xdot = [x_Rdot; x_Rddot; omega];
        end

        function y = monza_output_fcn_wrapper(~, x, ~, params)
            x_R = x(1);
            theta = x(3);

            k = params.k;
            C_k = params.C_k;
            y_R = k*x_R^2 + C_k;

            y = [
                x_R*cos(theta) - y_R*sin(theta);
                x_R*sin(theta) + y_R*cos(theta)
            ];
        end

        function [x_R, y_R, vx_R, vy_R] = inertial_to_rotating(obj, x_I, y_I, vx_I, vy_I, omega)
            % Transform state from inertial frame to rotating frame
            %
            % Inputs:
            %   x_I, y_I   - Position in inertial frame [m]
            %   vx_I, vy_I - Velocity in inertial frame [m/s]
            %   omega      - (optional) Angular velocity of platform [rad/s]
            %
            % Outputs:
            %   x_R, y_R   - Position in rotating frame [m]
            %   vx_R, vy_R - Velocity in rotating frame [m/s]
            %
            % If omega is provided, velocity transformation accounts for
            % the rotating frame's angular velocity contribution.

            if nargin < 6
                omega = 0;
            end

            theta = obj.geometry.theta;

            % Position transformation (pure rotation by -theta)
            x_R = x_I * cos(theta) + y_I * sin(theta);
            y_R = -x_I * sin(theta) + y_I * cos(theta);

            % Velocity transformation with omega correction
            % v_R = R(-theta) * v_I - omega × r_R
            vx_R = vx_I * cos(theta) + vy_I * sin(theta) + omega * y_R;
            vy_R = -vx_I * sin(theta) + vy_I * cos(theta) - omega * x_R;
        end

        function [x_I, y_I, vx_I, vy_I] = rotating_to_inertial(obj, x_R, y_R, vx_R, vy_R, omega)
            % Transform state from rotating frame to inertial frame
            %
            % Inputs:
            %   x_R, y_R   - Position in rotating frame [m]
            %   vx_R, vy_R - Velocity in rotating frame [m/s]
            %   omega      - (optional) Angular velocity of platform [rad/s]
            %
            % Outputs:
            %   x_I, y_I   - Position in inertial frame [m]
            %   vx_I, vy_I - Velocity in inertial frame [m/s]
            %
            % If omega is provided, velocity transformation accounts for
            % the rotating frame's angular velocity contribution.

            if nargin < 6
                omega = 0;
            end

            theta = obj.geometry.theta;

            % Position transformation (pure rotation)
            x_I = x_R * cos(theta) - y_R * sin(theta);
            y_I = x_R * sin(theta) + y_R * cos(theta);

            % Velocity transformation with omega correction
            % v_I = R(theta) * v_R + omega × r_I
            vx_I = vx_R * cos(theta) - vy_R * sin(theta) - omega * y_I;
            vy_I = vx_R * sin(theta) + vy_R * cos(theta) + omega * x_I;
        end

        function animate(obj, varargin)
            % Animate the current state of the platform
            %
            % Usage:
            %   obj.animate() - Basic animation with defaults
            %   obj.animate('trajectory', trajectory_history) - Show trajectory
            %   obj.animate('title', 'Custom Title') - Custom title
            %   obj.animate('xlim', [-0.5, 0.5], 'ylim', [-0.5, 0.5]) - Set axis limits
            %
            % Optional Name-Value Arguments:
            %   trajectory - [2 x N] array of [x_I; y_I] positions to plot as history
            %   title - Custom title string (default: auto-generated)
            %   time - Current time to display in title
            %   xlim - X-axis limits (default: [-0.5, 0.5])
            %   ylim - Y-axis limits (default: [-0.5, 0.5])

            % Parse optional arguments
            p = inputParser;
            addParameter(p, 'trajectory', [], @(x) isempty(x) || (isnumeric(x) && size(x,1)==2));
            addParameter(p, 'title', '', @ischar);
            addParameter(p, 'time', obj.t, @isnumeric);
            addParameter(p, 'xlim', [-0.5, 0.5], @isnumeric);
            addParameter(p, 'ylim', [-0.5, 0.5], @isnumeric);
            parse(p, varargin{:});

            trajectory = p.Results.trajectory;
            custom_title = p.Results.title;
            time_val = p.Results.time;
            x_lim = p.Results.xlim;
            y_lim = p.Results.ylim;

            % Clear and setup axes
            hold on;
            axis equal;
            grid on;

            % Get number of floors
            num_floors = length(obj.geometry.C_tracks);
            colors = lines(num_floors);

            % Draw all floors
            for floor_idx = 1:num_floors
                x_floor = linspace(obj.geometry.floor_x_ranges(floor_idx,1), ...
                                   obj.geometry.floor_x_ranges(floor_idx,2), 200);
                y_floor = obj.geometry.k * x_floor.^2 + obj.geometry.C_tracks(floor_idx);
                x_floor_rot = x_floor*cos(obj.geometry.theta) - y_floor*sin(obj.geometry.theta);
                y_floor_rot = x_floor*sin(obj.geometry.theta) + y_floor*cos(obj.geometry.theta);

                if obj.on_floor && floor_idx == obj.current_floor
                    % Highlight current floor
                    plot(x_floor_rot, y_floor_rot, '-', 'LineWidth', 4, 'Color', colors(floor_idx,:));
                else
                    plot(x_floor_rot, y_floor_rot, '-', 'LineWidth', 2, 'Color', colors(floor_idx,:)*0.5 + 0.5);
                end
            end

            % Compute ball position for visualization
            if obj.on_floor
                % Ball is on a floor - offset by r_ball along normal
                y_R_surf = obj.geometry.k * obj.x_R^2 + obj.geometry.C_tracks(obj.current_floor);

                % Normal vector to parabola
                dy_dx = 2 * obj.geometry.k * obj.x_R;
                normal_x = -dy_dx;
                normal_y = 1;
                normal_mag = sqrt(normal_x^2 + normal_y^2);
                normal_x = normal_x / normal_mag;
                normal_y = normal_y / normal_mag;

                % Offset ball center from surface
                x_R_vis = obj.x_R + obj.r_ball * normal_x;
                y_R_vis = y_R_surf + obj.r_ball * normal_y;

                % Transform to inertial frame
                x_ball_I = x_R_vis*cos(obj.geometry.theta) - y_R_vis*sin(obj.geometry.theta);
                y_ball_I = x_R_vis*sin(obj.geometry.theta) + y_R_vis*cos(obj.geometry.theta);
            else
                % Ball is in the air - use inertial position directly
                x_ball_I = obj.x_I;
                y_ball_I = obj.y_I;
            end

            % Draw ball as a circle
            theta_circle = linspace(0, 2*pi, 50);
            x_circle = x_ball_I + obj.r_ball * cos(theta_circle);
            y_circle = y_ball_I + obj.r_ball * sin(theta_circle);
            fill(x_circle, y_circle, 'g', 'EdgeColor', 'k', 'LineWidth', 1.5);
            plot(x_ball_I, y_ball_I, 'ko', 'MarkerSize', 4, 'MarkerFaceColor', 'k');

            % Draw center
            plot(0, 0, 'ko', 'MarkerSize', 5, 'MarkerFaceColor', 'k');

            % Draw inner and outer circles

            % Inner circle - draw only solid sections (rotates with platform)
            if ~isempty(obj.geometry.inner_solid_ranges)
                for i = 1:size(obj.geometry.inner_solid_ranges, 1)
                    theta_start = obj.geometry.inner_solid_ranges(i, 1);
                    theta_end = obj.geometry.inner_solid_ranges(i, 2);
                    theta_section = linspace(theta_start, theta_end, 50);
                    % Apply platform rotation
                    theta_rotated = theta_section + obj.geometry.theta;
                    x_inner = obj.geometry.r_inner * cos(theta_rotated);
                    y_inner = obj.geometry.r_inner * sin(theta_rotated);
                    plot(x_inner, y_inner, 'r-', 'LineWidth', 3);
                end
            else
                % No solid ranges defined - draw full circle as dashed (fixed in inertial frame)
                theta_circ = linspace(0, 2*pi, 100);
                x_inner = obj.geometry.r_inner * cos(theta_circ);
                y_inner = obj.geometry.r_inner * sin(theta_circ);
                plot(x_inner, y_inner, 'r--', 'LineWidth', 2);
            end

            % Outer circle (platform edge - fixed in inertial frame)
            theta_circ = linspace(0, 2*pi, 100);
            x_outer = obj.geometry.r_outer * cos(theta_circ);
            y_outer = obj.geometry.r_outer * sin(theta_circ);
            plot(x_outer, y_outer, 'k-', 'LineWidth', 3);

            % Draw trajectory history if provided
            if ~isempty(trajectory)
                plot(trajectory(1, :), trajectory(2, :), 'c-', 'LineWidth', 1.5, 'Color', [0 0.7 0.7]);
            end

            % Labels
            xlabel('$x_I$ [m]');
            ylabel('$y_I$ [m]');

            % Title
            if ~isempty(custom_title)
                title(custom_title);
            else
                if obj.on_floor
                    title({sprintf('t = %.2f s', time_val), ...
                           sprintf('Floor %d/%d, $x_R$ = %.3f m, $\\theta$ = %.1f deg', ...
                                  obj.current_floor, num_floors, obj.x_R, obj.geometry.theta*180/pi)});
                else
                    title({sprintf('t = %.2f s', time_val), ...
                           sprintf('BALLISTIC, $\\theta$ = %.1f deg', obj.geometry.theta*180/pi)});
                end
            end

            xlim(x_lim);
            ylim(y_lim);
        end
    end
end
