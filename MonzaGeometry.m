classdef MonzaGeometry < handle
    % MonzaGeometry - Class for Monza platform geometry and visualization

    properties
        % Geometric parameters
        k               % Parabolic curvature coefficient
        C_tracks        % Track vertical offsets [m]

        % Floor boundaries (each floor has its own range)
        floor_x_ranges  % Nx2 matrix: [x_min, x_max] for each floor

        % Circle radii
        r_inner         % Inner circle radius [m]
        r_outer         % Outer circle radius [m]

        % Inner circle solid sections (angular ranges in radians)
        inner_solid_ranges  % Nx2 matrix: [theta_start, theta_end] for solid sections

        % Platform rotation angle
        theta               % Current rotation angle [rad]
    end

    methods
        function obj = MonzaGeometry(options)
            % Constructor
            arguments
                options.k = -0.54
                options.C_tracks = [0.16 0.1143 0.0686 0.03 -0.03 -0.0686 -0.1143 -0.16]
                options.floor_x_ranges = []
            end

            obj.k = options.k;
            obj.C_tracks = options.C_tracks;

            % Circle radii
            obj.r_inner = 0.2;
            obj.r_outer = 0.23;

            % Initialize rotation angle
            obj.theta = 0;

            % Solid ranges of the inner circle (for visualization)
            obj.inner_solid_ranges = [
              deg2rad(-155), deg2rad(-140);
              deg2rad(-124), deg2rad(-115);
              deg2rad(-70), deg2rad(-39);
              deg2rad(-14), deg2rad(-20);
              deg2rad(8), deg2rad(14);
              deg2rad(50), deg2rad(85);
              deg2rad(95), deg2rad(151);
              deg2rad(177),  deg2rad(185);
            ];

            % Default floor x-ranges (can be customized)
            if ~isempty(options.floor_x_ranges)
                obj.floor_x_ranges = options.floor_x_ranges;
            else
                % Compute x-ranges based on intersection with inner circle
                n_floors = length(obj.C_tracks);
                obj.floor_x_ranges = zeros(n_floors, 2);

                for i = 1:n_floors
                    x_intersect = obj.computeIntersectionX(obj.C_tracks(i), obj.r_inner);
                    if ~isempty(x_intersect)
                        obj.floor_x_ranges(i, :) = [min(x_intersect), max(x_intersect)];
                    else
                        % No intersection - use default small range
                        obj.floor_x_ranges(i, :) = [-0.01, 0.01];
                    end
                end
            end

            % Replace inf values with computed intersections
            for i = 1:size(obj.floor_x_ranges, 1)
                x_intersect = obj.computeIntersectionX(obj.C_tracks(i), obj.r_inner);

                % Replace -inf in xmin with minimum intersection x
                if isinf(obj.floor_x_ranges(i, 1)) && obj.floor_x_ranges(i, 1) < 0
                    if ~isempty(x_intersect)
                        obj.floor_x_ranges(i, 1) = min(x_intersect);
                    else
                        obj.floor_x_ranges(i, 1) = -0.01;  % fallback
                    end
                end

                % Replace inf in xmax with maximum intersection x
                if isinf(obj.floor_x_ranges(i, 2)) && obj.floor_x_ranges(i, 2) > 0
                    if ~isempty(x_intersect)
                        obj.floor_x_ranges(i, 2) = max(x_intersect);
                    else
                        obj.floor_x_ranges(i, 2) = 0.01;  % fallback
                    end
                end
            end
        end

        function rotate(obj, angle_deg)
            % Set the rotation angle of the platform
            % angle_deg: rotation angle in degrees
            obj.theta = deg2rad(angle_deg);
        end

        function updateFloorRanges(obj)
            % Replace inf values in floor_x_ranges with computed intersections
            for i = 1:size(obj.floor_x_ranges, 1)
                x_intersect = obj.computeIntersectionX(obj.C_tracks(i), obj.r_inner);

                % Replace -inf in xmin with minimum intersection x
                if isinf(obj.floor_x_ranges(i, 1)) && obj.floor_x_ranges(i, 1) < 0
                    if ~isempty(x_intersect)
                        obj.floor_x_ranges(i, 1) = min(x_intersect);
                    else
                        obj.floor_x_ranges(i, 1) = -0.01;  % fallback
                    end
                end

                % Replace inf in xmax with maximum intersection x
                if isinf(obj.floor_x_ranges(i, 2)) && obj.floor_x_ranges(i, 2) > 0
                    if ~isempty(x_intersect)
                        obj.floor_x_ranges(i, 2) = max(x_intersect);
                    else
                        obj.floor_x_ranges(i, 2) = 0.01;  % fallback
                    end
                end
            end
        end

        function plot(obj, theta)
            % Plot floors with individual x-ranges
            % theta: optional rotation angle (default: use obj.theta)
            if nargin < 2
                theta = obj.theta;
            end

            figure('Position', [100, 100, 1000, 1000]);
            hold on;
            axis equal;
            grid on;
            xlabel('x [m]');
            ylabel('y [m]');
            title(sprintf('Monza Platform Geometry (\\theta = %.1f deg)', theta*180/pi));

            colors = lines(length(obj.C_tracks));

            % Plot each floor with its own x-range
            for i = 1:length(obj.C_tracks)
                C_k = obj.C_tracks(i);
                x_min_i = obj.floor_x_ranges(i, 1);
                x_max_i = obj.floor_x_ranges(i, 2);

                % Generate points only within this floor's range
                x_range = linspace(x_min_i, x_max_i, 200);
                y_track = obj.k * x_range.^2 + C_k;

                % Apply rotation (from rotating frame to inertial frame)
                x_rot = x_range*cos(theta) - y_track*sin(theta);
                y_rot = x_range*sin(theta) + y_track*cos(theta);

                % Plot floor
                plot(x_rot, y_rot, 'Color', colors(i,:), 'LineWidth', 3, ...
                     'DisplayName', sprintf('Floor %d', i));

                % Mark endpoints to show floor boundaries
                plot(x_rot([1, end]), y_rot([1, end]), 'o', ...
                     'Color', colors(i,:), 'MarkerSize', 8, 'MarkerFaceColor', colors(i,:), ...
                     'HandleVisibility', 'off');
            end

            % Add center point
            plot(0, 0, 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k', 'HandleVisibility', 'off');

            % Plot circles

            % Inner circle - plot only solid sections (rotating with platform)
            for i = 1:size(obj.inner_solid_ranges, 1)
                theta_start_rot = obj.inner_solid_ranges(i, 1);
                theta_end_rot = obj.inner_solid_ranges(i, 2);

                % Apply platform rotation
                theta_start_inertial = theta_start_rot + theta;
                theta_end_inertial = theta_end_rot + theta;
                theta_section = linspace(theta_start_inertial, theta_end_inertial, 50);

                x_inner = obj.r_inner * cos(theta_section);
                y_inner = obj.r_inner * sin(theta_section);

                if i == 1
                    plot(x_inner, y_inner, 'k-', 'LineWidth', 3, 'DisplayName', 'Inner circle walls (r=0.2)');
                else
                    plot(x_inner, y_inner, 'k-', 'LineWidth', 3, 'HandleVisibility', 'off');
                end
            end

            % Outer circle - complete boundary (fixed in inertial frame)
            theta_circle = linspace(0, 2*pi, 200);
            x_outer = obj.r_outer * cos(theta_circle);
            y_outer = obj.r_outer * sin(theta_circle);
            plot(x_outer, y_outer, 'k:', 'LineWidth', 2, 'DisplayName', 'Outer circle (r=0.23)');

            % Legend
            legend('Location', 'bestoutside');

            xlim([-0.25, 0.25]);
            ylim([-0.25, 0.25]);
        end

        function [floor_limits, wall_limits] = getGeometryLimits(obj, theta)
            % Get (x,y) coordinates of floor endpoints and inner circle wall endpoints
            % theta: rotation angle (default: use obj.theta)
            %
            % Returns:
            %   floor_limits: struct array with fields x_left, y_left, x_right, y_right
            %   wall_limits: struct array with fields x_start, y_start, x_end, y_end

            if nargin < 2
                theta = obj.theta;
            end

            % Floor limits
            n_floors = length(obj.C_tracks);
            floor_limits = struct('floor', {}, 'x_left', {}, 'y_left', {}, 'x_right', {}, 'y_right', {});

            for i = 1:n_floors
                C_k = obj.C_tracks(i);
                x_min = obj.floor_x_ranges(i, 1);
                x_max = obj.floor_x_ranges(i, 2);

                % Compute y values on parabola (in rotating frame)
                y_min = obj.k * x_min^2 + C_k;
                y_max = obj.k * x_max^2 + C_k;

                % Apply rotation to inertial frame
                x_left_rot = x_min*cos(theta) - y_min*sin(theta);
                y_left_rot = x_min*sin(theta) + y_min*cos(theta);

                x_right_rot = x_max*cos(theta) - y_max*sin(theta);
                y_right_rot = x_max*sin(theta) + y_max*cos(theta);

                floor_limits(i).floor = i;
                floor_limits(i).x_left = x_left_rot;
                floor_limits(i).y_left = y_left_rot;
                floor_limits(i).x_right = x_right_rot;
                floor_limits(i).y_right = y_right_rot;
            end

            % Inner circle wall limits
            n_walls = size(obj.inner_solid_ranges, 1);
            wall_limits = struct('wall', {}, 'x_start', {}, 'y_start', {}, 'x_end', {}, 'y_end', {});

            for i = 1:n_walls
                % Angular positions in rotating frame
                theta_start_rot = obj.inner_solid_ranges(i, 1);
                theta_end_rot = obj.inner_solid_ranges(i, 2);

                % Apply platform rotation to get inertial frame positions
                theta_start_inertial = theta_start_rot + theta;
                theta_end_inertial = theta_end_rot + theta;

                % Compute (x,y) at start and end of wall in inertial frame
                x_start = obj.r_inner * cos(theta_start_inertial);
                y_start = obj.r_inner * sin(theta_start_inertial);

                x_end = obj.r_inner * cos(theta_end_inertial);
                y_end = obj.r_inner * sin(theta_end_inertial);

                wall_limits(i).wall = i;
                wall_limits(i).x_start = x_start;
                wall_limits(i).y_start = y_start;
                wall_limits(i).x_end = x_end;
                wall_limits(i).y_end = y_end;
            end
        end

        function x_vals = computeIntersectionX(obj, C, r)
            % Compute x-coordinates where parabola y = kx^2 + C intersects circle x^2 + y^2 = r^2
            % Returns real positive and negative x values (symmetric)
            %
            % Equation: k^2*x^4 + (2*k*C + 1)*x^2 + C^2 - r^2 = 0
            % This is a biquadratic: solve for u = x^2

            a_coeff = obj.k^2;
            b_coeff = 2*obj.k*C + 1;
            c_coeff = C^2 - r^2;

            % Solve quadratic in u = x^2
            discriminant = b_coeff^2 - 4*a_coeff*c_coeff;

            if discriminant < 0
                % No real solutions
                x_vals = [];
                return;
            end

            u1 = (-b_coeff + sqrt(discriminant)) / (2*a_coeff);
            u2 = (-b_coeff - sqrt(discriminant)) / (2*a_coeff);

            x_vals = [];

            % For each positive u, get x = ±sqrt(u)
            if u1 > 0
                x_vals = [x_vals, sqrt(u1), -sqrt(u1)];
            elseif u1 == 0
                x_vals = [x_vals, 0];
            end

            if u2 > 0 && u2 ~= u1  % avoid duplicates
                x_vals = [x_vals, sqrt(u2), -sqrt(u2)];
            elseif u2 == 0 && u1 ~= 0
                x_vals = [x_vals, 0];
            end

            x_vals = sort(x_vals);  % Sort for consistent ordering
        end
    end

    methods (Static)
        function floor_ranges = getDifficultyLevel(level)
            % Get floor x-ranges for a specific difficulty level
            %
            % Input:
            %   level - String: 'very_easy', 'easy', 'medium', 'hard', 'very_hard'
            %
            % Output:
            %   floor_ranges - 8x2 matrix of [x_min, x_max] for each floor
            %
            % Usage:
            %   geom = MonzaGeometry();
            %   geom.floor_x_ranges = MonzaGeometry.getDifficultyLevel('hard');
            %   geom.updateFloorRanges();

            switch lower(level)
                case 'very_easy'
                    floor_ranges = [
                        0,     inf;    % Floor 1
                        -inf,  0;      % Floor 2
                        0,     inf;    % Floor 3
                        -inf,  0;      % Floor 4
                        0,     inf;    % Floor 5
                        -inf,  0;      % Floor 6
                        0,     inf;    % Floor 7
                        -inf,  0;      % Floor 8
                    ];

                case 'easy'
                    floor_ranges = [
                        -0.05,  inf;   % Floor 1
                        -inf,   0.05;  % Floor 2
                        -0.05,  inf;   % Floor 3
                        -inf,   0.05;  % Floor 4
                        -0.05,  inf;   % Floor 5
                        -inf,   0.05;  % Floor 6
                        -0.05,  inf;   % Floor 7
                        -inf,   0.05;  % Floor 8
                    ];

                case 'medium'
                    floor_ranges = [
                        -0.05,   inf;     % Floor 1
                        -inf,    0.125;   % Floor 2
                        -0.125,  inf;     % Floor 3
                        -inf,    0.12;    % Floor 4
                        -0.08,   inf;     % Floor 5
                        -inf,    0.1;     % Floor 6
                        -0.08,   inf;     % Floor 7
                        -inf,   -0.015;   % Floor 8
                    ];

                case 'hard'
                    floor_ranges = [
                        -0.05,  inf;     % Floor 1
                        -inf,   0.15;    % Floor 2
                        -0.15,  inf;     % Floor 3
                        -inf,   0.12;    % Floor 4
                        -0.08,  inf;     % Floor 5
                        -inf,   0.1;     % Floor 6
                        -0.08,  inf;     % Floor 7
                        -inf,  -0.015;   % Floor 8
                    ];

                case 'very_hard'
                    floor_ranges = [
                        -0.05,  inf;     % Floor 1
                        -inf,   0.15;    % Floor 2
                        -0.15,  inf;     % Floor 3
                        -inf,   0.16;    % Floor 4
                        -0.15,  inf;     % Floor 5
                        -inf,   0.1;     % Floor 6
                        -0.08,  inf;     % Floor 7
                        -inf,  -0.08;    % Floor 8
                    ];

                otherwise
                    error('Unknown difficulty level: %s. Valid options: very_easy, easy, medium, hard, very_hard', level);
            end
        end
    end
end
