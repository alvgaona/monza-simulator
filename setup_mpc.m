% setup_mpc.m
% Configure MATLAB nlmpc object for Monza platform control

function nlobj = setup_mpc(platform, opts)
    % Setup nonlinear MPC controller for ball on rotating parabolic floor
    %
    % Inputs:
    %   platform - MonzaPlatform instance
    %   opts     - MPC configuration options (see arguments block)
    %
    % Outputs:
    %   nlobj - configured nlmpc object

    arguments
        platform MonzaPlatform
        opts.x_min (1,1) double = -0.15          % Floor minimum x boundary [m]
        opts.x_max (1,1) double = 0.15           % Floor maximum x boundary [m]
        opts.boundary_margin (1,1) double = 0.02 % Safety margin from inner circle [m]
        opts.Ts (1,1) double = 0.05              % Sample time [s]
        opts.PredictionHorizon (1,1) double {mustBeInteger, mustBePositive} = 20
        opts.ControlHorizon (1,1) double {mustBeInteger, mustBePositive} = 10
        opts.omega_min (1,1) double = -5.0       % Min angular velocity [rad/s]
        opts.omega_max (1,1) double = 5.0        % Max angular velocity [rad/s]
        opts.Q_xR (1,1) double = 10              % Output weight for x_R
        opts.Q_vxR (1,1) double = 0              % Output weight for vx_R (0 = ignore)
        opts.Q_theta (1,1) double = 0            % Output weight for theta (0 = ignore)
        opts.R (1,1) double = 0.1                % Control weight
        opts.R_rate (1,1) double = 0.05          % Control rate weight
    end

    %% MPC Parameters (from arguments)
    Ts = opts.Ts;
    PredictionHorizon = opts.PredictionHorizon;
    ControlHorizon = opts.ControlHorizon;

    %% Create nlmpc object
    % State: [x_R; vx_R; theta]
    % Control: [omega]
    % Output: [x_R; vx_R; theta] (same as state)

    nx = 3;  % Number of states
    ny = 3;  % Number of outputs (all states)
    nu = 1;  % Number of control inputs

    nlobj = nlmpc(nx, ny, nu);

    %% Set dimensions and horizons
    nlobj.Ts = Ts;
    nlobj.PredictionHorizon = PredictionHorizon;
    nlobj.ControlHorizon = ControlHorizon;

    %% Setup params for platform methods
    params.g = platform.g;
    params.k = platform.geometry.k;
    params.C_k = platform.geometry.C_tracks(platform.current_floor);

    %% State function (system dynamics)
    nlobj.Model.StateFcn = @(x, u) platform.monza_state_fcn_wrapper(x, u, params);

    %% Output function (output = state)
    nlobj.Model.OutputFcn = @(x, u) x;  % Output all states: [x_R; vx_R; theta]

    %% Constraints
    % Control limits: angular velocity
    nlobj.MV.Min = opts.omega_min;
    nlobj.MV.Max = opts.omega_max;

    % State constraints: x_R bounds (only set if finite)
    if isfinite(opts.x_min)
        nlobj.States(1).Min = opts.x_min;
    end
    if isfinite(opts.x_max)
        nlobj.States(1).Max = opts.x_max;
    end

    %% Weights
    % Output weights for [x_R; vx_R; theta]
    nlobj.Weights.OutputVariables = [opts.Q_xR, opts.Q_vxR, opts.Q_theta];

    % Control weight (penalize large angular velocities)
    nlobj.Weights.ManipulatedVariables = opts.R;

    % Control rate weight (smooth control)
    nlobj.Weights.ManipulatedVariablesRate = opts.R_rate;

    %% Display configuration
    fprintf('=== NLMPC Configuration ===\n');
    fprintf('Sample Time: %.3f s\n', Ts);
    fprintf('Prediction Horizon: %d steps (%.2f s)\n', PredictionHorizon, PredictionHorizon*Ts);
    fprintf('Control Horizon: %d steps\n', ControlHorizon);
    fprintf('States: x_R, x_Rdot, theta\n');
    fprintf('Control: omega (%.1f to %.1f rad/s)\n', nlobj.MV.Min, nlobj.MV.Max);
    fprintf('State bounds: x_R ∈ [%.3f, %.3f] m\n', nlobj.States(1).Min, nlobj.States(1).Max);
    fprintf('Outputs: x_R, vx_R, theta (rotating frame)\n');
    fprintf('Weights: Q=[%.1f, %.1f, %.1f], R=%.2f, R_rate=%.3f\n', ...
        opts.Q_xR, opts.Q_vxR, opts.Q_theta, opts.R, opts.R_rate);
    fprintf('===========================\n');
end
