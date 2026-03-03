function SPC_controller(config)
% SPC_CONTROLLER Implements Subspace Predictive Control (SPC) experiment
% Approach:
%   1) Identify a discrete-time state-space model via subspace ID (n4sid)
%   2) Build an MPC controller on that identified model (MPC Toolbox)
%   3) Run closed-loop simulation using the repo thermal_model(...) (same call as MPC_controller)

    if nargin < 1
        config = config_simulation();
    end

    close all;

    % Create results directory
    if ~exist('results', 'dir')
        mkdir('results');
    end

    fprintf('========================================\n');
    fprintf('  SPC (Subspace Predictive Control)\n');
    fprintf('========================================\n\n');

    %% ========== LOAD DATA FOR SUBSPACE ID ==========
    fprintf('Loading data for subspace identification...\n');
    load('results/multisine_response_data.mat', 'multisine_data');

    u_data = multisine_data.Q(:);
    y_data = multisine_data.T(:);
    dt = multisine_data.params.dt;

    % Center the data
    u_mean = mean(u_data);
    y_mean = mean(y_data);
    u_id = u_data - u_mean;
    y_id = y_data - y_mean;

    fprintf('  Data length: %d samples\n', length(u_id));
    fprintf('  dt: %.3f s\n', dt);
    fprintf('  u mean: %.3f, y mean: %.3f\n\n', u_mean, y_mean);

    %% ========== IDENTIFY STATE-SPACE MODEL (SUBSPACE) ==========
    fprintf('Identifying state-space model using %s...\n', config.SPC.ident.method);

    % Requires System Identification Toolbox
    z = iddata(y_id, u_id, dt);
    nx = config.SPC.ident.nx;

    switch lower(config.SPC.ident.method)
        case 'n4sid'
            opt = n4sidOptions('Focus', config.SPC.ident.focus);
            sys_id = n4sid(z, nx, opt);
        otherwise
            error('Unsupported config.SPC.ident.method: %s (use ''n4sid'')', config.SPC.ident.method);
    end

    % Ensure discrete-time
    if ~isdt(sys_id)
        sys_id = c2d(sys_id, dt, 'zoh');
    end

    sys_ss = ss(sys_id);

    fprintf('  Identified model order nx=%d\n', nx);
    fprintf('  Identified model Ts=%.3f s\n\n', sys_ss.Ts);

    %% ========== SPC (MPC) PARAMETERS FROM CONFIG ==========
    P = config.SPC.P;
    M = config.SPC.M;
    Q_weight = config.SPC.Q_weight;
    R_weight = config.SPC.R_weight;

    % Constraints (absolute)
    u_min = config.constraints.u_min;
    u_max = config.constraints.u_max;
    du_max = config.constraints.du_max;

    fprintf('SPC Parameters:\n');
    fprintf('  Prediction horizon (P): %d\n', P);
    fprintf('  Control horizon (M): %d\n', M);
    fprintf('  Q weight: %.3f\n', Q_weight);
    fprintf('  R weight: %.3f\n', R_weight);
    fprintf('  Constraints: u ∈ [%d, %d]%%, Δu ≤ %d%%\n\n', u_min, u_max, du_max);

    %% ========== CREATE MPC OBJECT (on subspace model) ==========
    fprintf('Creating MPC controller based on identified subspace model...\n');

    mpcobj = mpc(sys_ss, dt);

    % Set horizons
    mpcobj.PredictionHorizon = P;
    mpcobj.ControlHorizon = M;

    % Set weights (match MPC_controller style)
    mpcobj.Weights.OutputVariables = Q_weight;
    mpcobj.Weights.ManipulatedVariablesRate = R_weight;
    mpcobj.Weights.ManipulatedVariables = 0;

    % Set constraints in deviation variables
    mpcobj.ManipulatedVariables.Min = u_min - u_mean;
    mpcobj.ManipulatedVariables.Max = u_max - u_mean;
    mpcobj.ManipulatedVariables.RateMin = -du_max;
    mpcobj.ManipulatedVariables.RateMax = du_max;

    % Output constraints
    mpcobj.OutputVariables.Min = -Inf;
    mpcobj.OutputVariables.Max = Inf;

    fprintf('  SPC MPC object created successfully\n');

    %% ========== SIMULATION PARAMETERS ==========
    t_sim = config.simulation.t_sim;
    t = 0:dt:t_sim;
    n_steps = length(t);

    % Build setpoint profile (absolute)
    T_setpoint_abs = build_setpoint_profile(config);

    fprintf('\nSimulation Parameters:\n');
    fprintf('  Duration: %d s\n', t_sim);
    fprintf('  Samples: %d\n', n_steps);
    fprintf('  Setpoint changes: %s °C at %s s\n', ...
            mat2str(config.setpoint.values), mat2str(config.setpoint.times));
    fprintf('\n');

    %% ========== SIMULATION WITH SPC ==========
    fprintf('Running closed-loop simulation with SPC...\n');

    % Initialize
    y = zeros(n_steps, 1);
    u = zeros(n_steps, 1);
    y(1) = y_mean;
    u(1) = 0;

    % Initialize MPC state
    xmpc = mpcstate(mpcobj);

    % Thermal model parameters (match MPC_controller style)
    params = struct();
    params.dt = dt;
    params.T0 = y(1);
    params.model_type = config.thermal_model.model_type;

    for k = 1:n_steps-1
        % Current output (deviation)
        y_dev = y(k) - y_mean;

        % Setpoint (deviation)
        r_dev = T_setpoint_abs(k) - y_mean;

        % Compute optimal control move (deviation)
        u_dev = mpcmove(mpcobj, xmpc, y_dev, r_dev);

        % Convert to absolute input and saturate
        u_new = u_dev + u_mean;
        u_new = max(u_min, min(u_max, u_new));
        u(k+1) = u_new;

        % Simulate plant (USE EXACT SAME CALL AS OTHER CONTROLLERS)
        params.T0 = y(k);
        params.t_final = dt;
        Q_func = @(tt) u(k+1);
        [T_sim, ~, ~] = thermal_model(Q_func, params);
        y(k+1) = T_sim(end);

        if mod(k, 100) == 0
            fprintf('  Step %d: T=%.2f°C, SP=%.2f°C, u=%.2f%%\n', ...
                    k, y(k), T_setpoint_abs(k), u(k));
        end
    end

    %% ========== PERFORMANCE METRICS ==========
    fprintf('\nComputing performance metrics...\n');

    error = T_setpoint_abs - y;
    MAE = mean(abs(error));
    RMSE = sqrt(mean(error.^2));
    ISE = sum(error.^2) * dt;
    IAE = sum(abs(error)) * dt;

    du = [0; diff(u)];
    control_effort = sum(abs(du));

    fprintf('  MAE: %.3f °C\n', MAE);
    fprintf('  RMSE: %.3f °C\n', RMSE);
    fprintf('  ISE: %.3f\n', ISE);
    fprintf('  IAE: %.3f\n', IAE);
    fprintf('  Total control effort: %.3f\n', control_effort);

    %% ========== SAVE RESULTS ==========
    SPC_results = struct();
    SPC_results.t = t(:);
    SPC_results.y = y;
    SPC_results.u = u;
    SPC_results.setpoint = T_setpoint_abs;
    SPC_results.error = error;
    SPC_results.MAE = MAE;
    SPC_results.RMSE = RMSE;
    SPC_results.ISE = ISE;
    SPC_results.IAE = IAE;
    SPC_results.control_effort = control_effort;
    SPC_results.params = struct('P', P, 'M', M, 'Q', Q_weight, 'R', R_weight, 'nx', nx);
    SPC_results.mpcobj = mpcobj;
    SPC_results.model = sys_ss;
    SPC_results.u_mean = u_mean;
    SPC_results.y_mean = y_mean;
    SPC_results.config = config;

    save('results/SPC_results.mat', 'SPC_results');
    fprintf('\nSaved: results/SPC_results.mat\n');

    %% ========== PLOTTING ==========
    fprintf('Generating plots...\n');

    figure('Position', config.plotting.figure_size);

    subplot(3,1,1);
    plot(t, T_setpoint_abs, 'k--', 'LineWidth', 2); hold on;
    if isfield(config.plotting.colors, 'SPC')
        spc_color = config.plotting.colors.SPC;
    else
        spc_color = [0.49, 0.18, 0.56];
    end
    plot(t, y, 'Color', spc_color, 'LineWidth', config.plotting.linewidth);
    xlabel('Time [s]'); ylabel('Temperature [°C]');
    legend('Setpoint', 'SPC Output', 'Location', 'best');
    title('SPC Controller - Temperature Tracking');
    grid on;

    subplot(3,1,2);
    stairs(t, u, 'Color', spc_color, 'LineWidth', config.plotting.linewidth);
    xlabel('Time [s]'); ylabel('Heater Power [%]');
    title('SPC Controller - Control Input');
    ylim([u_min-5, u_max+5]);
    grid on;

    subplot(3,1,3);
    plot(t, error, 'r', 'LineWidth', config.plotting.linewidth);
    xlabel('Time [s]'); ylabel('Error [°C]');
    title('SPC Controller - Tracking Error');
    grid on;

    sgtitle(sprintf('SPC Controller Results (MAE: %.3f°C, RMSE: %.3f°C)', MAE, RMSE));
    saveas(gcf, 'results/SPC_controller_plots.png');
    fprintf('  Saved: results/SPC_controller_plots.png\n');

    fprintf('\nSPC controller simulation complete!\n');
end

function T_setpoint_abs = build_setpoint_profile(config)
% Build setpoint profile matching the existing controller scripts

    t_sim = config.simulation.t_sim;
    dt = config.simulation.dt;
    t = 0:dt:t_sim;
    n_steps = length(t);

    T_setpoint_abs = ones(n_steps, 1) * config.setpoint.values(1);

    for i = 1:length(config.setpoint.times)
        idx = round(config.setpoint.times(i) / dt) + 1;
        if idx <= n_steps
            T_setpoint_abs(idx:end) = config.setpoint.values(i);
        end
    end
end