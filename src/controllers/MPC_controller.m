function MPC_controller(config)
% MPC_CONTROLLER Implements Model Predictive Control using MATLAB's MPC Toolbox
% IMPROVED: Uses centralized configuration

    if nargin < 1
        config = config_simulation();
    end
    
    close all;
    
    % Create results directory
    if ~exist('results', 'dir')
        mkdir('results');
    end
    
    fprintf('========================================\n');
    fprintf('  MPC Controller\n');
    fprintf('========================================\n\n');
    
    %% ========== LOAD IDENTIFIED MODEL ==========
    fprintf('Loading identified model...\n');
    load('results/identified_models.mat', 'identified_models');
    
    sys = identified_models.sys_tf;
    dt = identified_models.dt;
    u_mean = identified_models.u_mean;
    y_mean = identified_models.y_mean;
    
    % Convert to discrete-time
    if ~isdt(sys)
        sys_d = c2d(sys, dt, 'zoh');
    else
        sys_d = sys;
    end
    
    fprintf('  Using discrete-time model with Ts = %.2f s\n', dt);
    
    %% ========== MPC PARAMETERS FROM CONFIG ==========
    P = config.MPC.P;
    M = config.MPC.M;
    Q_weight = config.MPC.Q_weight;
    R_weight = config.MPC.R_weight;
    
    % Constraints from config
    u_min = config.constraints.u_min;
    u_max = config.constraints.u_max;
    du_max = config.constraints.du_max;
    
    fprintf('\nMPC Parameters:\n');
    fprintf('  Prediction horizon (P): %d\n', P);
    fprintf('  Control horizon (M): %d\n', M);
    fprintf('  Q weight: %.3f\n', Q_weight);
    fprintf('  R weight: %.3f\n', R_weight);
    fprintf('  Constraints: u ∈ [%d, %d]%%, Δu ≤ %d%%\n', u_min, u_max, du_max);
    fprintf('\n');
    
    %% ========== CREATE MPC OBJECT ==========
    fprintf('Creating MPC controller...\n');
    
    mpcobj = mpc(sys_d, dt);
    
    % Set horizons
    mpcobj.PredictionHorizon = P;
    mpcobj.ControlHorizon = M;
    
    % Set weights
    mpcobj.Weights.OutputVariables = Q_weight;
    mpcobj.Weights.ManipulatedVariablesRate = R_weight;
    mpcobj.Weights.ManipulatedVariables = 0;
    
    % Set constraints (in deviation variables)
    mpcobj.ManipulatedVariables.Min = u_min - u_mean;
    mpcobj.ManipulatedVariables.Max = u_max - u_mean;
    mpcobj.ManipulatedVariables.RateMin = -du_max;
    mpcobj.ManipulatedVariables.RateMax = du_max;
    
    % Output constraints
    mpcobj.OutputVariables.Min = -Inf;
    mpcobj.OutputVariables.Max = Inf;
    
    fprintf('  MPC object created successfully\n');
    
    %% ========== SIMULATION PARAMETERS ==========
    t_sim = config.simulation.t_sim;
    t = 0:dt:t_sim;
    n_steps = length(t);
    
    % Build setpoint profile
    T_setpoint_abs = build_setpoint_profile(config);
    
    fprintf('\nSimulation Parameters:\n');
    fprintf('  Duration: %d s\n', t_sim);
    fprintf('  Samples: %d\n', n_steps);
    fprintf('  Setpoint changes: %s °C at %s s\n', ...
            mat2str(config.setpoint.values), mat2str(config.setpoint.times));
    fprintf('\n');
    
    %% ========== SIMULATION WITH MPC ==========
    fprintf('Running closed-loop simulation with MPC...\n');
    
    % Initialize
    y = zeros(n_steps, 1);
    u = zeros(n_steps, 1);
    y(1) = y_mean;
    u(1) = 0;
    
    % Initialize MPC state
    xmpc = mpcstate(mpcobj);
    
    % Thermal model parameters
    params = struct();
    params.dt = dt;
    params.T0 = y(1);
    params.model_type = config.thermal_model.model_type;
    
    for k = 1:n_steps-1
        % Current output (deviation)
        y_dev = y(k) - y_mean;
        
        % Setpoint (deviation)
        r_dev = T_setpoint_abs(k) - y_mean;
        
        % Compute optimal control move
        u_dev = mpcmove(mpcobj, xmpc, y_dev, r_dev);
        
        % Convert to absolute input
        u_new = u_dev + u_mean;
        u_new = max(u_min, min(u_max, u_new));
        u(k+1) = u_new;
        
        % Simulate plant
        params.T0 = y(k);
        params.t_final = dt;
        Q_func = @(t) u(k+1);
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
    MPC_results = struct();
    MPC_results.t = t;
    MPC_results.y = y;
    MPC_results.u = u;
    MPC_results.setpoint = T_setpoint_abs;
    MPC_results.error = error;
    MPC_results.MAE = MAE;
    MPC_results.RMSE = RMSE;
    MPC_results.ISE = ISE;
    MPC_results.IAE = IAE;
    MPC_results.control_effort = control_effort;
    MPC_results.params = struct('P', P, 'M', M, 'Q', Q_weight, 'R', R_weight);
    MPC_results.mpcobj = mpcobj;
    MPC_results.config = config;
    
    save('results/MPC_results.mat', 'MPC_results');
    fprintf('\nSaved: results/MPC_results.mat\n');
    
    %% ========== PLOTTING ==========
    fprintf('Generating plots...\n');
    
    figure('Position', config.plotting.figure_size);
    
    subplot(3,1,1);
    plot(t, T_setpoint_abs, 'k--', 'LineWidth', 2); hold on;
    plot(t, y, 'Color', config.plotting.colors.MPC, 'LineWidth', config.plotting.linewidth);
    xlabel('Time [s]'); ylabel('Temperature [°C]');
    legend('Setpoint', 'MPC Output', 'Location', 'best');
    title('MPC Controller - Temperature Tracking');
    grid on;
    
    subplot(3,1,2);
    stairs(t, u, 'Color', config.plotting.colors.MPC, 'LineWidth', config.plotting.linewidth);
    xlabel('Time [s]'); ylabel('Heater Power [%]');
    title('MPC Controller - Control Input');
    ylim([u_min-5, u_max+5]);
    grid on;
    
    subplot(3,1,3);
    plot(t, error, 'r', 'LineWidth', config.plotting.linewidth);
    xlabel('Time [s]'); ylabel('Error [°C]');
    title('MPC Controller - Tracking Error');
    grid on;
    
    sgtitle(sprintf('MPC Controller Results (MAE: %.3f°C, RMSE: %.3f°C)', MAE, RMSE));
    saveas(gcf, 'results/MPC_controller_plots.png');
    fprintf('  Saved: results/MPC_controller_plots.png\n');
    
    fprintf('\nMPC controller simulation complete!\n');
end