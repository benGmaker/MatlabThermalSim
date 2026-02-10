function MPC_controller()
% MPC_CONTROLLER Implements Model Predictive Control using MATLAB's MPC Toolbox
% Uses identified transfer function model

    clc; close all;
    
    %% ========== LOAD IDENTIFIED MODEL ==========
    fprintf('Loading identified model...\n');
    load('identified_models.mat', 'identified_models');
    
    sys = identified_models.sys_tf;
    dt = identified_models.dt;
    u_mean = identified_models.u_mean;
    y_mean = identified_models.y_mean;
    
    % Convert to discrete-time if continuous
    if ~isdt(sys)
        sys_d = c2d(sys, dt, 'zoh');
    else
        sys_d = sys;
    end
    
    fprintf('  Using discrete-time model with Ts = %.2f s\n', dt);
    
    %% ========== MPC PARAMETERS ==========
    P = 30;             % Prediction horizon
    M = 10;             % Control horizon
    
    % Weights
    Q_weight = 1;       % Output tracking weight
    R_weight = 0.01;    % Input change penalty
    
    % Constraints
    u_min = 0;          % Min heater power [%]
    u_max = 100;        % Max heater power [%]
    du_max = 50;        % Max heater power change [%/sample]
    
    %% ========== CREATE MPC OBJECT ==========
    fprintf('Creating MPC controller...\n');
    
    % Create MPC controller
    mpcobj = mpc(sys_d, dt);
    
    % Set horizons
    mpcobj.PredictionHorizon = P;
    mpcobj.ControlHorizon = M;
    
    % Set weights
    mpcobj.Weights.OutputVariables = Q_weight;
    mpcobj.Weights.ManipulatedVariablesRate = R_weight;
    mpcobj.Weights.ManipulatedVariables = 0;  % No penalty on absolute input
    
    % Set constraints (in deviation variables)
    mpcobj.ManipulatedVariables.Min = u_min - u_mean;
    mpcobj.ManipulatedVariables.Max = u_max - u_mean;
    mpcobj.ManipulatedVariables.RateMin = -du_max;
    mpcobj.ManipulatedVariables.RateMax = du_max;
    
    % Output constraints (none for now)
    mpcobj.OutputVariables.Min = -Inf;
    mpcobj.OutputVariables.Max = Inf;
    
    % Display MPC properties
    fprintf('  Prediction Horizon: %d\n', mpcobj.PredictionHorizon);
    fprintf('  Control Horizon: %d\n', mpcobj.ControlHorizon);
    fprintf('  Sample Time: %.2f s\n', mpcobj.Ts);
    
    %% ========== SIMULATION PARAMETERS ==========
    t_sim = 600;        % Simulation time [s]
    t = 0:dt:t_sim;
    n_steps = length(t);
    
    % Setpoint profile (in deviation variables)
    T_setpoint_abs = zeros(n_steps, 1);
    T_setpoint_abs(1:200) = 40;         % First setpoint: 40°C
    T_setpoint_abs(201:400) = 50;       % Second setpoint: 50°C
    T_setpoint_abs(401:end) = 35;       % Final setpoint: 35°C
    
    %% ========== SIMULATION WITH MPC ==========
    fprintf('Running closed-loop simulation with MPC...\n');
    
    % Initialize
    y = zeros(n_steps, 1);
    u = zeros(n_steps, 1);
    y(1) = y_mean;      % Start at ambient temperature
    u(1) = 0;           % Start with heater off
    
    % Initialize MPC state
    xmpc = mpcstate(mpcobj);
    
    % Thermal model parameters
    params = struct();
    params.dt = dt;
    params.T0 = y(1);
    params.model_type = 'nonlinear';
    
    for k = 1:n_steps-1
        % Current output (deviation from operating point)
        y_dev = y(k) - y_mean;
        
        % Setpoint (deviation from operating point)
        r_dev = T_setpoint_abs(k) - y_mean;
        
        % Compute optimal control move
        u_dev = mpcmove(mpcobj, xmpc, y_dev, r_dev);
        
        % Convert to absolute input
        u_new = u_dev + u_mean;
        
        % Apply absolute constraints (safety check)
        u_new = max(u_min, min(u_max, u_new));
        u(k+1) = u_new;
        
        % Simulate plant (one step)
        params.T0 = y(k);
        params.t_final = dt;
        Q_func = @(t) u(k+1);
        [T_sim, ~, ~] = thermal_model(Q_func, params);
        y(k+1) = T_sim(end);
        
        % Debug output every 100 steps
        if mod(k, 100) == 0
            fprintf('  Step %d: T=%.2f°C, SP=%.2f°C, u=%.2f%%\n', ...
                    k, y(k), T_setpoint_abs(k), u(k));
        end
    end
    
    %% ========== PERFORMANCE METRICS ==========
    fprintf('\nComputing performance metrics...\n');
    
    % Calculate metrics
    error = T_setpoint_abs - y;
    MAE = mean(abs(error));
    RMSE = sqrt(mean(error.^2));
    ISE = sum(error.^2) * dt;
    IAE = sum(abs(error)) * dt;
    
    % Control effort
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
    MPC_results.mpcobj = mpcobj;  % Save MPC object for reference
    
    save('MPC_results.mat', 'MPC_results');
    fprintf('\nSaved: MPC_results.mat\n');
    
    %% ========== PLOTTING ==========
    fprintf('Generating plots...\n');
    
    figure('Position', [100, 100, 1200, 800]);
    
    % Temperature tracking
    subplot(3,1,1);
    plot(t, T_setpoint_abs, 'k--', 'LineWidth', 1.5); hold on;
    plot(t, y, 'r', 'LineWidth', 1.5);
    xlabel('Time [s]'); ylabel('Temperature [°C]');
    legend('Setpoint', 'MPC Output', 'Location', 'best');
    title('MPC Controller - Temperature Tracking');
    grid on;
    
    % Control input
    subplot(3,1,2);
    stairs(t, u, 'b', 'LineWidth', 1.5);
    xlabel('Time [s]'); ylabel('Heater Power [%]');
    title('MPC Controller - Control Input');
    ylim([u_min-5, u_max+5]);
    grid on;
    
    % Tracking error
    subplot(3,1,3);
    plot(t, error, 'r', 'LineWidth', 1.5);
    xlabel('Time [s]'); ylabel('Error [°C]');
    title('MPC Controller - Tracking Error');
    grid on;
    
    sgtitle(sprintf('MPC Controller Results (MAE: %.3f°C, RMSE: %.3f°C)', MAE, RMSE));
    saveas(gcf, 'MPC_controller_plots.png');
    fprintf('  Saved: MPC_controller_plots.png\n');
    
    fprintf('\nMPC controller simulation complete!\n');
end