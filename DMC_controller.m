function DMC_controller()
% DMC_CONTROLLER Implements Dynamic Matrix Control for thermal system
% Uses step response model for prediction and optimization
    clc; close all;
    % Create results directory if it doesn't exist
    if ~exist('results', 'dir')
        mkdir('results');
    end
    %% ========== LOAD IDENTIFIED MODEL ==========
    fprintf('Loading identified model...\n');
    load('results/identified_models.mat', 'identified_models');
    
    sys = identified_models.sys_tf;
    dt = identified_models.dt;
    u_mean = identified_models.u_mean;
    y_mean = identified_models.y_mean;
    
    %% ========== DMC PARAMETERS ==========
    P = 30;             % Prediction horizon
    M = 10;             % Control horizon
    N = 100;            % Model horizon (step response length)
    
    % Weights
    Q_weight = 1;       % Output tracking weight
    R_weight = 0.01;    % Input change penalty (reduced for better tracking)
    
    % Constraints
    u_min = 0;          % Min heater power [%]
    u_max = 100;        % Max heater power [%]
    du_max = 50;        % Max heater power change [%/sample] (increased)
    
    %% ========== SIMULATION PARAMETERS ==========
    t_sim = 600;        % Simulation time [s]
    t = 0:dt:t_sim;
    n_steps = length(t);
    
    % Setpoint profile
    T_setpoint = zeros(n_steps, 1);
    T_setpoint(1:200) = 40;         % First setpoint: 40°C
    T_setpoint(201:400) = 50;       % Second setpoint: 50°C
    T_setpoint(401:end) = 35;       % Final setpoint: 35°C
    
    %% ========== GET STEP RESPONSE COEFFICIENTS ==========
    fprintf('Computing step response coefficients...\n');
    
    % Generate step response to UNIT STEP (deviation variables)
    [y_step, t_step] = step(sys, (0:N-1)*dt);
    S = y_step;  % Step response coefficients
    
    % Debug: Check if step response is positive (system has positive gain)
    fprintf('  Step response final value: %.4f\n', S(end));
    if S(end) < 0
        warning('Negative step response detected - check system identification');
    end
    
    %% ========== BUILD DMC MATRICES ==========
    fprintf('Building DMC matrices...\n');
    
    % Dynamic matrix (P x M) - maps future control MOVES to future outputs
    G = zeros(P, M);
    for i = 1:P
        for j = 1:M
            if i >= j
                idx = i - j + 1;
                if idx <= N
                    G(i,j) = S(idx);
                else
                    G(i,j) = S(end);  % Use steady-state value
                end
            end
        end
    end
    
    % Weight matrices
    Q_mat = Q_weight * eye(P);
    R_mat = R_weight * eye(M);
    
    % DMC gain (unconstrained solution)
    K_dmc = inv(G' * Q_mat * G + R_mat) * G' * Q_mat;
    
    fprintf('  DMC gain (first element): %.4f\n', K_dmc(1,1));
    
    %% ========== SIMULATION WITH DMC ==========
    fprintf('Running closed-loop simulation with DMC...\n');
    
    % Initialize
    y = zeros(n_steps, 1);
    u = zeros(n_steps, 1);
    y(1) = y_mean;      % Start at ambient temperature
    u(1) = 0;           % Start with heater off
    
    % Thermal model parameters
    params = struct();
    params.dt = dt;
    params.T0 = y(1);
    params.model_type = 'nonlinear';
    
    for k = 1:n_steps-1
        % Current output (deviation from operating point)
        y_dev = y(k) - y_mean;
        
        % Current input (deviation from operating point)
        u_dev = u(k) - u_mean;
        
        % Setpoint trajectory (deviation from mean)
        sp_traj = T_setpoint(k:min(k+P-1, end)) - y_mean;
        if length(sp_traj) < P
            sp_traj = [sp_traj; sp_traj(end)*ones(P-length(sp_traj), 1)];
        end
        
        % Predict future outputs assuming no future control action
        % For simplified DMC: free response = current output
        y_free = y_dev * ones(P, 1);
        
        % Error to correct
        e = sp_traj - y_free;
        
        % Compute optimal control MOVES (deviations)
        du_opt = K_dmc * e;
        
        % First control move
        du_0 = du_opt(1);
        
        % Apply rate constraint
        du_0 = max(-du_max, min(du_max, du_0));
        
        % Compute new input (in deviation variables)
        u_dev_new = u_dev + du_0;
        
        % Convert back to absolute input
        u_new = u_dev_new + u_mean;
        
        % Apply absolute constraints
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
                    k, y(k), T_setpoint(k), u(k));
        end
    end
    
    %% ========== PERFORMANCE METRICS ==========
    fprintf('\nComputing performance metrics...\n');
    
    % Calculate metrics
    error = T_setpoint - y;
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
    DMC_results = struct();
    DMC_results.t = t;
    DMC_results.y = y;
    DMC_results.u = u;
    DMC_results.setpoint = T_setpoint;
    DMC_results.error = error;
    DMC_results.MAE = MAE;
    DMC_results.RMSE = RMSE;
    DMC_results.ISE = ISE;
    DMC_results.IAE = IAE;
    DMC_results.control_effort = control_effort;
    DMC_results.params = struct('P', P, 'M', M, 'N', N, 'Q', Q_weight, 'R', R_weight);
    
    save('results/DMC_results.mat', 'DMC_results');
    fprintf('\nSaved: results/DMC_results.mat\n');
    
    %% ========== PLOTTING ==========
    fprintf('Generating plots...\n');
    
    figure('Position', [100, 100, 1200, 800]);
    
    % Temperature tracking
    subplot(3,1,1);
    plot(t, T_setpoint, 'k--', 'LineWidth', 1.5); hold on;
    plot(t, y, 'r', 'LineWidth', 1.5);
    xlabel('Time [s]'); ylabel('Temperature [°C]');
    legend('Setpoint', 'DMC Output', 'Location', 'best');
    title('DMC Controller - Temperature Tracking');
    grid on;
    
    % Control input
    subplot(3,1,2);
    stairs(t, u, 'b', 'LineWidth', 1.5);
    xlabel('Time [s]'); ylabel('Heater Power [%]');
    title('DMC Controller - Control Input');
    ylim([u_min-5, u_max+5]);
    grid on;
    
    % Tracking error
    subplot(3,1,3);
    plot(t, error, 'r', 'LineWidth', 1.5);
    xlabel('Time [s]'); ylabel('Error [°C]');
    title('DMC Controller - Tracking Error');
    grid on;
    
    sgtitle(sprintf('DMC Controller Results (MAE: %.3f°C, RMSE: %.3f°C)', MAE, RMSE));
    saveas(gcf, 'results/DMC_controller_plots.png');
    fprintf('  Saved: results/DMC_controller_plots.png\n');
    
    fprintf('\nDMC controller simulation complete!\n');
end