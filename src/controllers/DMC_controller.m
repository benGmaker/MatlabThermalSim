function DMC_controller(config)
% DMC_CONTROLLER Implements Dynamic Matrix Control for thermal system
% IMPROVED: Uses centralized configuration

    if nargin < 1
        config = config_simulation();
    end
    
    close all;
    
    % Create results directory if it doesn't exist
    if ~exist('results', 'dir')
        mkdir('results');
    end
    
    fprintf('========================================\n');
    fprintf('  DMC Controller\n');
    fprintf('========================================\n\n');
    
    %% ========== LOAD IDENTIFIED MODEL ==========
    fprintf('Loading identified model...\n');
    load('results/identified_models.mat', 'identified_models');
    
    dt = identified_models.dt;
    u_mean = identified_models.u_mean;
    y_mean = identified_models.y_mean;
    
    %% ========== DMC PARAMETERS FROM CONFIG ==========
    P = config.DMC.P;
    M = config.DMC.M;
    N = config.DMC.N;
    Q_weight = config.DMC.Q_weight;
    R_weight = config.DMC.R_weight;
    step_response_source = config.DMC.step_response_source;
    
    % Constraints from config
    u_min = config.constraints.u_min;
    u_max = config.constraints.u_max;
    du_max = config.constraints.du_max;
    
    fprintf('DMC Parameters:\n');
    fprintf('  Prediction horizon (P): %d\n', P);
    fprintf('  Control horizon (M): %d\n', M);
    fprintf('  Model horizon (N): %d\n', N);
    fprintf('  Q weight: %.3f\n', Q_weight);
    fprintf('  R weight: %.3f\n', R_weight);
    fprintf('  Step response source: %s\n', step_response_source);
    fprintf('  Constraints: u ∈ [%d, %d]%%, Δu ≤ %d%%\n', u_min, u_max, du_max);
    fprintf('\n');
    
    %% ========== GET STEP RESPONSE COEFFICIENTS ==========
    fprintf('Computing step response coefficients...\n');
    
    switch step_response_source
        case 'measured'
            S = get_measured_step_response(N, dt, u_mean, y_mean);
            fprintf('  Using measured step response data\n');
            
        case 'ss_model'
            sys_ss = identified_models.sys_ss;
            S = get_model_step_response(sys_ss, N, dt);
            fprintf('  Using state-space model step response\n');
            
        case 'tf_model'
            sys_tf = identified_models.sys_tf;
            S = get_model_step_response(sys_tf, N, dt);
            fprintf('  Using transfer function model step response\n');
            
        otherwise
            error('Unknown step response source: %s', step_response_source);
    end
    
    fprintf('  Step response final value: %.4f\n', S(end));
    fprintf('  Step response length: %d samples\n', length(S));
    
    if S(end) < 0
        warning('Negative step response detected - check system identification');
    end
    
    %% ========== BUILD DMC MATRICES ==========
    fprintf('Building DMC matrices...\n');
    
    % Dynamic matrix
    G = zeros(P, M);
    for i = 1:P
        for j = 1:M
            if i >= j
                idx = i - j + 1;
                if idx <= N
                    G(i,j) = S(idx);
                else
                    G(i,j) = S(end);
                end
            end
        end
    end
    
    % Weight matrices
    Q_mat = Q_weight * eye(P);
    R_mat = R_weight * eye(M);
    
    % DMC gain
    K_dmc = inv(G' * Q_mat * G + R_mat) * G' * Q_mat;
    
    fprintf('  DMC gain (first element): %.4f\n', K_dmc(1,1));
    fprintf('  Dynamic matrix condition number: %.2e\n', cond(G));
    
    %% ========== SIMULATION PARAMETERS ==========
    t_sim = config.simulation.t_sim;
    t = 0:dt:t_sim;
    n_steps = length(t);
    
    % Build setpoint profile from config
    T_setpoint = build_setpoint_profile(config);
    
    fprintf('\nSimulation Parameters:\n');
    fprintf('  Duration: %d s\n', t_sim);
    fprintf('  Samples: %d\n', n_steps);
    fprintf('  Setpoint changes: %s °C at %s s\n', ...
            mat2str(config.setpoint.values), mat2str(config.setpoint.times));
    fprintf('\n');
    
    %% ========== SIMULATION WITH DMC ==========
    fprintf('Running closed-loop simulation with DMC...\n');
    
    % Initialize
    y = zeros(n_steps, 1);
    u = zeros(n_steps, 1);
    y(1) = y_mean;
    u(1) = 0;
    
    % Thermal model parameters
    params = struct();
    params.dt = dt;
    params.T0 = y(1);
    params.model_type = config.thermal_model.model_type;
    
    for k = 1:n_steps-1
        % Current state (deviation)
        y_dev = y(k) - y_mean;
        u_dev = u(k) - u_mean;
        
        % Setpoint trajectory (deviation)
        sp_traj = T_setpoint(k:min(k+P-1, end)) - y_mean;
        if length(sp_traj) < P
            sp_traj = [sp_traj; sp_traj(end)*ones(P-length(sp_traj), 1)];
        end
        
        % Free response
        y_free = y_dev * ones(P, 1);
        
        % Error to correct
        e = sp_traj - y_free;
        
        % Optimal control moves
        du_opt = K_dmc * e;
        du_0 = du_opt(1);
        
        % Apply constraints
        du_0 = max(-du_max, min(du_max, du_0));
        u_dev_new = u_dev + du_0;
        u_new = u_dev_new + u_mean;
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
                    k, y(k), T_setpoint(k), u(k));
        end
    end
    
    %% ========== PERFORMANCE METRICS ==========
    fprintf('\nComputing performance metrics...\n');
    
    error = T_setpoint - y;
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
    DMC_results.step_response_source = step_response_source;
    DMC_results.step_response = S;
    DMC_results.config = config;
    
    save('results/DMC_results.mat', 'DMC_results');
    fprintf('\nSaved: results/DMC_results.mat\n');
    
    %% ========== PLOTTING ==========
    fprintf('Generating plots...\n');
    
    figure('Position', config.plotting.figure_size);
    
    subplot(3,2,1);
    plot(t, T_setpoint, 'k--', 'LineWidth', 2); hold on;
    plot(t, y, 'Color', config.plotting.colors.DMC, 'LineWidth', config.plotting.linewidth);
    xlabel('Time [s]'); ylabel('Temperature [°C]');
    legend('Setpoint', 'DMC Output', 'Location', 'best');
    title('DMC Controller - Temperature Tracking');
    grid on;
    
    subplot(3,2,2);
    stairs(t, u, 'Color', config.plotting.colors.DMC, 'LineWidth', config.plotting.linewidth);
    xlabel('Time [s]'); ylabel('Heater Power [%]');
    title('DMC Controller - Control Input');
    ylim([u_min-5, u_max+5]);
    grid on;
    
    subplot(3,2,3);
    plot(t, error, 'r', 'LineWidth', config.plotting.linewidth);
    xlabel('Time [s]'); ylabel('Error [°C]');
    title('DMC Controller - Tracking Error');
    grid on;
    
    subplot(3,2,4);
    plot(0:length(S)-1, S, 'b.-', 'LineWidth', config.plotting.linewidth, 'MarkerSize', 8);
    xlabel('Sample'); ylabel('Step Response Coefficient');
    title(sprintf('Step Response Model (%s)', step_response_source));
    grid on;
    
    subplot(3,2,5);
    imagesc(G);
    colorbar;
    xlabel('Control Horizon'); ylabel('Prediction Horizon');
    title('Dynamic Matrix G');
    
    subplot(3,2,6);
    histogram(du, 30);
    xlabel('Control Move [%/sample]'); ylabel('Frequency');
    title('Distribution of Control Moves');
    grid on;
    
    sgtitle(sprintf('DMC Controller Results (%s) - MAE: %.3f°C, RMSE: %.3f°C', ...
                    step_response_source, MAE, RMSE));
    saveas(gcf, 'results/DMC_controller_plots.png');
    fprintf('  Saved: results/DMC_controller_plots.png\n');
    
    fprintf('\nDMC controller simulation complete!\n');
end

%% ========== HELPER FUNCTIONS ==========

function S = get_measured_step_response(N, dt, u_mean, y_mean)
    data = load('results/step_response_data.mat');
    step_data = data.step_data;
    
    if isfield(step_data, 'step_idx')
        step_idx = step_data.step_idx;
    else
        step_idx = find(step_data.Q > mean(step_data.Q), 1, 'first');
    end
    
    if isfield(step_data, 'step_amplitude')
        step_magnitude = step_data.step_amplitude;
    else
        u_before = mean(step_data.Q(1:max(1,step_idx-1)));
        u_after = mean(step_data.Q(step_idx:min(step_idx+10, end)));
        step_magnitude = u_after - u_before;
    end
    
    if abs(step_magnitude) < 1e-6
        error('Step magnitude too small');
    end
    
    y_before = mean(step_data.T(1:max(1,step_idx-1)));
    y_response = step_data.T(step_idx:end) - y_before;
    y_unit_step = y_response / step_magnitude;
    
    if length(y_unit_step) >= N
        S = y_unit_step(1:N);
    else
        S = [y_unit_step; y_unit_step(end) * ones(N - length(y_unit_step), 1)];
    end
    
    S = S(:);
    
    fprintf('  Measured step magnitude: %.2f\n', step_magnitude);
    fprintf('  Measured steady-state gain: %.4f\n', S(end));
end

function S = get_model_step_response(sys, N, dt)
    [y_step, t_step] = step(sys, (0:N-1)*dt);
    S = y_step;
    
    if length(S) > N
        S = S(1:N);
    elseif length(S) < N
        S = [S; S(end) * ones(N - length(S), 1)];
    end
    
    S = S(:);
end