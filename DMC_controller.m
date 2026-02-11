function DMC_controller()
% DMC_CONTROLLER Implements Dynamic Matrix Control for thermal system
% Uses step response model for prediction and optimization
% IMPROVED: Can use real measured data or state-space model instead of TF

    clc; close all;
    % Create results directory if it doesn't exist
    if ~exist('results', 'dir')
        mkdir('results');
    end
    
    %% ========== CONFIGURATION ==========
    % Choose step response source:
    % 'measured'  - Use actual step response from experimental data
    % 'ss_model'  - Use state-space model from system identification
    % 'tf_model'  - Use transfer function model (original method)
    step_response_source = 'measured';  % RECOMMENDED: 'measured' for true DMC
    
    fprintf('DMC Controller Configuration:\n');
    fprintf('  Step response source: %s\n\n', step_response_source);
    
    %% ========== LOAD DATA BASED ON SOURCE ==========
    fprintf('Loading data and models...\n');
    load('results/identified_models.mat', 'identified_models');
    
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
    
    %% ========== GET STEP RESPONSE COEFFICIENTS ==========
    fprintf('Computing step response coefficients...\n');
    
    switch step_response_source
        case 'measured'
            % Use actual measured step response data
            S = get_measured_step_response(N, dt, u_mean, y_mean);
            fprintf('  Using measured step response data\n');
            
        case 'ss_model'
            % Use state-space model
            sys_ss = identified_models.sys_ss;
            S = get_model_step_response(sys_ss, N, dt);
            fprintf('  Using state-space model step response\n');
            
        case 'tf_model'
            % Use transfer function model (original method)
            sys_tf = identified_models.sys_tf;
            S = get_model_step_response(sys_tf, N, dt);
            fprintf('  Using transfer function model step response\n');
            
        otherwise
            error('Unknown step response source: %s', step_response_source);
    end
    
    % Validate step response
    fprintf('  Step response final value: %.4f\n', S(end));
    fprintf('  Step response length: %d samples\n', length(S));
    
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
    fprintf('  Dynamic matrix condition number: %.2e\n', cond(G));
    
    %% ========== SIMULATION PARAMETERS ==========
    t_sim = 600;        % Simulation time [s]
    t = 0:dt:t_sim;
    n_steps = length(t);
    
    % Setpoint profile
    T_setpoint = zeros(n_steps, 1);
    T_setpoint(1:200) = 40;         % First setpoint: 40°C
    T_setpoint(201:400) = 50;       % Second setpoint: 50°C
    T_setpoint(401:end) = 35;       % Final setpoint: 35°C
    
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
    DMC_results.step_response_source = step_response_source;
    DMC_results.step_response = S;  % Save for analysis
    
    save('results/DMC_results.mat', 'DMC_results');
    fprintf('\nSaved: results/DMC_results.mat\n');
    
    %% ========== PLOTTING ==========
    fprintf('Generating plots...\n');
    
    figure('Position', [100, 100, 1400, 900]);
    
    % Temperature tracking
    subplot(3,2,1);
    plot(t, T_setpoint, 'k--', 'LineWidth', 1.5); hold on;
    plot(t, y, 'r', 'LineWidth', 1.5);
    xlabel('Time [s]'); ylabel('Temperature [°C]');
    legend('Setpoint', 'DMC Output', 'Location', 'best');
    title('DMC Controller - Temperature Tracking');
    grid on;
    
    % Control input
    subplot(3,2,2);
    stairs(t, u, 'b', 'LineWidth', 1.5);
    xlabel('Time [s]'); ylabel('Heater Power [%]');
    title('DMC Controller - Control Input');
    ylim([u_min-5, u_max+5]);
    grid on;
    
    % Tracking error
    subplot(3,2,3);
    plot(t, error, 'r', 'LineWidth', 1.5);
    xlabel('Time [s]'); ylabel('Error [°C]');
    title('DMC Controller - Tracking Error');
    grid on;
    
    % Step response coefficients used
    subplot(3,2,4);
    plot(0:length(S)-1, S, 'b.-', 'LineWidth', 1.5, 'MarkerSize', 8);
    xlabel('Sample'); ylabel('Step Response Coefficient');
    title(sprintf('Step Response Model (%s)', step_response_source));
    grid on;
    
    % Dynamic matrix visualization
    subplot(3,2,5);
    imagesc(G);
    colorbar;
    xlabel('Control Horizon'); ylabel('Prediction Horizon');
    title('Dynamic Matrix G');
    
    % Control move histogram
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
% Extract step response coefficients from measured experimental data
    
    % Load step response experimental data
    data = load('results/step_response_data.mat');
    step_data = data.step_data;
    
    t = step_data.t;
    u = step_data.Q;
    y = step_data.T;
    
    % Find when step occurs
    step_idx = find(u > mean(u), 1, 'first');
    if isempty(step_idx)
        error('Could not find step in measured data');
    end
    
    % Get step magnitude (in deviation variables)
    u_before = mean(u(1:max(1,step_idx-1)));
    u_after = mean(u(step_idx:min(step_idx+10, end)));
    step_magnitude = u_after - u_before;
    
    if abs(step_magnitude) < 1e-6
        error('Step magnitude too small');
    end
    
    % Extract output response (deviation from initial)
    y_before = mean(y(1:max(1,step_idx-1)));
    y_response = y(step_idx:end) - y_before;
    
    % Normalize to unit step
    y_unit_step = y_response / step_magnitude;
    
    % Resample if necessary to match desired length
    if length(y_unit_step) >= N
        % Take first N samples
        S = y_unit_step(1:N);
    else
        % Pad with final value
        S = [y_unit_step; y_unit_step(end) * ones(N - length(y_unit_step), 1)];
    end
    
    % Ensure column vector
    S = S(:);
    
    fprintf('  Measured step magnitude: %.2f\n', step_magnitude);
    fprintf('  Measured steady-state gain: %.4f\n', S(end));
end

function S = get_model_step_response(sys, N, dt)
% Get step response from a dynamic system model (TF or SS)
    
    % Generate step response to UNIT STEP
    [y_step, t_step] = step(sys, (0:N-1)*dt);
    
    % Extract step response coefficients
    S = y_step;
    
    % Ensure we have exactly N samples
    if length(S) > N
        S = S(1:N);
    elseif length(S) < N
        S = [S; S(end) * ones(N - length(S), 1)];
    end
    
    % Ensure column vector
    S = S(:);
end