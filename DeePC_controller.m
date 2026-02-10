function DeePC_controller()
% DEEPC_CONTROLLER Implements Data-enabled Predictive Control
% Uses collected data directly without explicit model identification

    clc; close all;
    % Create results directory if it doesn't exist
    if ~exist('results', 'dir')
        mkdir('results');
    end
    %% ========== LOAD DATA FOR DeePC ==========
    fprintf('Loading data for DeePC...\n');
    
    % Use multisine data for DeePC (best excitation)
    load('results/multisine_response_data.mat', 'multisine_data');
    
    u_data = multisine_data.Q;
    y_data = multisine_data.T;
    dt = multisine_data.params.dt;
    
    % Center the data (remove mean)
    u_mean = mean(u_data);
    y_mean = mean(y_data);
    u_data = u_data - u_mean;
    y_data = y_data - y_mean;
    
    L = length(u_data);  % Total data length
    
    fprintf('  Data length: %d samples\n', L);
    fprintf('  Input mean: %.2f, std: %.2f\n', u_mean, std(u_data));
    fprintf('  Output mean: %.2f, std: %.2f\n', y_mean, std(y_data));
    
    %% ========== DeePC PARAMETERS ==========
    T_ini = 5;          % Past horizon (reduced for better conditioning)
    N = 15;             % Future horizon (reduced for stability)
    
    % Increase slack variable penalties
    lambda_y = 1000;     % Increased from 100
    lambda_g = 1000;     % Increased from 10  
    lambda_u = 10;       % Increased from 1

    % Constraints
    u_min = 0 - u_mean;     % Min heater power (deviation)
    u_max = 100 - u_mean;   % Max heater power (deviation)
    du_max = 50;            % Max heater power change (increased)
    
    %% ========== BUILD HANKEL MATRICES ==========
    fprintf('Building Hankel matrices...\n');
    
    T = T_ini + N;  % Total horizon
    
    % Check data sufficiency (Willems' fundamental lemma)
    min_cols = (T_ini + N + 1) * 2;  % Conservative estimate
    if L < T + min_cols
        warning('Data length may be insufficient. Consider longer experiments.');
    end
    
    % Number of columns in Hankel matrix
    n_cols = L - T + 1;
    
    % Build Hankel matrices
    U_p = zeros(T_ini, n_cols);  % Past inputs
    Y_p = zeros(T_ini, n_cols);  % Past outputs
    U_f = zeros(N, n_cols);      % Future inputs
    Y_f = zeros(N, n_cols);      % Future outputs
    
    for i = 1:n_cols
        U_p(:,i) = u_data(i:i+T_ini-1);
        Y_p(:,i) = y_data(i:i+T_ini-1);
        U_f(:,i) = u_data(i+T_ini:i+T_ini+N-1);
        Y_f(:,i) = y_data(i+T_ini:i+T_ini+N-1);
    end
    
    fprintf('  Hankel matrix size: %d columns\n', n_cols);
    fprintf('  Condition number of [Up; Yp]: %.2e\n', cond([U_p; Y_p]));
    
    %% ========== SIMULATION PARAMETERS ==========
    t_sim = 600;
    t = 0:dt:t_sim;
    n_steps = length(t);
    
    % Setpoint profile (deviation from mean)
    T_setpoint_abs = zeros(n_steps, 1);
    T_setpoint_abs(1:200) = 40;
    T_setpoint_abs(201:400) = 50;
    T_setpoint_abs(401:end) = 35;
    T_setpoint = T_setpoint_abs - y_mean;
    
    %% ========== SIMULATION WITH DeePC ==========
    fprintf('Running closed-loop simulation with DeePC...\n');
    
    % Initialize
    y = zeros(n_steps, 1);
    u = zeros(n_steps, 1);
    y(1) = 0;  % Start at equilibrium (deviation = 0)
    u(1) = 0;
    
    % Thermal model parameters
    params = struct();
    params.dt = dt;
    params.T0 = y_mean;
    params.model_type = 'nonlinear';
    
    % Storage for past data
    u_ini = zeros(T_ini, 1);
    y_ini = zeros(T_ini, 1);
    
    % Track solver failures
    n_failures = 0;
    
    for k = 1:n_steps-1
        % Update initial conditions
        start_idx = max(1, k - T_ini + 1);
        len = min(T_ini, k);
        
        if len < T_ini
            % Pad with zeros if not enough history
            u_ini(end-len+1:end) = u(start_idx:k);
            y_ini(end-len+1:end) = y(start_idx:k);
        else
            u_ini = u(start_idx:k);
            y_ini = y(start_idx:k);
        end
        
        % Reference trajectory
        r = T_setpoint(k:min(k+N-1, end));
        if length(r) < N
            r = [r; r(end)*ones(N-length(r), 1)];
        end
        
        % Solve DeePC optimization problem
        [u_opt, status] = solve_deepc_qp(U_p, Y_p, U_f, Y_f, ...
                                          u_ini, y_ini, r, ...
                                          lambda_y, lambda_g, lambda_u, ...
                                          u_min, u_max, du_max, u(k));
        
        if status == 0
            u(k+1) = u_opt(1);
        else
            % If optimization fails, use fallback controller
            n_failures = n_failures + 1;
            error_current = r(1) - y(k);
            % Simple proportional controller as fallback
            Kp = 2.0;
            u(k+1) = u(k) + Kp * error_current;
            u(k+1) = max(u_min, min(u_max, u(k+1)));
            
            if mod(n_failures, 10) == 1
                fprintf('  Warning: QP solver failed at step %d (total failures: %d)\n', k, n_failures);
            end
        end
        
        % Simulate plant
        params.T0 = y(k) + y_mean;
        params.t_final = dt;
        Q_func = @(t) u(k+1) + u_mean;
        [T_sim, ~, ~] = thermal_model(Q_func, params);
        y(k+1) = T_sim(end) - y_mean;
        
        % Debug output
        if mod(k, 100) == 0
            fprintf('  Step %d: T=%.2f°C, SP=%.2f°C, u=%.2f%%\n', ...
                    k, y(k)+y_mean, T_setpoint_abs(k), u(k)+u_mean);
        end
    end
    
    fprintf('  Total solver failures: %d (%.1f%%)\n', n_failures, 100*n_failures/n_steps);
    
    % Convert back to absolute values
    y_abs = y + y_mean;
    u_abs = u + u_mean;
    
    %% ========== PERFORMANCE METRICS ==========
    fprintf('\nComputing performance metrics...\n');
    
    error = T_setpoint_abs - y_abs;
    MAE = mean(abs(error));
    RMSE = sqrt(mean(error.^2));
    ISE = sum(error.^2) * dt;
    IAE = sum(abs(error)) * dt;
    
    du = [0; diff(u_abs)];
    control_effort = sum(abs(du));
    
    fprintf('  MAE: %.3f °C\n', MAE);
    fprintf('  RMSE: %.3f °C\n', RMSE);
    fprintf('  ISE: %.3f\n', ISE);
    fprintf('  IAE: %.3f\n', IAE);
    fprintf('  Total control effort: %.3f\n', control_effort);
    
    %% ========== SAVE RESULTS ==========
    DeePC_results = struct();
    DeePC_results.t = t;
    DeePC_results.y = y_abs;
    DeePC_results.u = u_abs;
    DeePC_results.setpoint = T_setpoint_abs;
    DeePC_results.error = error;
    DeePC_results.MAE = MAE;
    DeePC_results.RMSE = RMSE;
    DeePC_results.ISE = ISE;
    DeePC_results.IAE = IAE;
    DeePC_results.control_effort = control_effort;
    DeePC_results.n_failures = n_failures;
    DeePC_results.params = struct('T_ini', T_ini, 'N', N, ...
                                   'lambda_y', lambda_y, 'lambda_g', lambda_g);
    
    save('results/DeePC_results.mat', 'DeePC_results');
    fprintf('\nSaved: results/DeePC_results.mat\n');
    
    %% ========== PLOTTING ==========
    fprintf('Generating plots...\n');
    
    figure('Position', [100, 100, 1200, 800]);
    
    subplot(3,1,1);
    plot(t, T_setpoint_abs, 'k--', 'LineWidth', 1.5); hold on;
    plot(t, y_abs, 'r', 'LineWidth', 1.5);
    xlabel('Time [s]'); ylabel('Temperature [°C]');
    legend('Setpoint', 'DeePC Output', 'Location', 'best');
    title('DeePC Controller - Temperature Tracking');
    grid on;
    
    subplot(3,1,2);
    stairs(t, u_abs, 'b', 'LineWidth', 1.5);
    xlabel('Time [s]'); ylabel('Heater Power [%]');
    title('DeePC Controller - Control Input');
    grid on;
    
    subplot(3,1,3);
    plot(t, error, 'r', 'LineWidth', 1.5);
    xlabel('Time [s]'); ylabel('Error [°C]');
    title('DeePC Controller - Tracking Error');
    grid on;
    
    sgtitle(sprintf('DeePC Controller Results (MAE: %.3f°C, RMSE: %.3f°C)', MAE, RMSE));
    saveas(gcf, 'results/DeePC_controller_plots.png');
    fprintf('  Saved: results/DeePC_controller_plots.png\n');
    
    fprintf('\nDeePC controller simulation complete!\n');
end

function [u_opt, status] = solve_deepc_qp(U_p, Y_p, U_f, Y_f, u_ini, y_ini, r, ...
                                           lambda_y, lambda_g, lambda_u, ...
                                           u_min, u_max, du_max, u_prev)
% Solve DeePC quadratic programming problem with improved numerics
    
    [N, n_cols] = size(U_f);
    T_ini = size(U_p, 1);
    
    % Decision variables: [g; u; sigma_y; sigma_u]
    n_g = n_cols;
    n_u = N;
    n_sigma_y = T_ini;
    n_sigma_u = T_ini;
    n_vars = n_g + n_u + n_sigma_y + n_sigma_u;
    
    % Cost function: min 0.5*x'*H*x + f'*x
    H = zeros(n_vars);
    f = zeros(n_vars, 1);
    
    idx_g = 1:n_g;
    idx_u = n_g + (1:n_u);
    idx_sigma_y = n_g + n_u + (1:n_sigma_y);
    idx_sigma_u = n_g + n_u + n_sigma_y + (1:n_sigma_u);
    
    % Output tracking cost: ||Y_f*g - r||^2
    H(idx_g, idx_g) = 2 * (Y_f' * Y_f);
    f(idx_g) = -2 * Y_f' * r;
    
    % Input regularization: lambda_u * ||u||^2
    H(idx_u, idx_u) = 2 * lambda_u * eye(n_u);
    
    % Slack variable penalties
    H(idx_sigma_y, idx_sigma_y) = 2 * lambda_y * eye(n_sigma_y);  % lambda_y for output slack
    H(idx_sigma_u, idx_sigma_u) = 2 * lambda_g * eye(n_sigma_u);  % lambda_g for input slack
    
    % Ensure H is positive definite
    H = H + 1e-6 * eye(n_vars);
    
    % Equality constraints: [U_p; Y_p; U_f] * g = [u_ini + sigma_u; y_ini + sigma_y; u]
    A_eq = zeros(T_ini + T_ini + N, n_vars);
    b_eq = zeros(T_ini + T_ini + N, 1);
    
    % U_p * g = u_ini + sigma_u
    A_eq(1:T_ini, idx_g) = U_p;
    A_eq(1:T_ini, idx_sigma_u) = -eye(T_ini);
    b_eq(1:T_ini) = u_ini;
    
    % Y_p * g = y_ini + sigma_y
    A_eq(T_ini + (1:T_ini), idx_g) = Y_p;
    A_eq(T_ini + (1:T_ini), idx_sigma_y) = -eye(T_ini);
    b_eq(T_ini + (1:T_ini)) = y_ini;
    
    % U_f * g = u
    A_eq(2*T_ini + (1:N), idx_g) = U_f;
    A_eq(2*T_ini + (1:N), idx_u) = -eye(N);
    b_eq(2*T_ini + (1:N)) = 0;
    
    % Inequality constraints
    A_ineq = [];
    b_ineq = [];
    
    % Input bounds: u_min <= u <= u_max
    A_u = [zeros(2*N, n_g), [eye(N); -eye(N)], zeros(2*N, n_sigma_y + n_sigma_u)];
    b_u = [u_max * ones(N, 1); -u_min * ones(N, 1)];
    
    A_ineq = [A_ineq; A_u];
    b_ineq = [b_ineq; b_u];
    
    % Rate constraint on first moves
    A_rate = [zeros(2, n_g), [1, zeros(1, N-1); -1, zeros(1, N-1)], zeros(2, n_sigma_y + n_sigma_u)];
    b_rate = [u_prev + du_max; -u_prev + du_max];
    
    A_ineq = [A_ineq; A_rate];
    b_ineq = [b_ineq; b_rate];
    
    % Solve QP with better options
    options = optimoptions('quadprog', 'Display', 'off', ...
                           'MaxIterations', 200, ...
                           'ConstraintTolerance', 1e-5, ...
                           'OptimalityTolerance', 1e-5);
    
    try
        [x_opt, ~, exitflag] = quadprog(H, f, A_ineq, b_ineq, A_eq, b_eq, [], [], [], options);
        
        if exitflag > 0
            u_opt = x_opt(idx_u);
            status = 0;
        else
            u_opt = zeros(N, 1);
            status = -1;
        end
    catch ME
        % If solver crashes completely
        u_opt = zeros(N, 1);
        status = -1;
    end
end