function [ctrl_step, ctrl_init, meta] = deepc_policy_factory(config)
%DEEPC_POLICY_FACTORY Builds the DeePC data (Hankel matrices) once,
%then returns a step-wise "policy" that solves the QP each sample.

    if nargin < 1
        config = config_simulation();
    end

    
    load(config.predictive.data_source, 'step_data'); % hard coded needs long term solution


    u_data_abs = step_data.Q(:);
    y_data_abs = step_data.T(:);
    dt = step_data.params.dt;

    % Center data (DeePC in deviation variables)
    u_mean = mean(u_data_abs);
    y_mean = mean(y_data_abs);

    u_data = u_data_abs - u_mean;
    y_data = y_data_abs - y_mean;

    L = length(u_data);

    % DeePC params
    T_ini = config.DeePC.T_ini;
    N = config.DeePC.N;
    lambda_y = config.DeePC.lambda_y;
    lambda_g = config.DeePC.lambda_g;
    lambda_u = config.DeePC.lambda_u;

    % Constraints in deviation variables
    u_min = config.constraints.u_min - u_mean;
    u_max = config.constraints.u_max - u_mean;
    du_max = config.constraints.du_max;

    % Hankel sizes
    T = T_ini + N;
    n_cols = L - T + 1;
    if n_cols <= 0
        error('Insufficient DeePC data length: need L >= T_ini+N.');
    end

    % Build Hankel matrices
    U_p = zeros(T_ini, n_cols);
    Y_p = zeros(T_ini, n_cols);
    U_f = zeros(N, n_cols);
    Y_f = zeros(N, n_cols);

    for i = 1:n_cols
        U_p(:,i) = u_data(i:i+T_ini-1);
        Y_p(:,i) = y_data(i:i+T_ini-1);
        U_f(:,i) = u_data(i+T_ini:i+T_ini+N-1);
        Y_f(:,i) = y_data(i+T_ini:i+T_ini+N-1);
    end

    % Init/step
    ctrl_init = @() struct( ...
        'dt', dt, ...
        'u_mean', u_mean, ...
        'y_mean', y_mean, ...
        'T_ini', T_ini, ...
        'N', N, ...
        'lambda_y', lambda_y, ...
        'lambda_g', lambda_g, ...
        'lambda_u', lambda_u, ...
        'u_min', u_min, ...
        'u_max', u_max, ...
        'du_max', du_max, ...
        'U_p', U_p, ...
        'Y_p', Y_p, ...
        'U_f', U_f, ...
        'Y_f', Y_f, ...
        'u_hist', zeros(T_ini,1), ...   % deviation history
        'y_hist', zeros(T_ini,1), ...   % deviation history
        'u_prev', 0, ...               % previous u in deviation
        'n_failures', 0 ...
    );

    ctrl_step = @step;

    meta = struct();
    meta.dt = dt;
    meta.u_mean = u_mean;
    meta.y_mean = y_mean;
    meta.hankel_cols = n_cols;
    meta.cond_UpYp = cond([U_p; Y_p]);

    function [u_next_abs, ctrl] = step(k, y_k_abs, r_traj_abs, ctrl, config) %#ok<INUSD>
        % Current output in deviation
        y_k = y_k_abs - ctrl.y_mean;

        % Update past window (deviation)
        % Shift and append newest sample (like maintaining last T_ini points)
        ctrl.y_hist = [ctrl.y_hist(2:end); y_k];
        % ctrl.u_hist already contains u(k-T_ini+1:k) in deviation. It will be updated
        % after we compute u(k+1). For DeePC formulation we need u_ini and y_ini
        u_ini = ctrl.u_hist;
        y_ini = ctrl.y_hist;

        % Build reference trajectory in deviation (length N)
        r = r_traj_abs(:) - ctrl.y_mean;
        if numel(r) < ctrl.N
            r(end+1:ctrl.N,1) = r(end);
        elseif numel(r) > ctrl.N
            r = r(1:ctrl.N);
        end

        % Solve DeePC QP (returns deviation u sequence)
        [u_opt, status] = solve_deepc_qp( ...
            ctrl.U_p, ctrl.Y_p, ctrl.U_f, ctrl.Y_f, ...
            u_ini, y_ini, r, ...
            ctrl.lambda_y, ctrl.lambda_g, ctrl.lambda_u, ...
            ctrl.u_min, ctrl.u_max, ctrl.du_max, ctrl.u_prev);

        if status == 0
            u_next_dev = u_opt(1);
        else
            % Same fallback as your original: incremental P controller (in deviation)
            ctrl.n_failures = ctrl.n_failures + 1;
            err = r(1) - y_k;
            Kp = 2.0;
            u_next_dev = ctrl.u_prev + Kp * err;
            u_next_dev = max(ctrl.u_min, min(ctrl.u_max, u_next_dev));
        end

        % Update stored previous u (deviation) and past u window
        ctrl.u_prev = u_next_dev;
        ctrl.u_hist = [ctrl.u_hist(2:end); u_next_dev];

        % Convert to absolute
        u_next_abs = u_next_dev + ctrl.u_mean;
        u_next_abs = max(config.constraints.u_min, min(config.constraints.u_max, u_next_abs));
    end
end

function [u_opt, status] = solve_deepc_qp(U_p, Y_p, U_f, Y_f, u_ini, y_ini, r, ...
                                         lambda_y, lambda_g, lambda_u, ...
                                         u_min, u_max, du_max, u_prev)

    [N, n_cols] = size(U_f);
    T_ini = size(U_p, 1);

    % Decision variables: [g; u; sigma_y; sigma_u]
    n_g = n_cols;
    n_u = N;
    n_sigma_y = T_ini;
    n_sigma_u = T_ini;
    n_vars = n_g + n_u + n_sigma_y + n_sigma_u;

    idx_g = 1:n_g;
    idx_u = n_g + (1:n_u);
    idx_sigma_y = n_g + n_u + (1:n_sigma_y);
    idx_sigma_u = n_g + n_u + n_sigma_y + (1:n_sigma_u);

    % Cost
    H = zeros(n_vars);
    f = zeros(n_vars, 1);

    % Output tracking via g (matches your original implementation)
    H(idx_g, idx_g) = 2 * (Y_f' * Y_f);
    f(idx_g) = -2 * (Y_f' * r);

    % Input regularization
    H(idx_u, idx_u) = 2 * lambda_u * eye(n_u);

    % Slack penalties
    H(idx_sigma_y, idx_sigma_y) = 2 * lambda_y * eye(n_sigma_y);
    H(idx_sigma_u, idx_sigma_u) = 2 * lambda_g * eye(n_sigma_u);

    H = H + 1e-6 * eye(n_vars);

    % Equalities
    A_eq = zeros(2*T_ini + N, n_vars);
    b_eq = zeros(2*T_ini + N, 1);

    % U_p*g = u_ini + sigma_u
    A_eq(1:T_ini, idx_g) = U_p;
    A_eq(1:T_ini, idx_sigma_u) = -eye(T_ini);
    b_eq(1:T_ini) = u_ini;

    % Y_p*g = y_ini + sigma_y
    A_eq(T_ini+(1:T_ini), idx_g) = Y_p;
    A_eq(T_ini+(1:T_ini), idx_sigma_y) = -eye(T_ini);
    b_eq(T_ini+(1:T_ini)) = y_ini;

    % U_f*g = u
    A_eq(2*T_ini+(1:N), idx_g) = U_f;
    A_eq(2*T_ini+(1:N), idx_u) = -eye(N);
    b_eq(2*T_ini+(1:N)) = 0;

    % Inequalities
    A_ineq = [];
    b_ineq = [];

    % Input bounds on u decision vars
    A_u = [zeros(2*N, n_g), [eye(N); -eye(N)], zeros(2*N, n_sigma_y + n_sigma_u)];
    b_u = [u_max * ones(N, 1); -u_min * ones(N, 1)];
    A_ineq = [A_ineq; A_u];
    b_ineq = [b_ineq; b_u];

    % Rate constraint ONLY on first move (same as your original)
    A_rate = [zeros(2, n_g), [1, zeros(1, N-1); -1, zeros(1, N-1)], zeros(2, n_sigma_y + n_sigma_u)];
    b_rate = [u_prev + du_max; -u_prev + du_max];
    A_ineq = [A_ineq; A_rate];
    b_ineq = [b_ineq; b_rate];

    % Solve
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
    catch
        u_opt = zeros(N, 1);
        status = -1;
    end
end