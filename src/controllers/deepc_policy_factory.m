function [ctrl_step, ctrl_init, meta] = deepc_policy_factory(config)
%DEEPC_POLICY_FACTORY Builds the DeePC data (Hankel matrices) once,
%then returns a step-wise "policy" that solves the QP each sample.

    if nargin < 1
        config = config_simulation();
    end

    % loading data
    ds = load_predictive_data(config);
    u_data_abs = ds.Q(:);
    y_data_abs = ds.T(:);
    dt = ds.params.dt;

    % Center data (DeePC in deviation variables)
    u_mean = mean(u_data_abs);
    y_mean = mean(y_data_abs);

    u_data = u_data_abs - u_mean;
    y_data = y_data_abs - y_mean;

    L = length(u_data);

    % General params
    Qw = config.predictive.Q_weight;
    Rw = config.predictive.R_weight;

    % DeePC params
    T_ini = config.DeePC.T_ini;
    N = config.predictive.P; % Prediction horizon [samples]
    lambda_y = config.DeePC.lambda_y;
    lambda_g = config.DeePC.lambda_g;

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
    
    check_pe_condition(U_p, struct('name','U_p'));
    check_pe_condition(Y_p, struct('name','Y_p'));
    check_pe_condition(U_f, struct('name','U_f'));
    check_pe_condition(Y_f, struct('name','Y_f'));

    % Init/step
    ctrl_init = @() struct( ...
        'dt', dt, ...
        'u_mean', u_mean, ...
        'y_mean', y_mean, ...
        'T_ini', T_ini, ...
        'Qw', Qw, ...
        'Rw', Rw, ...
        'N', N, ...
        'lambda_y', lambda_y, ...
        'lambda_g', lambda_g, ...
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

    function [u_next_abs, ctrl] = step(k, y_k_abs, r_traj_abs, ctrl, config) 
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
            ctrl.Qw, ctrl.Rw, ...
            ctrl.lambda_y, ctrl.lambda_g, ...
            ctrl.u_min, ctrl.u_max, ctrl.du_max, ctrl.u_prev, ...
            config);

        if status == 0
            u_next_dev = u_opt(1);
        else
            % Same fallback as your original: incremental P controller (in deviation)
            ctrl.n_failures = ctrl.n_failures + 1;
            err = r(1) - y_k;
            Kp = 2.0;
            u_next_dev = ctrl.u_prev + Kp * err;
        end

        % Update stored previous u (deviation) and past u window
        ctrl.u_prev = u_next_dev;
        ctrl.u_hist = [ctrl.u_hist(2:end); u_next_dev];

        % Convert to absolute
        u_next_abs = u_next_dev + ctrl.u_mean;
    end
end
