function [u_opt, status, dbg] = solve_deepc_qp(U_p, Y_p, U_f, Y_f, u_ini, y_ini, r, ...
                                               Qw, Rw, ...
                                               lambda_y, lambda_g, ...
                                               u_min, u_max, du_max, u_prev, ...
                                               config)

    dbg = struct();

    [N, n_cols] = size(U_f);
    T_ini = size(U_p, 1);

    % Decide mode from config
    deterministic = config.DeePC.deterministic;

    if deterministic
        % =========================
        % Deterministic DeePC (no slack)
        % Decision variables: [g; u]
        % =========================
        n_g = n_cols;
        n_u = N;
        n_vars = n_g + n_u;

        idx_g = 1:n_g;
        idx_u = n_g + (1:n_u);

        % Cost
        H = zeros(n_vars);
        f = zeros(n_vars, 1);

        Q = Qw * eye(N);
        R = Rw * eye(N);

        H(idx_g, idx_g) = 2 * (Y_f' * Q * Y_f);
        f(idx_g)        = -2 * (Y_f' * Q * r);
        H(idx_u, idx_u) = 2 * R;

        H = H + 1e-6 * eye(n_vars);
        H = (H + H')/2;

        % Equalities (hard)
        A_eq = zeros(2*T_ini + N, n_vars);
        b_eq = zeros(2*T_ini + N, 1);

        % U_p*g = u_ini
        A_eq(1:T_ini, idx_g) = U_p;
        b_eq(1:T_ini) = u_ini;

        % Y_p*g = y_ini
        A_eq(T_ini+(1:T_ini), idx_g) = Y_p;
        b_eq(T_ini+(1:T_ini)) = y_ini;

        % U_f*g = u
        A_eq(2*T_ini+(1:N), idx_g) = U_f;
        A_eq(2*T_ini+(1:N), idx_u) = -eye(N);
        b_eq(2*T_ini+(1:N)) = 0;

        % Inequalities
        A_ineq = [];
        b_ineq = [];

        if isfinite(u_min) || isfinite(u_max)
            A_u = [zeros(2*N, n_g), [eye(N); -eye(N)]];
            b_u = [u_max * ones(N, 1); -u_min * ones(N, 1)];
            A_ineq = [A_ineq; A_u];
            b_ineq = [b_ineq; b_u];
        end

        if isfinite(du_max)
            A_rate = [zeros(2, n_g), [1, zeros(1, N-1); -1, zeros(1, N-1)]];
            b_rate = [u_prev + du_max; -u_prev + du_max];
            A_ineq = [A_ineq; A_rate];
            b_ineq = [b_ineq; b_rate];
        end

        options = optimoptions('quadprog', 'Display', 'off', ...
            'MaxIterations', 200, ...
            'ConstraintTolerance', 1e-8, ...
            'OptimalityTolerance', 1e-8);

        try
            [x_opt, ~, exitflag, output] = quadprog(H, f, A_ineq, b_ineq, A_eq, b_eq, [], [], [], options);
            dbg.exitflag = exitflag;
            dbg.output = output;

            if exitflag > 0
                u_opt = x_opt(idx_u);
                status = 0;
            else
                u_opt = zeros(N, 1);
                status = -1;
            end
        catch ME
            warning("quadprog failed (deterministic): %s", ME.message);
            u_opt = zeros(N, 1);
            status = -1;
        end

        return;
    end

  
    % =========================
    % Regularized DeePC (slacks + penalties)
    % Decision variables: [g; u; sigma_y; sigma_u]
    % =========================
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
    Q = Qw * eye(N);
    R = Rw * eye(N);

    H(idx_g, idx_g) = 2 * (Y_f' * Q * Y_f);
    f(idx_g)        = -2 * (Y_f' * Q * r);
    H(idx_u, idx_u) = 2 * R;

    % Slack penalties
    H(idx_sigma_y, idx_sigma_y) = 2 * lambda_y * eye(n_sigma_y);
    H(idx_sigma_u, idx_sigma_u) = 2 * lambda_g * eye(n_sigma_u);

    H = H + 1e-6 * eye(n_vars);
    H = (H + H')/2;

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

    if isfinite(u_min) || isfinite(u_max)
        A_u = [zeros(2*N, n_g), [eye(N); -eye(N)], zeros(2*N, n_sigma_y + n_sigma_u)];
        b_u = [u_max * ones(N, 1); -u_min * ones(N, 1)];
        A_ineq = [A_ineq; A_u];
        b_ineq = [b_ineq; b_u];
    end

    if isfinite(du_max)
        A_rate = [zeros(2, n_g), [1, zeros(1, N-1); -1, zeros(1, N-1)], zeros(2, n_sigma_y + n_sigma_u)];
        b_rate = [u_prev + du_max; -u_prev + du_max];
        A_ineq = [A_ineq; A_rate];
        b_ineq = [b_ineq; b_rate];
    end

    options = optimoptions('quadprog', 'Display', 'off', ...
        'MaxIterations', 200, ...
        'ConstraintTolerance', 1e-5, ...
        'OptimalityTolerance', 1e-5);

    try
        [x_opt, ~, exitflag, output] = quadprog(H, f, A_ineq, b_ineq, A_eq, b_eq, [], [], [], options);
        dbg.exitflag = exitflag;
        dbg.output = output;
        dbg.norm_sigma_y = norm(x_opt(idx_sigma_y));
        dbg.norm_sigma_u = norm(x_opt(idx_sigma_u));

        if exitflag > 0
            u_opt = x_opt(idx_u);
            status = 0;
        else
            u_opt = zeros(N, 1);
            status = -1;
        end
    catch ME
        warning("quadprog failed (regularized): %s", ME.message);
        u_opt = zeros(N, 1);
        status = -1;
    end
end