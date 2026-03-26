function [u_opt, status, dbg] = solve_deepc_qp(U_p, Y_p, U_f, Y_f, u_ini, y_ini, r, ...
                                               Qw, Rw, ...
                                               lambda_y, lambda_g, ...
                                               u_min, u_max, du_max, u_prev, ...
                                               config)

    dbg = struct();

    % ---------------- Dimensions ----------------
    N     = size(U_f, 1);
    n_g   = size(U_f, 2);
    T_ini = size(U_p, 1);

 % ---------------- Config ----------------
    % You must provide these fields in config.DeePC 
    slack_mode            = string(config.DeePC.slack_mode);
    soft_uini             = logical(config.DeePC.soft_uini);
    lambda_uini           = config.DeePC.lambda_uini;

    enable_du_penalty     = logical(config.constraints.enable_du_penalty);
    enable_du_constraints = logical(config.constraints.enable_du_constraints);
    du_weight             = config.constraints.du_weight;
    du_min_cfg            = config.constraints.du_min;
    du_max_cfg            = config.constraints.du_max;

    enable_scaling        = logical(config.DeePC.enable_scaling);
    scaling_eps           = config.DeePC.scaling_eps;

    enable_y_constraints  = logical(config.constraints.enable_y_constraints);

    % Bounds live in config.constraints 
    y_min = config.constraints.y_min;
    y_max = config.constraints.y_max;

    % ---------------- Slack usage  ----------------
    use_sigma_y = (lambda_y > 0);
    use_sigma_u = soft_uini && (lambda_uini > 0);

    % ---------------- Weights ----------------
    Q = Qw * eye(N);
    R = Rw * eye(N);

     % ---------------- Scaling ----------------
    Su_p = ones(size(U_p,1),1);
    Sy_p = ones(size(Y_p,1),1);
    Su_f = ones(size(U_f,1),1);
    Sy_f = ones(size(Y_f,1),1);

    if enable_scaling
        row_rms = @(A) sqrt(mean(A.^2, 2));
        Su_p = 1 ./ max(row_rms(U_p), scaling_eps);
        Sy_p = 1 ./ max(row_rms(Y_p), scaling_eps);
        Su_f = 1 ./ max(row_rms(U_f), scaling_eps);
        Sy_f = 1 ./ max(row_rms(Y_f), scaling_eps);
    end

    U_p_s = Su_p .* U_p;
    Y_p_s = Sy_p .* Y_p;
    U_f_s = Su_f .* U_f;
    Y_f_s = Sy_f .* Y_f;

    u_ini_s = Su_p .* u_ini;
    y_ini_s = Sy_p .* y_ini;
    r_s     = Sy_f .* r;

    % Scalar scaling for bounds (SISO)
    u_prev_s = Su_f(1) * u_prev;
    u_min_s  = Su_f(1) * u_min;
    u_max_s  = Su_f(1) * u_max;

    du_max_s = Su_f(1) * du_max;         % legacy single-step limit
    du_min_s = Su_f(1) * (-du_max);      % legacy symmetric (if used)
    du_min_cfg_s = Su_f(1) * du_min_cfg; % full-horizon Δu min
    du_max_cfg_s = Su_f(1) * du_max_cfg; % full-horizon Δu max

    dbg.Su_p = Su_p; dbg.Sy_p = Sy_p; dbg.Su_f = Su_f; dbg.Sy_f = Sy_f;

    % ---------------- Weights ----------------
    Q = Qw * eye(N);
    R = Rw * eye(N);

    % ---------------- Decision variables ----------------
    n_sigma_y = T_ini * use_sigma_y;
    n_sigma_u = T_ini * use_sigma_u;
    n_vars    = n_g + n_sigma_y + n_sigma_u;

    idx_g = 1:n_g;
    base = n_g;
    idx_sigma_y = base + (1:n_sigma_y);
    base = base + n_sigma_y;
    idx_sigma_u = base + (1:n_sigma_u);

    % ---------------- Cost ----------------
    H = zeros(n_vars);
    f = zeros(n_vars, 1);

    % Tracking: ||Y_f g - r||_Q^2 (scaled)
    H(idx_g, idx_g) = H(idx_g, idx_g) + 2*(Y_f_s' * Q * Y_f_s);
    f(idx_g)        = f(idx_g)        - 2*(Y_f_s' * Q * r_s);

    % Input magnitude: ||U_f g||_R^2 (scaled)
    H(idx_g, idx_g) = H(idx_g, idx_g) + 2*(U_f_s' * R * U_f_s);

    % g regularization (you set lambda_g = 0 for unregularized tests)
    if lambda_g > 0
        H(idx_g, idx_g) = H(idx_g, idx_g) + 2*lambda_g*eye(n_g);
    end

    % sigma penalties
    if use_sigma_y
        % n_sigma_y == T_ini when enabled
        H(idx_sigma_y, idx_sigma_y) = H(idx_sigma_y, idx_sigma_y) + 2*lambda_y*eye(n_sigma_y);
    end
    if use_sigma_u
        % penalty on u_ini slack should use lambda_uini (not lambda_y)
        H(idx_sigma_u, idx_sigma_u) = H(idx_sigma_u, idx_sigma_u) + 2*lambda_uini*eye(n_sigma_u);
    end

    % ---- Δu penalty: (D*(U_f*g) - d0)' S (D*(U_f*g) - d0) (scaled) ----
    if enable_du_penalty
        Dm = eye(N);
        for k = 2:N
            Dm(k, k-1) = -1;
        end
        S = du_weight * eye(N);

        A = Dm * U_f_s;
        d0 = zeros(N,1);
        d0(1) = u_prev_s;

        H(idx_g, idx_g) = H(idx_g, idx_g) + 2*(A' * S * A);
        f(idx_g)        = f(idx_g)        + 2*(A' * S * (-d0));
    end

    % ---------------- Equalities ----------------
    n_eq = 2*T_ini;
    A_eq = zeros(n_eq, n_vars);
    b_eq = zeros(n_eq, 1);

    row = 0;

    % U_p*g (+ sigma_u) = u_ini
    A_eq(row+(1:T_ini), idx_g) = U_p_s;
    if use_sigma_u
        A_eq(row+(1:T_ini), idx_sigma_u) = eye(T_ini);
    end
    b_eq(row+(1:T_ini)) = u_ini_s;
    row = row + T_ini;

    % Y_p*g (+ sigma_y) = y_ini
    A_eq(row+(1:T_ini), idx_g) = Y_p_s;
    if use_sigma_y
        A_eq(row+(1:T_ini), idx_sigma_y) = eye(T_ini);
    end
    b_eq(row+(1:T_ini)) = y_ini_s;

    % ---------------- Inequalities ----------------
    A_ineq = [];
    b_ineq = [];

    % Input bounds on U = U_f*g (scaled)
    if isfinite(u_max)
        A_ineq = [A_ineq; [ U_f_s, zeros(N, n_sigma_y+n_sigma_u)]];
        b_ineq = [b_ineq; u_max_s * ones(N,1)];
    end
    if isfinite(u_min)
        A_ineq = [A_ineq; [-U_f_s, zeros(N, n_sigma_y+n_sigma_u)]];
        b_ineq = [b_ineq; -u_min_s * ones(N,1)];
    end

    % Output bounds on Y = Y_f*g (scaled)
    if enable_y_constraints
        y_max_s = Sy_f .* (y_max * ones(N,1));
        y_min_s = Sy_f .* (y_min * ones(N,1));

        A_ineq = [A_ineq; [ Y_f_s, zeros(N, n_sigma_y+n_sigma_u)]];
        b_ineq = [b_ineq; y_max_s];

        A_ineq = [A_ineq; [-Y_f_s, zeros(N, n_sigma_y+n_sigma_u)]];
        b_ineq = [b_ineq; -y_min_s];
    end

    % Δu constraints: du_min <= D*(U_f*g) - d0 <= du_max (scaled)
    if enable_du_constraints
        Dm = eye(N);
        for k = 2:N
            Dm(k, k-1) = -1;
        end
        d0 = zeros(N,1);
        d0(1) = u_prev_s;

        A = Dm * U_f_s;

        A_ineq = [A_ineq;
                  [ A,  zeros(N, n_sigma_y+n_sigma_u)];
                  [-A,  zeros(N, n_sigma_y+n_sigma_u)]];
        b_ineq = [b_ineq;
                  (du_max_cfg_s*ones(N,1) + d0);
                 -(du_min_cfg_s*ones(N,1) + d0)];
    else
        % Backward compatible: first-move-only constraint (scaled)
        if isfinite(du_max)
            Uf1 = U_f_s(1,:);
            A_ineq = [A_ineq;
                      [ Uf1, zeros(1, n_sigma_y+n_sigma_u)];
                      [-Uf1, zeros(1, n_sigma_y+n_sigma_u)]];
            b_ineq = [b_ineq;
                      u_prev_s + du_max_s;
                     -u_prev_s + du_max_s];
        end
    end

    % ---------------- Hessian repair ----------------
    H = (H + H')/2;
    eps_reg = 1e-10 * max(1, norm(H, 'fro'));
    H = H + eps_reg * eye(n_vars);
    H = (H + H')/2;

    % ---------------- Solve ----------------
    options = optimoptions('quadprog', ...
        'Display', 'off', ...
        'MaxIterations', 1000, ...
        'ConstraintTolerance', 1e-8, ...
        'OptimalityTolerance', 1e-8);

    try
        [x_opt, ~, exitflag, output] = quadprog(H, f, A_ineq, b_ineq, A_eq, b_eq, [], [], [], options);
        dbg.exitflag = exitflag;
        dbg.output = output;

        if exitflag > 0
            g_opt  = x_opt(idx_g);

            % scaled horizon input
            u_hor_s = U_f_s * g_opt;

            % unscale to physical deviation horizon:
            % U_f_s = diag(Su_f) * U_f  =>  U_f*g = diag(1./Su_f) * (U_f_s*g)
            u_opt = (1 ./ Su_f) .* u_hor_s;

            status = 0;
            dbg.norm_g = norm(g_opt);
            dbg.u1 = u_opt(1);
        else
            u_opt = zeros(N,1);
            status = -1;
        end

    catch ME
        warning("quadprog failed: %s", ME.message);
        u_opt = zeros(N,1);
        status = -1;
        dbg.exception = ME;
    end
end