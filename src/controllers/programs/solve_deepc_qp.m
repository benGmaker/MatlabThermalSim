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

    % ---------------- Config (keep same loading style) ----------------
    slack_mode = "g+u";
    if nargin >= 17 && isstruct(config) && isfield(config,'DeePC') && isfield(config.DeePC,'slack_mode')
        slack_mode = string(config.DeePC.slack_mode);
    end
    dbg.slack_mode = slack_mode;

    use_sigma_y = (~isempty(lambda_y) && isfinite(lambda_y) && lambda_y > 0);

    soft_uini = false;
    lambda_uini = 0;
    if isstruct(config) && isfield(config,'DeePC')
        if isfield(config.DeePC,'soft_uini')
            soft_uini = logical(config.DeePC.soft_uini);
        end
        if isfield(config.DeePC,'lambda_uini')
            lambda_uini = config.DeePC.lambda_uini;
        end
    end
    use_sigma_u = soft_uini && isfinite(lambda_uini) && lambda_uini > 0;

    % ---- SPC alignment: Δu options ----
    enable_du_penalty = false;
    enable_du_constraints = false;
    du_weight = 0;
    du_min = -du_max;
    du_max_cfg = du_max;

    if isstruct(config) && isfield(config,'DeePC')
        if isfield(config.DeePC,'enable_du_penalty')
            enable_du_penalty = logical(config.constraints.enable_du_penalty);
        end
        if isfield(config.DeePC,'enable_du_constraints')
            enable_du_constraints = logical(config.constraints.enable_du_constraints);
        end
        if isfield(config.DeePC,'du_weight')
            du_weight = config.constraints.du_weight;
        end
        if isfield(config.DeePC,'du_min')
            du_min = config.constraints.du_min;
        end
        if isfield(config.DeePC,'du_max')
            du_max_cfg = config.constraints.du_max;
        end
    end

    dbg.use_sigma_y = use_sigma_y;
    dbg.use_sigma_u = use_sigma_u;
    dbg.enable_du_penalty = enable_du_penalty;
    dbg.enable_du_constraints = enable_du_constraints;
    dbg.du_weight = du_weight;
    dbg.du_min = du_min;
    dbg.du_max = du_max_cfg;

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

    % Tracking: ||Y_f g - r||_Q^2
    H(idx_g, idx_g) = H(idx_g, idx_g) + 2*(Y_f' * Q * Y_f);
    f(idx_g)        = f(idx_g)        - 2*(Y_f' * Q * r);

    % Input magnitude: ||U_f g||_R^2
    H(idx_g, idx_g) = H(idx_g, idx_g) + 2*(U_f' * R * U_f);

    % g regularization: lambda_g ||g||^2
    if ~isempty(lambda_g) && isfinite(lambda_g) && lambda_g > 0
        H(idx_g, idx_g) = H(idx_g, idx_g) + 2*lambda_g*eye(n_g);
    end

    % sigma penalties
    if use_sigma_y
        H(idx_sigma_y, idx_sigma_y) = H(idx_sigma_y, idx_sigma_y) + 2*lambda_y*eye(T_ini);
    end
    if use_sigma_u
        H(idx_sigma_u, idx_sigma_u) = H(idx_sigma_u, idx_sigma_u) + 2*lambda_uini*eye(T_ini);
    end

    % ---- Δu penalty exactly like SPC but with U = U_f*g ----
    % ΔU = D*U - d0 = D*(U_f*g) - d0
    if enable_du_penalty && isfinite(du_weight) && du_weight > 0
        D = eye(N);
        for k = 2:N
            D(k, k-1) = -1;
        end
        S = du_weight * eye(N);

        A = D * U_f;              % (N x n_g)
        d0 = zeros(N,1);
        d0(1) = u_prev;

        % Add (A g - d0)' S (A g - d0)
        H(idx_g, idx_g) = H(idx_g, idx_g) + 2*(A' * S * A);
        f(idx_g)        = f(idx_g)        + 2*(A' * S * (-d0));
    end

    % Stabilize
    H = H + 1e-9 * eye(n_vars);
    H = (H + H')/2;

    % ---------------- Equalities ----------------
    n_eq = 2*T_ini;
    A_eq = zeros(n_eq, n_vars);
    b_eq = zeros(n_eq, 1);

    row = 0;

    % U_p*g (+ sigma_u) = u_ini
    A_eq(row+(1:T_ini), idx_g) = U_p;
    if use_sigma_u
        A_eq(row+(1:T_ini), idx_sigma_u) = eye(T_ini);
    end
    b_eq(row+(1:T_ini)) = u_ini;
    row = row + T_ini;

    % Y_p*g (+ sigma_y) = y_ini
    A_eq(row+(1:T_ini), idx_g) = Y_p;
    if use_sigma_y
        A_eq(row+(1:T_ini), idx_sigma_y) = eye(T_ini);
    end
    b_eq(row+(1:T_ini)) = y_ini;

    % ---------------- Inequalities ----------------
    A_ineq = [];
    b_ineq = [];

    has_umin = isfinite(u_min);
    has_umax = isfinite(u_max);

    % Input bounds on U = U_f*g
    if has_umax
        A_ineq = [A_ineq; [ U_f, zeros(N, n_sigma_y+n_sigma_u)]];
        b_ineq = [b_ineq; u_max * ones(N,1)];
    end
    if has_umin
        A_ineq = [A_ineq; [-U_f, zeros(N, n_sigma_y+n_sigma_u)]];
        b_ineq = [b_ineq; -u_min * ones(N,1)];
    end

    % ---- Δu constraints like SPC: du_min <= D*U - d0 <= du_max ----
    if enable_du_constraints
        D = eye(N);
        for k = 2:N
            D(k, k-1) = -1;
        end
        d0 = zeros(N,1);
        d0(1) = u_prev;

        A = D * U_f; % (N x n_g)

        A_ineq = [A_ineq;
                  [ A,  zeros(N, n_sigma_y+n_sigma_u)];
                  [-A,  zeros(N, n_sigma_y+n_sigma_u)]];
        b_ineq = [b_ineq;
                  (du_max_cfg*ones(N,1) + d0);
                 -(du_min*ones(N,1) + d0)];
    else
        % Backward compatible: your old first-move-only constraint
        if isfinite(du_max)
            Uf1 = U_f(1,:);
            A_ineq = [A_ineq;
                      [ Uf1, zeros(1, n_sigma_y+n_sigma_u)];
                      [-Uf1, zeros(1, n_sigma_y+n_sigma_u)]];
            b_ineq = [b_ineq;
                      u_prev + du_max;
                     -u_prev + du_max];
        end
    end

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
            u_opt  = U_f * g_opt;   % deviation horizon
            status = 0;

            dbg.norm_g = norm(g_opt);
            dbg.u1     = u_opt(1);
            dbg.y1     = (Y_f * g_opt); 

            if use_sigma_y, dbg.norm_sigma_y = norm(x_opt(idx_sigma_y)); end
            if use_sigma_u, dbg.norm_sigma_u = norm(x_opt(idx_sigma_u)); end
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