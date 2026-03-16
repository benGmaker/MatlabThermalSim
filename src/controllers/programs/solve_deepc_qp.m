function [u_opt, status, dbg] = solve_deepc_qp(U_p, Y_p, U_f, Y_f, u_ini, y_ini, r, ...
                                               Qw, Rw, ...
                                               lambda_y, lambda_g, ...
                                               u_min, u_max, du_max, u_prev, ...
                                               config)
% solve_deepc_qp
% Canonical DeePC QP aligned with reference DeePCcontroller:
%   min_g    ||Y_f g - r||_Q^2 + ||U_f g||_R^2 + lambda_g ||g||^2
%   s.t.     U_p g = u_ini              (optionally soft)
%            Y_p g + sigma_y = y_ini    (optionally soft, like reference)
%            u bounds and first-move rate bounds

    dbg = struct();

    % ---------------- Dimensions ----------------
    N     = size(U_f, 1);
    n_g   = size(U_f, 2);
    T_ini = size(U_p, 1);

    % ---------------- Config (keep same loading style) ----------------
    slack_mode = "g+u"; % your historical default
    if nargin >= 17 && isstruct(config) && isfield(config,'DeePC') && isfield(config.DeePC,'slack_mode')
        slack_mode = string(config.DeePC.slack_mode);
    end
    dbg.slack_mode = slack_mode;

    % Reference behavior: slack enabled if lambda_y is provided (non-empty) and >0.
    % In your codebase lambda_y is typically a scalar; treat lambda_y<=0 as "off".
    use_sigma_y = (~isempty(lambda_y) && isfinite(lambda_y) && lambda_y > 0);

    % Optional: softening of U_p*g = u_ini (OFF unless explicitly enabled)
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

    dbg.use_sigma_y  = use_sigma_y;
    dbg.use_sigma_u  = use_sigma_u;
    dbg.lambda_uini  = lambda_uini;

    % ---------------- Weights ----------------
    Q = Qw * eye(N);
    R = Rw * eye(N);

    % ---------------- Decision variables ----------------
    % x = [ g ; sigma_y ; sigma_u ] (sigma blocks are T_ini each if enabled)
    n_sigma_y = T_ini * use_sigma_y;
    n_sigma_u = T_ini * use_sigma_u;

    n_vars = n_g + n_sigma_y + n_sigma_u;

    idx_g = 1:n_g;
    base = n_g;

    idx_sigma_y = base + (1:n_sigma_y);
    base = base + n_sigma_y;

    idx_sigma_u = base + (1:n_sigma_u);

    % ---------------- Cost (quadprog uses 0.5*x'Hx + f'x) ----------------
    H = zeros(n_vars);
    f = zeros(n_vars, 1);

    % ||Y_f g - r||_Q^2  = g' Y_f'QY_f g - 2 r'QY_f g + const
    H(idx_g, idx_g) = H(idx_g, idx_g) + 2*(Y_f' * Q * Y_f);
    f(idx_g)        = f(idx_g)        - 2*(Y_f' * Q * r);

    % ||U_f g||_R^2
    H(idx_g, idx_g) = H(idx_g, idx_g) + 2*(U_f' * R * U_f);

    % lambda_g ||g||^2
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

    % Stabilize numerically
    H = H + 1e-9 * eye(n_vars);
    H = (H + H')/2;

    % ---------------- Equalities ----------------
    % U_p g + sigma_u = u_ini   (sigma_u optional)
    % Y_p g + sigma_y = y_ini   (sigma_y optional)
    n_eq = 2*T_ini;

    A_eq = zeros(n_eq, n_vars);
    b_eq = zeros(n_eq, 1);

    row = 0;

    % (1) U_p*g (+ sigma_u) = u_ini
    A_eq(row+(1:T_ini), idx_g) = U_p;
    if use_sigma_u
        A_eq(row+(1:T_ini), idx_sigma_u) = eye(T_ini);
    end
    b_eq(row+(1:T_ini)) = u_ini;
    row = row + T_ini;

    % (2) Y_p*g (+ sigma_y) = y_ini
    A_eq(row+(1:T_ini), idx_g) = Y_p;
    if use_sigma_y
        A_eq(row+(1:T_ini), idx_sigma_y) = eye(T_ini);
    end
    b_eq(row+(1:T_ini)) = y_ini;
    row = row + T_ini;

    % ---------------- Inequalities ----------------
    A_ineq = [];
    b_ineq = [];

    % Helper to treat "no bound" cleanly
    has_umin = isfinite(u_min);
    has_umax = isfinite(u_max);

    % Input bounds on U_f*g
    if has_umax
        A_ineq = [A_ineq; [ U_f, zeros(N, n_sigma_y+n_sigma_u)]];
        b_ineq = [b_ineq; u_max * ones(N,1)];
    end
    if has_umin
        A_ineq = [A_ineq; [-U_f, zeros(N, n_sigma_y+n_sigma_u)]];
        b_ineq = [b_ineq; -u_min * ones(N,1)];
    end

    % First move rate constraint on u(1) = (e1'*U_f)*g
    if isfinite(du_max)
        Uf1 = U_f(1,:); % 1-by-n_g
        A_ineq = [A_ineq;
                  [ Uf1, zeros(1, n_sigma_y+n_sigma_u)];
                  [-Uf1, zeros(1, n_sigma_y+n_sigma_u)]];
        b_ineq = [b_ineq;
                  u_prev + du_max;
                 -u_prev + du_max];
    end

    % ---------------- Solve ----------------
    options = optimoptions('quadprog', ...
        'Display', 'off', ...
        'MaxIterations', 200, ...
        'ConstraintTolerance', 1e-8, ...
        'OptimalityTolerance', 1e-8);

    try
        [x_opt, ~, exitflag, output] = quadprog(H, f, A_ineq, b_ineq, A_eq, b_eq, [], [], [], options);
        dbg.exitflag = exitflag;
        dbg.output   = output;

        if exitflag > 0
            g_opt  = x_opt(idx_g);
            u_pred = U_f * g_opt;
            y_pred = Y_f * g_opt;

            u_opt  = u_pred;   % your API expects full horizon
            status = 0;

            dbg.norm_g = norm(g_opt);
            dbg.u1     = u_pred(1);
            dbg.y1     = y_pred(1);

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