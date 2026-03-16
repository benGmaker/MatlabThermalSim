function [u_opt, status, dbg] = solve_deepc_qp(U_p, Y_p, U_f, Y_f, u_ini, y_ini, r, ...
                                               Qw, Rw, ...
                                               lambda_y, lambda_g, ...
                                               u_min, u_max, du_max, u_prev, ...
                                               config)
% Two modes (selected by config.DeePC.slack_mode):
%   "g"   : slack/regularization on g only (adds s_g, penalizes it)
%   "g+u" : slack on g (s_g) AND slack on past constraints (sigma_y, sigma_u

    dbg = struct();

    [N, n_cols] = size(U_f);
    T_ini = size(U_p, 1);

    % ---------- Mode selection ----------
    slack_mode = "g+u"; % default
    if nargin >= 17 && isstruct(config) && isfield(config,'DeePC') && isfield(config.DeePC,'slack_mode')
        slack_mode = string(config.DeePC.slack_mode);
    end

    use_sigma = (slack_mode == "g+u");  % sigma_y/sigma_u included only in this mode
    use_sg    = true;                  % always include s_g in both modes per your request

    dbg.slack_mode = slack_mode;
    dbg.use_sigma = use_sigma;
    dbg.use_sg = use_sg;

    % ---------- Penalties ----------
    % s_g penalty (new): make this your main stabilizer for g
    lambda_sg = 0;
    if isstruct(config) && isfield(config,'DeePC') && isfield(config.DeePC,'lambda_sg')
        lambda_sg = config.DeePC.lambda_sg;
    end
    % Backward-compatible fallback: if user didn't set lambda_sg, reuse lambda_g
    if lambda_sg == 0
        lambda_sg = lambda_g;
    end
    dbg.lambda_sg = lambda_sg;

    % Optional Δu smoothing (off by default)
    alpha_du = 0;
    if isstruct(config) && isfield(config,'DeePC') && isfield(config.DeePC,'alpha_du')
        alpha_du = config.DeePC.alpha_du;
    end
    dbg.alpha_du = alpha_du;

    % ---------- Decision variables ----------
    % Base variables: g and u
    n_g = n_cols;
    n_u = N;

    % s_g always included (per user request)
    n_sg = n_g;

    % sigma vars only in "g+u"
    n_sigma_y = T_ini * use_sigma;
    n_sigma_u = T_ini * use_sigma;

    n_vars = n_g + n_u + n_sg + n_sigma_y + n_sigma_u;

    idx_g = 1:n_g;
    idx_u = n_g + (1:n_u);
    idx_sg = n_g + n_u + (1:n_sg);

    base = n_g + n_u + n_sg;
    idx_sigma_y = base + (1:n_sigma_y);
    idx_sigma_u = base + n_sigma_y + (1:n_sigma_u);

    % ---------- Cost ----------
    H = zeros(n_vars);
    f = zeros(n_vars, 1);

    Q = Qw * eye(N);
    R = Rw * eye(N);

    % Tracking: ||Y_f*g - r||_Q^2
    H(idx_g, idx_g) = H(idx_g, idx_g) + 2 * (Y_f' * Q * Y_f);
    f(idx_g)        = f(idx_g)        - 2 * (Y_f' * Q * r);

    % Input effort: ||u||_R^2
    H(idx_u, idx_u) = H(idx_u, idx_u) + 2 * R;

    % g slack penalty: ||s_g||^2 * lambda_sg
    if lambda_sg > 0
        H(idx_sg, idx_sg) = H(idx_sg, idx_sg) + 2 * lambda_sg * eye(n_sg);
    end

    % sigma penalties (only in g+u mode)
    if use_sigma
        if lambda_y > 0
            H(idx_sigma_y, idx_sigma_y) = H(idx_sigma_y, idx_sigma_y) + 2 * lambda_y * eye(T_ini);
        end
        if lambda_g > 0
            % Here lambda_g is penalty for sigma_u (keep your original meaning)
            H(idx_sigma_u, idx_sigma_u) = H(idx_sigma_u, idx_sigma_u) + 2 * lambda_g * eye(T_ini);
        end
    end

    % Optional Δu smoothness (helps oscillations)
    if alpha_du > 0
        D = eye(N) - [zeros(1,N); eye(N-1), zeros(N-1,1)];
        d0 = zeros(N,1); d0(1) = u_prev;

        H(idx_u, idx_u) = H(idx_u, idx_u) + 2*alpha_du*(D'*D);
        f(idx_u)        = f(idx_u)        - 2*alpha_du*(D'*d0);
    end

    % Regularization for numerical stability
    H = H + 1e-9 * eye(n_vars);
    H = (H + H')/2;

    % ---------- Equalities ----------
    n_eq = N ...                    % U_f*g = u
         + n_sg ...                 % g - s_g = 0
         + T_ini ...                % U_p*g = u_ini (+ sigma_u)
         + T_ini;                   % Y_p*g = y_ini (+ sigma_y)

    A_eq = zeros(n_eq, n_vars);
    b_eq = zeros(n_eq, 1);

    row = 0;

    % (1) U_f*g = u
    A_eq(row+(1:N), idx_g) = U_f;
    A_eq(row+(1:N), idx_u) = -eye(N);
    % b=0 already
    row = row + N;

    % (2) g - s_g = 0
    A_eq(row+(1:n_g), idx_g) = eye(n_g);
    A_eq(row+(1:n_g), idx_sg) = -eye(n_g);
    row = row + n_g;

    % (3) U_p*g = u_ini (+ sigma_u)
    A_eq(row+(1:T_ini), idx_g) = U_p;
    if use_sigma
        A_eq(row+(1:T_ini), idx_sigma_u) = -eye(T_ini);
    end
    b_eq(row+(1:T_ini)) = u_ini;
    row = row + T_ini;

    % (4) Y_p*g = y_ini (+ sigma_y)
    A_eq(row+(1:T_ini), idx_g) = Y_p;
    if use_sigma
        A_eq(row+(1:T_ini), idx_sigma_y) = -eye(T_ini);
    end
    b_eq(row+(1:T_ini)) = y_ini;
    row = row + T_ini;

    % ---------- Inequalities ----------
    A_ineq = [];
    b_ineq = [];

    % Input bounds on u
    if isfinite(u_min) || isfinite(u_max)
        A_u = [zeros(2*N, n_g), [eye(N); -eye(N)], zeros(2*N, n_sg + n_sigma_y + n_sigma_u)];
        b_u = [u_max * ones(N, 1); -u_min * ones(N, 1)];
        A_ineq = [A_ineq; A_u];
        b_ineq = [b_ineq; b_u];
    end

    % First-move rate constraint (same as your original)
    if isfinite(du_max)
        A_rate = [zeros(2, n_g), [1, zeros(1, N-1); -1, zeros(1, N-1)], zeros(2, n_sg + n_sigma_y + n_sigma_u)];
        b_rate = [u_prev + du_max; -u_prev + du_max];
        A_ineq = [A_ineq; A_rate];
        b_ineq = [b_ineq; b_rate];
    end

    % ---------- Solve ----------
    options = optimoptions('quadprog', ...
        'Display', 'off', ...
        'MaxIterations', 50, ...
        'ConstraintTolerance', 1e-7, ...
        'OptimalityTolerance', 1e-7);

    try
        [x_opt, ~, exitflag, output] = quadprog(H, f, A_ineq, b_ineq, A_eq, b_eq, [], [], [], options);
        dbg.exitflag = exitflag;
        dbg.output = output;

        if exitflag > 0
            u_opt = x_opt(idx_u);
            status = 0;

            dbg.norm_g = norm(x_opt(idx_g));
            dbg.norm_sg = norm(x_opt(idx_sg));
            if use_sigma
                dbg.norm_sigma_y = norm(x_opt(idx_sigma_y));
                dbg.norm_sigma_u = norm(x_opt(idx_sigma_u));
            end
        else
            u_opt = zeros(N, 1);
            status = -1;
        end
    catch ME
        warning("quadprog failed: %s", ME.message);
        u_opt = zeros(N, 1);
        status = -1;
    end
end


