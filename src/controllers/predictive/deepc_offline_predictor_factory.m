function [predictor_step, predictor_init, meta] = deepc_offline_predictor_factory(config)
%DEEPC_OFFLINE_PREDICTOR_FACTORY
% Builds Hankel matrices once from recorded multisine data,
% then performs 1-step-ahead prediction using DeePC-style consistency constraints.

    if nargin < 1, config = config_simulation(); end

    load('results/multisine_response_data.mat', 'multisine_data');

    u_abs = multisine_data.Q(:);
    y_abs = multisine_data.T(:);
    dt = multisine_data.params.dt;

    u_mean = mean(u_abs);
    y_mean = mean(y_abs);
    u = u_abs - u_mean;
    y = y_abs - y_mean;

    L = numel(u);

    T_ini = config.DeePC.T_ini;
    N = config.DeePC.N;

    % Regularization (use existing config terms; interpret as ridge for numerical stability)
    lambda_g = config.DeePC.lambda_g;  % used here as ridge on g
    lambda_y = config.DeePC.lambda_y;  % slack weight on past y
    lambda_u = config.DeePC.lambda_u;  % slack weight on past u

    T = T_ini + N;
    n_cols = L - T + 1;
    if n_cols <= 0
        error('Insufficient data: need L >= T_ini+N.');
    end

    U_p = zeros(T_ini, n_cols);
    Y_p = zeros(T_ini, n_cols);
    U_f = zeros(N, n_cols);
    Y_f = zeros(N, n_cols);

    for i = 1:n_cols
        U_p(:,i) = u(i:i+T_ini-1);
        Y_p(:,i) = y(i:i+T_ini-1);
        U_f(:,i) = u(i+T_ini:i+T_ini+N-1);
        Y_f(:,i) = y(i+T_ini:i+T_ini+N-1);
    end

    predictor_init = @init;
    predictor_step = @step;

    meta = struct();
    meta.dt = dt;
    meta.u_mean = u_mean;
    meta.y_mean = y_mean;
    meta.T_ini = T_ini;
    meta.N = N;
    meta.hankel_cols = n_cols;
    meta.cond_UpYp = cond([U_p; Y_p]);

    function pred = init()
        pred = struct();
        pred.dt = dt;
        pred.u_mean = u_mean;
        pred.y_mean = y_mean;
        pred.T_ini = T_ini;
        pred.N = N;

        pred.U_p = U_p; pred.Y_p = Y_p; pred.U_f = U_f; pred.Y_f = Y_f;

        pred.lambda_g = lambda_g;
        pred.lambda_y = lambda_y;
        pred.lambda_u = lambda_u;

        pred.u_hist = zeros(T_ini,1);
        pred.y_hist = zeros(T_ini,1);

        % store recorded future u sequence source (absolute) for convenience
        pred.u_recorded_dev = u; % deviation
        pred.L = L;
    end

    function [y_hat_next_abs, pred] = step(k, y_k_abs, u_k_abs, pred, config) %#ok<INUSD>
        % Update y history with measured y(k) (deviation)
        y_k = y_k_abs - pred.y_mean;
        u_k = u_k_abs - pred.u_mean;

        pred.y_hist = [pred.y_hist(2:end); y_k];
        pred.u_hist = [pred.u_hist(2:end); u_k];

        % Need a future input sequence u_future(k : k+N-1).
        % Our offline runner provides u_k (current), and we also have the dataset inside pred.
        % Note: k is 1-based; u_recorded_dev(k) corresponds to u(k).
        idx_start = k;
        idx_end = min(pred.L, k + pred.N - 1);
        u_future = pred.u_recorded_dev(idx_start:idx_end);
        if numel(u_future) < pred.N
            u_future(end+1:pred.N,1) = u_future(end);
        end

        % Solve for g to match past and future u, with slack on past for robustness.
        % Then y_future = Y_f * g.
        [y_future, ok] = deepc_predict_future(pred.U_p, pred.Y_p, pred.U_f, pred.Y_f, ...
            pred.u_hist, pred.y_hist, u_future, pred.lambda_g, pred.lambda_u, pred.lambda_y);

        if ok
            y_hat_next = y_future(1);
        else
            % fallback: persistence on y
            y_hat_next = y_k;
        end

        y_hat_next_abs = y_hat_next + pred.y_mean;
    end
end

function [y_future, ok] = deepc_predict_future(U_p, Y_p, U_f, Y_f, u_ini, y_ini, u_future, lambda_g, lambda_u, lambda_y)
% Solve a small regularized least-squares / QP for g:
%   minimize ||U_p g - u_ini||^2 * lambda_u + ||Y_p g - y_ini||^2 * lambda_y + lambda_g ||g||^2
%   subject to U_f g = u_future
%
% Then y_future = Y_f g.

    n_cols = size(U_p,2);
    N = size(U_f,1);

    % Build QP: 0.5*g'Hg + f'g
    H = 2*(lambda_u*(U_p'*U_p) + lambda_y*(Y_p'*Y_p) + lambda_g*eye(n_cols));
    f = -2*(lambda_u*(U_p'*u_ini) + lambda_y*(Y_p'*y_ini));

    Aeq = U_f;
    beq = u_future;

    opts = optimoptions('quadprog','Display','off', ...
        'MaxIterations', 200, ...
        'ConstraintTolerance', 1e-7, ...
        'OptimalityTolerance', 1e-7);

    try
        [g,~,exitflag] = quadprog(H, f, [], [], Aeq, beq, [], [], [], opts);
        if exitflag > 0 && all(isfinite(g))
            y_future = Y_f * g;
            if numel(y_future) < N
                y_future(end+1:N,1) = y_future(end);
            end
            ok = true;
            return;
        end
    catch
    end

    y_future = zeros(N,1);
    ok = false;
end