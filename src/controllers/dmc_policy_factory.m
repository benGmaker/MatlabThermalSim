function [ctrl_step, ctrl_init, meta] = dmc_policy_factory(config)
%DMC_POLICY_FACTORY Returns controller step function + initializer + meta.
    
    % todo remove upgrade model u_mean y_mean dt to the one of the step
    % response
    % Load model once
    data = load('results/data/identified_models.mat', 'identified_models');
    identified_models = data.identified_models;

    dt = identified_models.dt;
    u_mean = identified_models.u_mean;
    y_mean = identified_models.y_mean;

    % DMC params
    P = config.predictive.P;
    M = config.predictive.M;
    N = config.DMC.N; 

    Q_weight = config.predictive.Q_weight;
    R_weight = config.predictive.R_weight;

    % Constraints
    u_min = config.constraints.u_min;
    u_max = config.constraints.u_max;
    du_max = config.constraints.du_max;

    % Step response
    S = get_measured_step_response(N, dt, u_mean, y_mean);

    % Build G
    G = zeros(P, M);
    for i = 1:P
        for j = 1:M
            if i >= j
                idx = i - j + 1;
                G(i,j) = S(min(idx, N));
            end
        end
    end

    Q_mat = Q_weight * eye(P);
    R_mat = R_weight * eye(M);
    K_dmc = (G' * Q_mat * G + R_mat) \ (G' * Q_mat);

    controller_init = @() struct( ...
        'dt', dt, ...
        'u_mean', u_mean, ...
        'y_mean', y_mean, ...
        'P', P, ...
        'M', M, ...
        'N', N, ...
        'S', S(:), ...
        'K_dmc', K_dmc, ...
        'u_prev', u_mean, ...          % important: start at mean (absolute)
        'du_hist', zeros(N-1, 1) ...   % Δu(k-1), Δu(k-2), ...
    );

    % ---- outputs expected by run_closed_loop_dmc
    ctrl_step = @step;
    ctrl_init = controller_init;

    % ---- meta for legacy results saving (step response etc.)
    meta = struct();
    meta.dt = dt;
    meta.u_mean = u_mean;
    meta.y_mean = y_mean;
    meta.S = S(:);
    meta.G = G;
    meta.K_dmc = K_dmc;
    meta.P = P;
    meta.M = M;
    meta.N = N;

    function [u_next, ctrl] = step(k, y_k, r_traj_abs, ctrl, config)
    % --- Convenience
    P = ctrl.P;
    M = ctrl.M;
    N = ctrl.N;
    S = ctrl.S(:);

    % --- Deviation variables
    y_dev = y_k - ctrl.y_mean;

    % --- Build P-long setpoint preview in deviation
    sp = r_traj_abs(:) - ctrl.y_mean;
    if numel(sp) < P
        sp(end+1:P,1) = sp(end);
    elseif numel(sp) > P
        sp = sp(1:P);
    end

    % ==========================================================
    % 1) Free response from past Δu history 
    %
    % We predict future output increments due to past moves:
    %   y_free(i) = y(k) + sum_{j=1}^{N-1} (S(i+j) - S(j)) * du_hist(j)
    %
    % Where du_hist(1)=Δu(k-1), du_hist(2)=Δu(k-2), ...
    % ==========================================================
    du_hist = ctrl.du_hist;                % (N-1)x1
    y_free = y_dev * ones(P,1);            % base: hold at current y_dev

    for i = 1:P
        acc = 0.0;
        for j = 1:(N-1)
            % Indices into step response, with saturation at N
            s_ij = S(min(N, i + j));       % S(i+j)
            s_j  = S(j);                   % S(j)
            acc = acc + (s_ij - s_j) * du_hist(j);
        end
        y_free(i) = y_free(i) + acc;
    end

    % --- Error for optimizer
    e = sp - y_free;

    % ==========================================================
    % 2) Unconstrained DMC move (same K you already compute)
    % ==========================================================
    du_seq = ctrl.K_dmc * e;
    du0_cmd = du_seq(1);

    % --- Rate limit on Δu
    du_max = config.constraints.du_max;
    du0_cmd = max(-du_max, min(du_max, du0_cmd));

    % ==========================================================
    % 3) Apply move, then saturate u, then compute APPLIED Δu
    % ==========================================================
    u_min = config.constraints.u_min;
    u_max = config.constraints.u_max;

    u_prev = ctrl.u_prev;                 % absolute
    u_cmd  = u_prev + du0_cmd;            % absolute command

    u_next = max(u_min, min(u_max, u_cmd));

    % Actual applied increment (after saturation)
    du0_applied = u_next - u_prev;

    % --- Update stored previous u
    ctrl.u_prev = u_next;

    % ==========================================================
    % 4) Update Δu history with the APPLIED move
    %    du_hist(1)=Δu(k), du_hist(2)=Δu(k-1), ...
    % ==========================================================
    if N > 1
        ctrl.du_hist = [du0_applied; du_hist(1:end-1)];
    end
end
end

