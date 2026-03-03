function [ctrl_step, ctrl_init, meta] = dmc_policy_factory(config)
%DMC_POLICY_FACTORY Returns controller step function + initializer + meta.

    % Load model once
    data = load('results/identified_models.mat', 'identified_models');
    identified_models = data.identified_models;

    dt = identified_models.dt;
    u_mean = identified_models.u_mean;
    y_mean = identified_models.y_mean;

    % DMC params
    P = config.DMC.P;
    M = config.DMC.M;
    N = config.DMC.N;

    Q_weight = config.DMC.Q_weight;
    R_weight = config.DMC.R_weight;

    % Constraints
    u_min = config.constraints.u_min;
    u_max = config.constraints.u_max;
    du_max = config.constraints.du_max;

    % Step response
    switch config.DMC.step_response_source
        case 'measured'
            S = get_measured_step_response(N, dt, u_mean, y_mean);
        case 'ss_model'
            S = get_model_step_response(identified_models.sys_ss, N, dt);
        case 'tf_model'
            S = get_model_step_response(identified_models.sys_tf, N, dt);
        otherwise
            error('Unknown step response source: %s', config.DMC.step_response_source);
    end

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
        'K_dmc', K_dmc, ...
        'u_prev', 0 ...
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
        %#ok<INUSD> k config

        % Deviation variables
        y_dev = y_k - ctrl.y_mean;

        % Need P-long preview; if provided shorter, pad
        P_local = ctrl.P;
        sp = r_traj_abs(:) - ctrl.y_mean;
        if numel(sp) < P_local
            sp(end+1:P_local,1) = sp(end);
        elseif numel(sp) > P_local
            sp = sp(1:P_local);
        end

        y_free = y_dev * ones(P_local,1);
        e = sp - y_free;

        du_seq = ctrl.K_dmc * e;
        du0 = du_seq(1);

        % Rate limit
        du0 = max(-du_max, min(du_max, du0));

        % Apply move in deviation, convert to absolute, saturate
        u_dev_prev = ctrl.u_prev - ctrl.u_mean;
        u_dev_new = u_dev_prev + du0;
        u_next = u_dev_new + ctrl.u_mean;

        u_next = max(u_min, min(u_max, u_next));
        ctrl.u_prev = u_next;
    end
end

% --- reuse your existing helpers (can move to common/ if you want)
function S = get_measured_step_response(N, dt, u_mean, y_mean) %#ok<INUSD>
    data = load('results/step_response_data.mat');
    step_data = data.step_data;

    if isfield(step_data, 'step_idx')
        step_idx = step_data.step_idx;
    else
        step_idx = find(step_data.Q > mean(step_data.Q), 1, 'first');
    end

    if isfield(step_data, 'step_amplitude')
        step_magnitude = step_data.step_amplitude;
    else
        u_before = mean(step_data.Q(1:max(1,step_idx-1)));
        u_after = mean(step_data.Q(step_idx:min(step_idx+10, end)));
        step_magnitude = u_after - u_before;
    end

    y_before = mean(step_data.T(1:max(1,step_idx-1)));
    y_response = step_data.T(step_idx:end) - y_before;
    y_unit_step = y_response / step_magnitude;

    if length(y_unit_step) >= N
        S = y_unit_step(1:N);
    else
        S = [y_unit_step; y_unit_step(end) * ones(N - length(y_unit_step), 1)];
    end
    S = S(:);
end

function S = get_model_step_response(sys, N, dt)
    [y_step, ~] = step(sys, (0:N-1)*dt);
    S = y_step(:);
    if numel(S) < N
        S(end+1:N,1) = S(end);
    else
        S = S(1:N);
    end
end