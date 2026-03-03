function [predictor_step, predictor_init, meta] = dmc_offline_predictor_factory(config)
%DMC_OFFLINE_PREDICTOR_FACTORY
% Offline 1-step predictor based on DMC step response S (FIR on Δu).
%
% Uses same step_response_source as your DMC config:
%   - measured: results/step_response_data.mat
%   - ss_model / tf_model: results/identified_models.mat

    if nargin < 1, config = config_simulation(); end

    % Load means + dt + models
    data = load('results/identified_models.mat', 'identified_models');
    identified_models = data.identified_models;

    dt = identified_models.dt;
    u_mean = identified_models.u_mean;
    y_mean = identified_models.y_mean;

    N = config.DMC.N;

    % Get step response S(1..N)
    switch config.DMC.step_response_source
        case 'measured'
            S = get_measured_step_response_local(N);
        case 'ss_model'
            S = get_model_step_response_local(identified_models.sys_ss, N, dt);
        case 'tf_model'
            S = get_model_step_response_local(identified_models.sys_tf, N, dt);
        otherwise
            error('Unknown step response source: %s', config.DMC.step_response_source);
    end
    S = S(:);

    predictor_init = @init;
    predictor_step = @step;

    meta = struct();
    meta.dt = dt;
    meta.u_mean = u_mean;
    meta.y_mean = y_mean;
    meta.S = S;
    meta.N = N;
    meta.step_response_source = config.DMC.step_response_source;

    function pred = init()
        pred = struct();
        pred.dt = dt;
        pred.u_mean = u_mean;
        pred.y_mean = y_mean;
        pred.S = S;
        pred.N = N;

        % Keep last N Δu in deviation variables (most recent at end)
        pred.du_hist = zeros(N,1);

        % For Δu computation
        pred.u_prev_dev = 0;
        pred.is_initialized = false;
    end

    function [y_hat_next_abs, pred] = step(k, y_k_abs, u_k_abs, pred, config) %#ok<INUSD>
        % Work in deviation variables
        u_k_dev = u_k_abs - pred.u_mean;

        if ~pred.is_initialized
            % Initialize using first sample
            pred.u_prev_dev = u_k_dev;
            pred.is_initialized = true;
        end

        du_k = u_k_dev - pred.u_prev_dev;
        pred.u_prev_dev = u_k_dev;

        % Update Δu history (shift up, append newest at end)
        pred.du_hist = [pred.du_hist(2:end); du_k];

        % Predict next output deviation using FIR on Δu
        % y_dev_hat(k+1) = sum_{i=1..N} S(i)*du(k+1-i)
        % Our du_hist(end) is du(k), so map:
        %   i=1 uses du(k), i=2 uses du(k-1), ...
        du_recent_to_old = flipud(pred.du_hist);   % [du(k); du(k-1); ...]
        y_hat_next_dev = pred.S(:).' * du_recent_to_old;

        y_hat_next_abs = y_hat_next_dev + pred.y_mean;
    end
end

% ---- Local helpers (copy of your existing ones, trimmed)
function S = get_measured_step_response_local(N)
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
        S = [y_unit_step; y_unit_step(end)*ones(N-length(y_unit_step),1)];
    end
end

function S = get_model_step_response_local(sys, N, dt)
    if ~isdt(sys)
        sys = c2d(sys, dt, 'zoh');
    end
    [y_step, ~] = step(sys, (0:N-1)*dt);
    S = y_step(:);
    if numel(S) < N
        S(end+1:N,1) = S(end);
    else
        S = S(1:N);
    end
end