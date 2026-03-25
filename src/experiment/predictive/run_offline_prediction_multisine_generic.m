function results = run_offline_prediction_multisine_generic(config, predictor_step, predictor_init)
%RUN_OFFLINE_PREDICTION_MULTISINE_GENERIC
% Offline predictive-performance evaluation on recorded multisine data.
%
% predictor_init: () -> predictor_state struct
% predictor_step:  [y_hat_next_abs, pred] = predictor_step(k, y_k_abs, u_k_abs, pred, config)
%
% Predicts y(k+1) from measured y(k), u(k). Compares y_hat vs measured y.

    config = config_simulation();
    config.dataset_choice = "multisine2"; % Hard set to different dataset
    if nargin < 3 || isempty(predictor_init)
        predictor_init = @() struct();
    end



    ds = load_predictive_data(config);
    u_meas = ds.Q(:);   % absolute
    y_meas = ds.T(:);   % absolute
    dt = ds.params.dt;

    n_steps = numel(y_meas);
    if numel(u_meas) ~= n_steps
        error('Recorded u and y lengths differ (%d vs %d).', numel(u_meas), n_steps);
    end

    t = (0:n_steps-1)' * dt;

    y_hat = nan(n_steps,1);
    y_hat(1) = y_meas(1);

    pred = predictor_init();

    for k = 1:n_steps-1
        [y_hat(k+1), pred] = predictor_step(k, y_meas(k), u_meas(k), pred, config);
    end

    % Error on k>=2
    e = y_meas - y_hat;
    e_eval = e(2:end);

    results = struct();
    results.t = t;
    results.y_meas = y_meas;
    results.u_meas = u_meas;
    results.y_hat = y_hat;
    results.error = e;

    results.metrics = struct();
    results.metrics.MAE  = mean(abs(e_eval), 'omitnan');
    results.metrics.RMSE = sqrt(mean(e_eval.^2, 'omitnan'));
    results.metrics.ISE  = sum(e_eval.^2, 'omitnan') * dt;
    results.metrics.IAE  = sum(abs(e_eval), 'omitnan') * dt;

    denom = norm(y_meas(2:end) - mean(y_meas(2:end)), 2);
    if denom > 0
        results.metrics.fit_percent = 100 * (1 - norm(e_eval,2)/denom);
    else
        results.metrics.fit_percent = NaN;
    end
end