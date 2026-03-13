function [results, ctrl_final] = run_closed_loop_generic(config, controller, controller_init)
%RUN_CLOSED_LOOP_GENERIC Generic closed-loop experiment runner.
%
% controller_init: function handle returning controller_state struct
% controller: function handle [u_next, controller_state] = controller(k, y_k, r_traj, controller_state, config)

    if nargin < 3 || isempty(controller_init)
        controller_init = @() struct();
    end

    dt = config.simulation.dt;
    t_sim = config.simulation.t_sim;
    t = (0:dt:t_sim).';
    n_steps = numel(t);

    r_abs = build_setpoint_profile(config);

    y = zeros(n_steps,1);
    u = zeros(n_steps,1);
    y(1) = r_abs(1);
    u(1) = 0;

    params = struct();
    params.dt = dt;
    params.T0 = y(1);
    params.model_type = config.thermal_model.model_type;

    ctrl = controller_init();

    for k = 1:n_steps-1
        P_preview = config.predictive.P; % Prediction horizon

        r_traj = r_abs(k:min(k+P_preview-1, n_steps));
        if numel(r_traj) < P_preview
            r_traj(end+1:P_preview,1) = r_traj(end);
        end

        [u_next, ctrl] = controller(k, y(k), r_traj, ctrl, config);

        % Hard input bounds
        u_next = max(config.constraints.u_min, min(config.constraints.u_max, u_next));
        u(k+1) = u_next;

        params.T0 = y(k);
        params.t_final = dt;
        Q_func = @(tt) u(k+1);
        [T_sim, ~, ~] = thermal_model(Q_func, params);
        y(k+1) = T_sim(end);
    end

    results = struct();
    results.t = t;
    results.y = y;
    results.u = u;
    results.setpoint = r_abs;
    results.error = r_abs - y;
    results.metrics = compute_metrics(results.error, u, dt);

    % ---- Command line performance stats ----
    fprintf('\n----------------------------------------\n');
    if isfield(config, 'controller_name')
        fprintf('Closed-loop run complete: %s\n', string(config.controller_name));
    else
        fprintf('Closed-loop run complete\n');
    end
    fprintf('Samples: %d, dt: %.4g s, t_sim: %.4g s\n', n_steps, dt, t_sim);

    m = results.metrics;
    if isstruct(m)
        if isfield(m,'MAE'), fprintf('MAE            : %.6g\n', m.MAE); end
        if isfield(m,'RMSE'), fprintf('RMSE           : %.6g\n', m.RMSE); end
        if isfield(m,'ISE'), fprintf('ISE            : %.6g\n', m.ISE); end
        if isfield(m,'IAE'), fprintf('IAE            : %.6g\n', m.IAE); end
        if isfield(m,'control_effort'), fprintf('Control effort : %.6g\n', m.control_effort); end
    end

    e = results.error;
    if ~isempty(e)
        fprintf('Error stats    : min %.6g | mean %.6g | max %.6g\n', min(e), mean(e), max(e));
    end
    fprintf('y stats        : min %.6g | mean %.6g | max %.6g\n', min(y), mean(y), max(y));
    fprintf('u stats        : min %.6g | mean %.6g | max %.6g\n', min(u), mean(u), max(u));
    fprintf('----------------------------------------\n');
    % ---------------------------------------

    ctrl_final = ctrl;
end