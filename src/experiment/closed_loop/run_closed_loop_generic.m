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
        if isfield(config, 'ref_preview_horizon')
            P_preview = config.ref_preview_horizon;
        else
            P_preview = 1;
        end

        r_traj = r_abs(k:min(k+P_preview-1, n_steps));
        if numel(r_traj) < P_preview
            r_traj(end+1:P_preview,1) = r_traj(end);
        end

        [u_next, ctrl] = controller(k, y(k), r_traj, ctrl, config);

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

    ctrl_final = ctrl;
end