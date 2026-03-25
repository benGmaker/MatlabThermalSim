function [ctrl_step, ctrl_init, meta] = spc_policy_factory(config)
%SPC_POLICY_FACTORY SPC (DeePC-equivalent predictor) + nominal MPC QP.

    % loading and centering data
    ds = load_predictive_data(config);
    u_data = ds.u_data;
    y_data = ds.T;
    dt = ds.dt;
    if config.data.centering 
    u_mean = mean(u_data); y_mean = mean(y_data);
    else
        u_mean = 0; y_mean = 0;
    end

    u_id = u_data - u_mean;
    y_id = y_data - y_mean;

    % Shared config
    P  = config.predictive.P;
    Qw = config.predictive.Q_weight;
    Rw = config.predictive.R_weight;

    % Creating deviation constraints values 
    umin_dev = config.constraints.u_min - u_mean;
    umax_dev = config.constraints.u_max - u_mean;

    ymin_dev = config.constraints.y_min - y_mean;
    ymax_dev = config.constraints.y_max - y_mean;

    % --- identification hyperparameters ---
    M  = config.SPC.M;         % past window length (Tini)

    % Fit DeePC-equivalent SPC predictor from data
    spc = spc_fit_lifted_predictor_siso(u_id, y_id, M, P);

    % Build QP (unchanged)
    qp = qp_spc_lifted_siso(spc.F, spc.Phi, P, Qw, Rw, config);

    meta = struct();
    meta.dt = dt;
    meta.u_mean = u_mean;
    meta.y_mean = y_mean;
    meta.M = M;
    meta.P = P;
    meta.Qw = Qw; meta.Rw = Rw;
    meta.spc = spc;

    ctrl_init = @init_state;
    ctrl_step = @step;

    function ctrl = init_state()
        ctrl = struct();
        ctrl.dt = dt;
        ctrl.u_mean = u_mean;
        ctrl.y_mean = y_mean;

        ctrl.qp = qp;
        ctrl.spc = spc;

        ctrl.umin_dev = umin_dev;
        ctrl.umax_dev = umax_dev;
        ctrl.ymin_dev = ymin_dev;
        ctrl.ymax_dev = ymax_dev;

        ctrl.u_prev_dev = 0;

        % history buffers for past window (length M)
        ctrl.M = M;
        ctrl.u_hist = zeros(M,1);
        ctrl.y_hist = zeros(M,1);

        % "x" is now [u_ini; y_ini] => dimension 2M
        ctrl.x = zeros(2*M, 1);
    end

    function [u_next_abs, ctrl] = step(k, y_k_abs, r_traj_abs, ctrl, config)
        % deviation signals
        y_dev = y_k_abs - ctrl.y_mean;
        r_dev = r_traj_abs(:) - ctrl.y_mean;

        % update history (shift up, append current)
        ctrl.u_hist = [ctrl.u_hist(2:end); ctrl.u_prev_dev];
        ctrl.y_hist = [ctrl.y_hist(2:end); y_dev];

        % x = [u_ini; y_ini] (Kx is identity but keep the pattern)
        w = [ctrl.u_hist; ctrl.y_hist];
        ctrl.x = ctrl.spc.Kx * w;

        % solve lifted QP
        [u0_dev, info] = ctrl.qp.solve(ctrl.x, r_dev, ctrl.u_prev_dev, ...
            ctrl.umin_dev, ctrl.umax_dev, ctrl.ymin_dev, ctrl.ymax_dev);

        % apply first input (deviation) with saturation
        u_next_dev = max(ctrl.umin_dev, min(ctrl.umax_dev, u0_dev));

        % store u for next history update
        ctrl.u_prev_dev = u_next_dev;

        % back to absolute + saturate
        u_next_abs = u_next_dev + ctrl.u_mean;
        u_next_abs = max(config.constraints.u_min, min(config.constraints.u_max, u_next_abs));

    end
end