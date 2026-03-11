function [ctrl_step, ctrl_init, meta] = mpc_policy_factory(config)
%MPC_POLICY_FACTORY Quadprog-based MPC policy (nominal MPC matching report).

    if nargin < 1
        config = config_simulation();
    end

    data = load('results/data/identified_models.mat', 'identified_models');
    identified_models = data.identified_models;

    sys = identified_models.sys_tf;
    dt = identified_models.dt;
    u_mean = identified_models.u_mean;
    y_mean = identified_models.y_mean;

    if ~isdt(sys)
        sys_d = c2d(sys, dt, 'zoh');
    else
        sys_d = sys;
    end
    sys_ss = ss(sys_d);

    % shared config
    P  = config.predictive.P;           % prediction horizon N in report
    Qw = config.predictive.Q_weight;
    Rw = config.predictive.R_weight;

    umin_dev = config.constraints.u_min - u_mean;
    umax_dev = config.constraints.u_max - u_mean;

    % Output constraints (required if you want to match report's y in Y)
    % If you don't have these fields yet, either add them to config or disable y constraints.
    if isfield(config.constraints,'y_min') && isfield(config.constraints,'y_max')
        ymin_dev = config.constraints.y_min - y_mean;
        ymax_dev = config.constraints.y_max - y_mean;
        enable_y_constraints = true;
    else
        ymin_dev = -Inf;
        ymax_dev =  Inf;
        enable_y_constraints = false;
    end

    qp_opts = struct('enable_y_constraints', enable_y_constraints);
    qp = qp_mpc_siso(sys_ss, P, Qw, Rw, qp_opts);

    meta = struct();
    meta.dt = dt;
    meta.u_mean = u_mean;
    meta.y_mean = y_mean;
    meta.sys_d = sys_d;
    meta.sys_ss = sys_ss;
    meta.P = P;
    meta.Qw = Qw; meta.Rw = Rw;
    meta.enable_y_constraints = enable_y_constraints;

    ctrl_init = @init_state;
    ctrl_step = @step;

    function ctrl = init_state()
        ctrl = struct();
        ctrl.dt = dt;
        ctrl.u_mean = u_mean;
        ctrl.y_mean = y_mean;

        ctrl.sys_ss = sys_ss;
        ctrl.qp = qp;

        ctrl.x = zeros(qp.nx, 1);

        ctrl.umin_dev = umin_dev;
        ctrl.umax_dev = umax_dev;
        ctrl.ymin_dev = ymin_dev;
        ctrl.ymax_dev = ymax_dev;
    end

    function [u_next_abs, ctrl] = step(k, y_k_abs, r_traj_abs, ctrl, config)
        %#ok<INUSD> k config
        % Deviation signals
        r_dev = r_traj_abs(:) - ctrl.y_mean;

        % Solve nominal MPC: decision variable is predicted absolute U sequence
        [u0_dev, info] = ctrl.qp.solve(ctrl.x, r_dev, ctrl.umin_dev, ctrl.umax_dev, ctrl.ymin_dev, ctrl.ymax_dev); %#ok<NASGU>

        % Apply first input (deviation)
        u_next_dev = u0_dev;
        u_next_dev = max(ctrl.umin_dev, min(ctrl.umax_dev, u_next_dev));

        % Time update model state
        A = ctrl.qp.A; B = ctrl.qp.B;
        ctrl.x = A*ctrl.x + B*u_next_dev;

        % Convert to absolute and clamp
        u_next_abs = u_next_dev + ctrl.u_mean;
        u_next_abs = max(config.constraints.u_min, min(config.constraints.u_max, u_next_abs));
    end
end