function [ctrl_step, ctrl_init, meta] = mpc_policy_factory(config)
%MPC_POLICY_FACTORY Quadprog-based MPC policy (nominal MPC matching report).

    if nargin < 1
        config = config_simulation();
    end

    data = load('results/data/identified_models.mat', 'identified_models');
    identified_models = data.identified_models;

    sys_ss = identified_models.sys_ss;
    dt = identified_models.dt;
    u_mean = identified_models.u_mean;
    y_mean = identified_models.y_mean;

    % shared config
    P  = config.MPC.P
    Qw = config.predictive.Q_weight;
    Rw = config.predictive.R_weight;

    umin_dev = config.constraints.u_min - u_mean;
    umax_dev = config.constraints.u_max - u_mean;

    % optional y constraints 
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

        % ---- Add observer + previous input ----
        ctrl.u_prev_dev = 0;

        A = qp.A; C = qp.C;
        % Place observer poles faster than plant poles (heuristic)
        polesA = eig(A);
        desired = 0.5 * polesA;
        % Ensure stable and inside unit circle
        desired = min(desired, 0.8);
        desired = max(desired, -0.8);

        % For SISO: place for (A',C') then transpose gain back
        try
            ctrl.L = place(A', C', desired).';
        catch
            % Fallback if place fails (e.g., non-observable numeric issues)
            ctrl.L = zeros(size(A,1), size(C,1));
        end
        % --------------------------------------
    end

    function [u_next_abs, ctrl] = step(k, y_k_abs, r_traj_abs, ctrl, config)
        % Deviation signals
        y_dev = y_k_abs - ctrl.y_mean;
        r_dev = r_traj_abs(:) - ctrl.y_mean;

        % ---- Observer update: predict then correct ----
        x_pred = ctrl.qp.A*ctrl.x + ctrl.qp.B*ctrl.u_prev_dev;
        y_pred = ctrl.qp.C*x_pred + ctrl.qp.D*ctrl.u_prev_dev;
        ctrl.x = x_pred + ctrl.L*(y_dev - y_pred);
        % ---------------------------------------------

        % Solve nominal MPC
        [u0_dev, info] = ctrl.qp.solve(ctrl.x, r_dev, ctrl.umin_dev, ctrl.umax_dev, ctrl.ymin_dev, ctrl.ymax_dev);

        % Apply first input (deviation)
        u_next_dev = max(ctrl.umin_dev, min(ctrl.umax_dev, u0_dev));

        % store previous input for next observer update
        ctrl.u_prev_dev = u_next_dev;

        % Convert to absolute and clamp
        u_next_abs = u_next_dev + ctrl.u_mean;
        u_next_abs = max(config.constraints.u_min, min(config.constraints.u_max, u_next_abs));
    end
end