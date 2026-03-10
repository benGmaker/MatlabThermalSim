function [ctrl_step, ctrl_init, meta] = mpc_policy_factory(config)
%MPC_POLICY_FACTORY Quadprog-based MPC policy.

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
    P = config.predictive.P;
    M = config.predictive.M;
    Qw = config.predictive.Q_weight;
    Rw = config.predictive.R_weight;

    umin_dev = config.constraints.u_min - u_mean;
    umax_dev = config.constraints.u_max - u_mean;
    du_max = config.constraints.du_max;

    qp_opts = struct('enable_integrator', config.enable_integrator);
    qp = qp_mpc_siso(sys_ss, P, M, Qw, Rw, qp_opts);

    meta = struct();
    meta.dt = dt;
    meta.u_mean = u_mean;
    meta.y_mean = y_mean;
    meta.sys_d = sys_d;
    meta.sys_ss = sys_ss;
    meta.P = P; meta.M = M;
    meta.Qw = Qw; meta.Rw = Rw;
    meta.enable_integrator = qp.enable_integrator;

    ctrl_init = @init_state;
    ctrl_step = @step;

    function ctrl = init_state()
        ctrl = struct();
        ctrl.dt = dt;
        ctrl.u_mean = u_mean;
        ctrl.y_mean = y_mean;

        ctrl.sys_ss = sys_ss;
        ctrl.qp = qp;

        % Augmented state size depends on qp integrator option
        ctrl.x = zeros(qp.nx, 1);

        % IMPORTANT: match run_closed_loop_generic initial u(1)=0 (absolute)
        u0_abs = 0;
        ctrl.u_prev_dev = u0_abs - u_mean;

        ctrl.umin_dev = umin_dev;
        ctrl.umax_dev = umax_dev;
        ctrl.du_max = du_max;
    end

    function [u_next_abs, ctrl] = step(k, y_k_abs, r_traj_abs, ctrl, config) 
        % Deviation signals
        y_dev = y_k_abs - ctrl.y_mean;
        r_dev = r_traj_abs(:) - ctrl.y_mean;

        % ---- Measurement update for integrator state (if enabled)
        % Our augmentation sets last state as an integrator-like term.
        % We update it using actual measured y and reference (first element).
        if ctrl.qp.enable_integrator
            % x = [xplant; xi], update xi <- xi + (r - y)
            xi_idx = numel(ctrl.x);
            ctrl.x(xi_idx) = ctrl.x(xi_idx) + (r_dev(1) - y_dev);
        end

        % ---- Solve QP
        [du0, info] = ctrl.qp.solve(ctrl.x, r_dev, ctrl.u_prev_dev, ctrl.umin_dev, ctrl.umax_dev, ctrl.du_max);

        u_next_dev = ctrl.u_prev_dev + du0;
        u_next_dev = max(ctrl.umin_dev, min(ctrl.umax_dev, u_next_dev));
        ctrl.u_prev_dev = u_next_dev;

        % ---- Time update model state (using augmented model)
        A = ctrl.qp.A; B = ctrl.qp.B;
        ctrl.x = A*ctrl.x + B*u_next_dev;

        % Convert to absolute
        u_next_abs = u_next_dev + ctrl.u_mean;
        u_next_abs = max(config.constraints.u_min, min(config.constraints.u_max, u_next_abs));
    end
end