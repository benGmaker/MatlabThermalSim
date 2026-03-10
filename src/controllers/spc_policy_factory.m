function [ctrl_step, ctrl_init, meta] = spc_policy_factory(config)
%SPC_POLICY_FACTORY Subspace ID + quadprog-based MPC policy.

    if nargin < 1
        config = config_simulation();
    end

     
    load(config.predictive.data_source, 'step_data'); % hard coded needs long term solution

    u_data = step_data.Q(:);
    y_data = step_data.T(:);
    dt = step_data.params.dt;

    u_mean = mean(u_data);
    y_mean = mean(y_data);
    u_id = u_data - u_mean;
    y_id = y_data - y_mean;

    z = iddata(y_id, u_id, dt);
    nx = config.SPC.ident.nx;

    switch lower(config.SPC.ident.method)
        case 'n4sid'
            opt = n4sidOptions('Focus', config.SPC.ident.focus);
            sys_id = n4sid(z, nx, opt);
        otherwise
            error('Unsupported config.SPC.ident.method: %s (use ''n4sid'')', config.SPC.ident.method);
    end

    if ~isdt(sys_id)
        sys_id = c2d(sys_id, dt, 'zoh');
    end
    sys_ss = ss(sys_id);

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
    meta.model = sys_ss;
    meta.nx = nx;
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

        ctrl.x = zeros(qp.nx, 1);

        % IMPORTANT: match run_closed_loop_generic initial u(1)=0 (absolute)
        u0_abs = 0;
        ctrl.u_prev_dev = u0_abs - u_mean;

        ctrl.umin_dev = umin_dev;
        ctrl.umax_dev = umax_dev;
        ctrl.du_max = du_max;
    end

    function [u_next_abs, ctrl] = step(k, y_k_abs, r_traj_abs, ctrl, config) 
        y_dev = y_k_abs - ctrl.y_mean;
        r_dev = r_traj_abs(:) - ctrl.y_mean;

        if ctrl.qp.enable_integrator
            xi_idx = numel(ctrl.x);
            ctrl.x(xi_idx) = ctrl.x(xi_idx) + (r_dev(1) - y_dev);
        end

        [du0, info] = ctrl.qp.solve(ctrl.x, r_dev, ctrl.u_prev_dev, ctrl.umin_dev, ctrl.umax_dev, ctrl.du_max);

        u_next_dev = ctrl.u_prev_dev + du0;
        u_next_dev = max(ctrl.umin_dev, min(ctrl.umax_dev, u_next_dev));
        ctrl.u_prev_dev = u_next_dev;

        A = ctrl.qp.A; B = ctrl.qp.B;
        ctrl.x = A*ctrl.x + B*u_next_dev;

        u_next_abs = u_next_dev + ctrl.u_mean;
        u_next_abs = max(config.constraints.u_min, min(config.constraints.u_max, u_next_abs));
    end
end