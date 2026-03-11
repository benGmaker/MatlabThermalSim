function [ctrl_step, ctrl_init, meta] = spc_policy_factory(config)
%SPC_POLICY_FACTORY Subspace ID (N4SID) + nominal MPC policy (matches report MPC formulation).
%
% Note: This is the "parametric SPC route" (identify A,B,C,D via subspace method, then do MPC).

    if nargin < 1
        config = config_simulation();
    end

    ds = load_predictive_data(config);
    
    u_data = ds.u_data;
    y_data = ds.T;
    dt = ds.dt;

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
    P  = config.predictive.P;
    Qw = config.predictive.Q_weight;
    Rw = config.predictive.R_weight;

    umin_dev = config.constraints.u_min - u_mean;
    umax_dev = config.constraints.u_max - u_mean;

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
    meta.model = sys_ss;
    meta.nx = nx;
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
        %#ok<INUSD> k y_k_abs
        r_dev = r_traj_abs(:) - ctrl.y_mean;

        [u0_dev, info] = ctrl.qp.solve(ctrl.x, r_dev, ctrl.umin_dev, ctrl.umax_dev, ctrl.ymin_dev, ctrl.ymax_dev); %#ok<NASGU>

        u_next_dev = u0_dev;
        u_next_dev = max(ctrl.umin_dev, min(ctrl.umax_dev, u_next_dev));

        A = ctrl.qp.A; B = ctrl.qp.B;
        ctrl.x = A*ctrl.x + B*u_next_dev;

        u_next_abs = u_next_dev + ctrl.u_mean;
        u_next_abs = max(config.constraints.u_min, min(config.constraints.u_max, u_next_abs));
    end
end