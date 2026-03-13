function [ctrl_step, ctrl_init, meta] = spc_policy_factory(config)
%SPC_POLICY_FACTORY SPC (data-driven lifted predictor) + nominal MPC QP.
%
% This implementation follows the report SPC narrative:
%   - Build Hankel matrices Up,Yp,Uf,Yf
%   - Oblique projection O_i = Yf /_{Uf} Wp
%   - SVD -> latent state sequence Xp
%   - Fit lifted predictor Y = F*x + Phi*U directly from data
%   - Solve the same condensed MPC QP, but with (F,Phi) from data (no A,B,C,D)

    % loading and centering data
    ds = load_predictive_data(config);
    u_data = ds.u_data;
    y_data = ds.T;
    dt = ds.dt;
    u_mean = mean(u_data);
    y_mean = mean(y_data);
    u_id = u_data - u_mean;
    y_id = y_data - y_mean;

    % Shared config
    P  = config.predictive.P;
    Qw = config.predictive.Q_weight;
    Rw = config.predictive.R_weight;

    umin_dev = config.constraints.u_min - u_mean;
    umax_dev = config.constraints.u_max - u_mean;

    % Position constraints (optional) 
    if isfield(config.constraints,'y_min') && isfield(config.constraints,'y_max')
        ymin_dev = config.constraints.y_min - y_mean;
        ymax_dev = config.constraints.y_max - y_mean;
        enable_y_constraints = true;
    else
        ymin_dev = -Inf;
        ymax_dev =  Inf;
        enable_y_constraints = false;
    end

    % --- SPC identification hyperparameters ---
    nx = config.SPC.ident.nx;
    if isfield(config.SPC, 'i')
        i = config.SPC.i;
    else
        i = max(2*nx, P); % safe-ish default; must satisfy i >= P
    end

    % Fit SPC lifted predictor directly from data
    spc = spc_fit_lifted_predictor_siso(u_id, y_id, i, nx, P);

    % Build quadratic program
    qp = qp_spc_lifted_siso(spc.F, spc.Phi, P, Qw, Rw, config);

    meta = struct();
    meta.dt = dt;
    meta.u_mean = u_mean;
    meta.y_mean = y_mean;
    meta.nx = nx;
    meta.i = i;
    meta.P = P;
    meta.Qw = Qw; meta.Rw = Rw;
    meta.enable_y_constraints = enable_y_constraints;

    % Keep for debugging/analysis
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
        % history buffers for past window (length i)
        ctrl.i = i;
        ctrl.u_hist = zeros(i,1);
        ctrl.y_hist = zeros(i,1);

        % latent state
        ctrl.x = zeros(nx,1);
    end

    function [u_next_abs, ctrl] = step(~, y_k_abs, r_traj_abs, ctrl, config)
        % deviation signals
        y_dev = y_k_abs - ctrl.y_mean;
        r_dev = r_traj_abs(:) - ctrl.y_mean;

        % update history (shift up, append current)
        ctrl.u_hist = [ctrl.u_hist(2:end); ctrl.u_prev_dev];
        ctrl.y_hist = [ctrl.y_hist(2:end); y_dev];

        % estimate latent state from past window: x = Kx * [u_past; y_past]
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
