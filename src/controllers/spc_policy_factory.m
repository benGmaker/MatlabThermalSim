function [ctrl_step, ctrl_init, meta] = spc_policy_factory(config)
%SPC_POLICY_FACTORY Build SPC (subspace-ID + MPC) controller as a step-wise policy.
%Fix: avoid calling mpcstate() inside an anonymous function (static workspace issue).

    if nargin < 1
        config = config_simulation();
    end

    %% Load data for ID
    load('results/multisine_response_data.mat', 'multisine_data');

    u_data = multisine_data.Q(:);
    y_data = multisine_data.T(:);
    dt = multisine_data.params.dt;

    % Center the data
    u_mean = mean(u_data);
    y_mean = mean(y_data);
    u_id = u_data - u_mean;
    y_id = y_data - y_mean;

    %% Identify state-space model
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

    %% Build MPC object
    P = config.SPC.P;
    M = config.SPC.M;

    u_min = config.constraints.u_min;
    u_max = config.constraints.u_max;
    du_max = config.constraints.du_max;

    mpcobj = mpc(sys_ss, dt);

    mpcobj.PredictionHorizon = P;
    mpcobj.ControlHorizon = M;

    mpcobj.Weights.OutputVariables = config.SPC.Q_weight;
    mpcobj.Weights.ManipulatedVariablesRate = config.SPC.R_weight;
    mpcobj.Weights.ManipulatedVariables = 0;

    % Constraints in deviation variables
    mpcobj.ManipulatedVariables.Min = u_min - u_mean;
    mpcobj.ManipulatedVariables.Max = u_max - u_mean;
    mpcobj.ManipulatedVariables.RateMin = -du_max;
    mpcobj.ManipulatedVariables.RateMax = du_max;

    mpcobj.OutputVariables.Min = -Inf;
    mpcobj.OutputVariables.Max = Inf;

    %% Meta for saving/inspection
    meta = struct();
    meta.dt = dt;
    meta.u_mean = u_mean;
    meta.y_mean = y_mean;
    meta.model = sys_ss;
    meta.nx = nx;

    %% Return init/step
    ctrl_init = @init_state;
    ctrl_step = @step;

    function ctrl = init_state()
        ctrl = struct();
        ctrl.mpcobj = mpcobj;

        % IMPORTANT: must execute in normal function workspace
        ctrl.xmpc = mpcstate(mpcobj);

        ctrl.u_mean = u_mean;
        ctrl.y_mean = y_mean;
        ctrl.dt = dt;
        ctrl.sys_ss = sys_ss;
        ctrl.nx = nx;
    end

    function [u_next_abs, ctrl] = step(k, y_k_abs, r_traj_abs, ctrl, config) %#ok<INUSD>
        % Deviation output and reference
        y_dev = y_k_abs - ctrl.y_mean;
        r_dev = r_traj_abs(1) - ctrl.y_mean;

        % Compute control move in deviation
        u_dev = mpcmove(ctrl.mpcobj, ctrl.xmpc, y_dev, r_dev);

        % Convert to absolute
        u_next_abs = u_dev + ctrl.u_mean;
    end
end