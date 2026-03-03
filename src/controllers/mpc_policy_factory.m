function [ctrl_step, ctrl_init, meta] = mpc_policy_factory(config)
%MPC_POLICY_FACTORY Build MPC controller as a step-wise policy.

    if nargin < 1
        config = config_simulation();
    end

    data = load('results/identified_models.mat', 'identified_models');
    identified_models = data.identified_models;

    sys = identified_models.sys_tf;
    dt = identified_models.dt;
    u_mean = identified_models.u_mean;
    y_mean = identified_models.y_mean;

    % Convert to discrete-time if needed
    if ~isdt(sys)
        sys_d = c2d(sys, dt, 'zoh');
    else
        sys_d = sys;
    end

    % Build MPC object
    mpcobj = mpc(sys_d, dt);
    mpcobj.PredictionHorizon = config.MPC.P;
    mpcobj.ControlHorizon = config.MPC.M;

    mpcobj.Weights.OutputVariables = config.MPC.Q_weight;
    mpcobj.Weights.ManipulatedVariablesRate = config.MPC.R_weight;
    mpcobj.Weights.ManipulatedVariables = 0;

    mpcobj.ManipulatedVariables.Min = config.constraints.u_min - u_mean;
    mpcobj.ManipulatedVariables.Max = config.constraints.u_max - u_mean;
    mpcobj.ManipulatedVariables.RateMin = -config.constraints.du_max;
    mpcobj.ManipulatedVariables.RateMax = config.constraints.du_max;

    mpcobj.OutputVariables.Min = -Inf;
    mpcobj.OutputVariables.Max = Inf;

    % ---- meta (optional)
    meta = struct();
    meta.dt = dt;
    meta.u_mean = u_mean;
    meta.y_mean = y_mean;
    meta.sys_d = sys_d;

    % ---- IMPORTANT: do NOT call mpcstate inside an anonymous function
    ctrl_init = @init_state;
    ctrl_step = @step;

    function ctrl = init_state()
        ctrl = struct();
        ctrl.mpcobj = mpcobj;

        % This must run in a normal function workspace (not anonymous)
        ctrl.xmpc = mpcstate(mpcobj);

        ctrl.u_mean = u_mean;
        ctrl.y_mean = y_mean;
        ctrl.dt = dt;
    end

    function [u_next_abs, ctrl] = step(k, y_k_abs, r_traj_abs, ctrl, config) %#ok<INUSD>
        y_dev = y_k_abs - ctrl.y_mean;
        r_dev = r_traj_abs(1) - ctrl.y_mean;

        u_dev = mpcmove(ctrl.mpcobj, ctrl.xmpc, y_dev, r_dev);
        u_next_abs = u_dev + ctrl.u_mean;
    end
end