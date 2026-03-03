function [predictor_step, predictor_init, meta] = mpc_offline_predictor_factory(config)
%MPC_OFFLINE_PREDICTOR_FACTORY
% Offline 1-step predictor using identified_models.sys_ss (same model used by MPC pipeline).

    if nargin < 1, config = config_simulation(); end %#ok<INUSD>

    data = load('results/identified_models.mat', 'identified_models');
    identified_models = data.identified_models;

    if ~isfield(identified_models, 'sys_ss') || isempty(identified_models.sys_ss)
        error('identified_models.sys_ss is missing/empty.');
    end

    sys = identified_models.sys_ss;
    dt = identified_models.dt;
    u_mean = identified_models.u_mean;
    y_mean = identified_models.y_mean;

    if ~isdt(sys)
        sys = c2d(sys, dt, 'zoh');
    end
    [A,B,C,D] = ssdata(sys);

    L_gain = 0.5;

    predictor_init = @init;
    predictor_step = @step;

    meta = struct();
    meta.dt = dt;
    meta.sys_ss = sys;
    meta.u_mean = u_mean;
    meta.y_mean = y_mean;
    meta.L_gain = L_gain;

    function pred = init()
        pred = struct();
        pred.A=A; pred.B=B; pred.C=C; pred.D=D;
        pred.x = zeros(size(A,1),1);
        pred.u_mean = u_mean;
        pred.y_mean = y_mean;
        pred.L_gain = L_gain;
    end

    function [y_hat_next_abs, pred] = step(k, y_k_abs, u_k_abs, pred, config) %#ok<INUSD>
        y_k = y_k_abs - pred.y_mean;
        u_k = u_k_abs - pred.u_mean;

        y_hat_k = pred.C*pred.x + pred.D*u_k;
        pred.x = pred.x + pred.L_gain*(pred.C')*(y_k - y_hat_k);

        x_next = pred.A*pred.x + pred.B*u_k;
        y_hat_next = pred.C*x_next + pred.D*u_k;

        pred.x = x_next;
        y_hat_next_abs = y_hat_next + pred.y_mean;
    end
end