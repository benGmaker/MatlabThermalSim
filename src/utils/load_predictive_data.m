function ds = load_predictive_data(config)
%LOAD_PREDICTIVE_DATA_SWITCH Switch/case loader based on config.dataset_choice.
%
% dataset_choice options:
%   'step'       -> loads variable 'step_data'
%   'multisine'  -> loads variable 'multisine_data'
%   'impulse'    -> loads variable 'impulse_data'
%
% Assumes identical internal structure for all datasets:
%   data.Q, data.T, data.params.dt
%
% Required config fields:
%   config.predictive.data_source
%   config.dataset_choice


    switch lower(string(config.dataset_choice))
        case "step"
            src = 'results/data/step_response_data.mat';
            var = 'step_data';
        case "multisine"
            src = 'results/data/multisine_response_data.mat';
            var = 'multisine_data';
        case "multisine2"
            src = 'results/data/multisine_response_data_2.mat';
            var = 'multisine2_data';            
        case "impulse"
            src = 'results/data/impulse_response_data.mat';
            var = 'impulse_data';
        case "doublet"
            src = 'results/data/doublet_response_data.mat';
            var = 'doublet_data';
        otherwise
            error('load_predictive_data_switch:UnsupportedChoice', ...
                'Unsupported config.dataset_choice: %s', string(config.dataset_choice));
    end

    S = load(src, var);
    V = S.(var);
    
    ds = struct();
    ds.t = V.t(:);
    ds.T = V.T(:);
    ds.T_clean = V.T_clean(:);
    ds.u_data = V.Q(:);
    ds.Q = V.Q(:);
    ds.params = V.params;
    ds.dt     = V.params.dt;
end