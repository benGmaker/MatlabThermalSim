function [T, t, Q] = thermal_model(Q_input, params)
% THERMAL_MODEL Simulates the thermal dynamics based on APMonitor models
%
% Inputs:
%   Q_input: function handle for heater input Q(t) [W or %]
%   params: structure with fields:
%       - T0: initial temperature [°C] (default: 23)
%       - Ta: ambient temperature [°C] (default: 23)
%       - U: heat transfer coefficient [W/m^2-K] (default: 10)
%       - A: surface area [m^2] (default: 1e-3)
%       - m: mass [kg] (default: 4e-3)
%       - Cp: heat capacity [J/kg-K] (default: 500)
%       - alpha: heater efficiency (default: 0.01)
%       - eps: radiation coefficient (default: 0.9)
%       - sigma: Stefan-Boltzmann constant [W/m^2-K^4]
%       - t_final: simulation time [s] (default: 600)
%       - dt: sampling time [s] (default: 1)
%       - model_type: 'linear' or 'nonlinear' (default: 'nonlinear')
%
% Outputs:
%   T: temperature array [°C]
%   t: time array [s]
%   Q: heater input array [W or %]

    % Default parameters
    if nargin < 2
        params = struct();
    end
    
    % Set defaults
    params = set_defaults(params);
    
    % Time vector
    t = 0:params.dt:params.t_final;
    n = length(t);
    
    % Initialize arrays
    T = zeros(n, 1);
    Q = zeros(n, 1);
    T(1) = params.T0;
    
    % Simulate
    for i = 1:n-1
        Q(i) = Q_input(t(i));
        
        if strcmp(params.model_type, 'nonlinear')
            % Nonlinear model with convection and radiation
            % dT/dt = (Q*alpha - U*A*(T-Ta) - eps*sigma*A*(T^4-Ta^4)) / (m*Cp)
            T_K = T(i) + 273.15;  % Convert to Kelvin for radiation
            Ta_K = params.Ta + 273.15;
            
            Q_heater = params.alpha * Q(i);
            Q_conv = params.U * params.A * (T(i) - params.Ta);
            Q_rad = params.eps * params.sigma * params.A * (T_K^4 - Ta_K^4);
            
            dTdt = (Q_heater - Q_conv - Q_rad) / (params.m * params.Cp);
        else
            % Linear model (first-order)
            % dT/dt = (Q*alpha - (T-Ta)/tau) / tau
            tau = (params.m * params.Cp) / (params.U * params.A);
            K = params.alpha / (params.U * params.A);
            
            dTdt = K * Q(i) / tau - (T(i) - params.Ta) / tau;
        end
        
        % Euler integration
        T(i+1) = T(i) + dTdt * params.dt;
    end
    Q(end) = Q_input(t(end));
end

function params = set_defaults(params)
    % Set default parameter values based on APMonitor TCLab
    if ~isfield(params, 'T0'), params.T0 = 23; end
    if ~isfield(params, 'Ta'), params.Ta = 23; end
    if ~isfield(params, 'U'), params.U = 10; end
    if ~isfield(params, 'A'), params.A = 1e-3; end
    if ~isfield(params, 'm'), params.m = 4e-3; end
    if ~isfield(params, 'Cp'), params.Cp = 500; end
    if ~isfield(params, 'alpha'), params.alpha = 0.01; end
    if ~isfield(params, 'eps'), params.eps = 0.9; end
    if ~isfield(params, 'sigma'), params.sigma = 5.67e-8; end
    if ~isfield(params, 't_final'), params.t_final = 600; end
    if ~isfield(params, 'dt'), params.dt = 1; end
    if ~isfield(params, 'model_type'), params.model_type = 'nonlinear'; end
end