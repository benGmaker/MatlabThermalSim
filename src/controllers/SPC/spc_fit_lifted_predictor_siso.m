function spc = spc_fit_lifted_predictor_siso(u_id, y_id, M, P)
%SPC_FIT_LIFTED_PREDICTOR_SISO Fit DeePC-equivalent SPC multi-step predictor (SISO).
%
% Builds the SPC predictor:
%   Yf = P1*u_ini + P2*y_ini + Gamma*Uf
%
% and maps it to internal lifted form used by qp_spc_lifted_siso:
%   Y  = F*x + Phi*U
% where:
%   x = [u_ini; y_ini]
%   F = [P1 P2]
%   Phi = Gamma
    u_id = u_id(:);
    y_id = y_id(:);

    % Hankel blocks (SISO):
    % Up,Yp: (M x j)
    % Uf,Yf: (P x j)
    [Up, Yp, Uf, Yf] = spc_hankel_past_future_siso(u_id, y_id, M, P);

    % Regression: Theta = Yf * pinv([Up;Yp;Uf]) = [P1 P2 Gamma]
    Z = [Up; Yp; Uf];           % (2M + P) x j
    Theta = Yf * pinv(Z);       % (P x (2M + P))

    P1 = Theta(:, 1:M);                 % P x M
    P2 = Theta(:, M+1:2*M);             % P x M
    Gamma = Theta(:, 2*M+1:end);        % P x P

    spc = struct();
    spc.M  = M;  
    spc.P  = P;

    % Map to your QP form: Y = F*x + Phi*U, x=[u_ini;y_ini]
    spc.F = [P1, P2];     % P x (2M)
    spc.Phi = Gamma;      % P x P

    % "State"/initialization map: x = Kx*[u_ini;y_ini]
    spc.Kx = eye(2*M);    % (2M x 2M)

    % Diagnostics
    spc.Theta = Theta;
end