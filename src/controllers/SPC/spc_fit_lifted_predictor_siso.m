function spc = spc_fit_lifted_predictor_siso(u_id, y_id, M, nx, P)
%SPC_FIT_LIFTED_PREDICTOR_SISO Fit SPC lifted predictor directly from data.
%
% Past horizon:  M
% Future horizon: P (same as MPC horizon)
%
% Outputs:
%   Kx   (nx x 2M)
%   F    (P x nx)
%   Phi  (P x P)

    [Up, Yp, Uf, Yf] = spc_hankel_past_future_siso(u_id, y_id, M, P);
    Wp = [Up; Yp];  % (2M) x j

    % Oblique projection and SVD
    Oi = spc_oblique_projection(Yf, Uf, Wp);   % Yf (P x j), Uf (P x j)
    [Gamma_i_hat, Xp_hat, sv] = spc_estimate_gammax(Oi, nx);

    % State regression: Xp_hat ≈ Kx * Wp
    Kx = Xp_hat * pinv(Wp);                   % nx x (2M)

    % Predictor regression: Yf ≈ Fi*Xp_hat + Phii*Uf
    Phi_reg = [Xp_hat; Uf];                   % (nx + P) x j
    Mreg = Yf * pinv(Phi_reg);                % P x (nx + P)

    F   = Mreg(:, 1:nx);                      % P x nx
    Phi = Mreg(:, nx+1:end);                  % P x P

    spc = struct();
    spc.M = M;
    spc.nx = nx;
    spc.P = P;

    spc.Kx = Kx;
    spc.F = F;
    spc.Phi = Phi;

    spc.Gamma_i_hat = Gamma_i_hat;
    spc.sv = sv;
end