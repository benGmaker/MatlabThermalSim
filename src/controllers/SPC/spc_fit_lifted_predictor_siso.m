function spc = spc_fit_lifted_predictor_siso(u_id, y_id, i, nx, P)
%SPC_FIT_LIFTED_PREDICTOR_SISO Fit SPC lifted predictor directly from data.
%
% Outputs struct spc with fields:
%   i, nx, P
%   Kx   (nx x 2i)   state map from [u_past; y_past]
%   F    (P x nx)    maps xk -> y_{k+1:k+P}
%   Phi  (P x P)     maps U_{k:k+P-1} -> y_{k+1:k+P}

    if P > i
        error('SPC: require P <= i for this implementation (got P=%d, i=%d).', P, i);
    end

    [Up, Yp, Uf, Yf] = spc_hankel_past_future_siso(u_id, y_id, i);
    Wp = [Up; Yp];

    % Oblique projection and SVD (Gamma_i_hat, Xp_hat)
    Oi = spc_oblique_projection(Yf, Uf, Wp);
    [Gamma_i_hat, Xp_hat, sv] = spc_estimate_gammax(Oi, nx); 

    % --- State regression: Xp_hat ≈ Kx * Wp
    Kx = Xp_hat * pinv(Wp);           % nx x (2i)

    % --- Predictor regression: Yf ≈ Fi*Xp_hat + Phii*Uf
    % Solve Yf = [Fi Phii] * [Xp_hat; Uf]
    Phi_reg = [Xp_hat; Uf];           % (nx + i) x j
    M = Yf * pinv(Phi_reg);           % i x (nx + i)

    Fi   = M(:, 1:nx);                % i x nx
    Phii = M(:, nx+1:end);            % i x i

    % Extract horizon P
    F = Fi(1:P, :);                   % P x nx
    Phi = Phii(1:P, 1:P);             % P x P

    spc = struct();
    spc.i = i;
    spc.nx = nx;
    spc.P = P;
    spc.Kx = Kx;
    spc.F = F;
    spc.Phi = Phi;
    spc.Gamma_i_hat = Gamma_i_hat; % keep if you want for debugging
    spc.sv = sv;
end