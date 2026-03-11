function [Gamma_i_hat, Xp_hat, sv] = spc_estimate_gammax(Oi, nx)
% Returns Gamma_i_hat (i*p x nx) and Xp_hat (nx x j)

    [U,S,V] = svd(Oi, 'econ');
    sv = diag(S);

    U1 = U(:,1:nx);
    S1 = S(1:nx,1:nx);
    V1 = V(:,1:nx);

    Gamma_i_hat = U1 * sqrtm(S1);
    Xp_hat = sqrtm(S1) * V1';
end