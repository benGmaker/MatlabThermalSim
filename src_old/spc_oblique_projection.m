function Oi = spc_oblique_projection(Yf, Uf, Wp)
%SPC_OBLIQUE_PROJECTION Compute O_i = Yf /_{Uf} Wp
% Practical implementation:
%   1) remove row-space(Uf) from both Yf and Wp,
%   2) project onto row-space(Wp_bar).

    j = size(Uf,2);
    Ij = eye(j);

    % Orthonormal basis for row-space(Uf)
    [Q,~] = qr(Uf', 0);     % Uf' is j x (i), Q is j x r
    Puf = Q*Q';             % j x j projector

    Nuf = Ij - Puf;         % nullspace projector (orthogonal complement)

    Yf_bar = Yf * Nuf;
    Wp_bar = Wp * Nuf;

    % Project Yf_bar onto row-space(Wp_bar)
    Oi = Yf_bar * pinv(Wp_bar) * Wp_bar; % pinv might be numerically unstable
end