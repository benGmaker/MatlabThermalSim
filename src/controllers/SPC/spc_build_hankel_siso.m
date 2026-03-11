function H = spc_build_hankel_siso(z, L)
%SPC_BUILD_HANKEL_SISO Hankel matrix of depth L from a SISO signal.
% z: (T x 1), H: (L x (T-L+1))

    z = z(:).';
    T = numel(z);
    j = T - L + 1;
    if j <= 0
        error('Signal too short for Hankel depth L=%d (T=%d).', L, T);
    end

    H = zeros(L, j);
    for r = 1:L
        H(r,:) = z(r:r+j-1);
    end
end