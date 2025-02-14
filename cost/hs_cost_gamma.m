function cost = hs_cost_gamma(satellite_coords, current_sat, start_sat, end_sat, gamma)
    % hs(N) = d(N, Nd) - d(N, Ns) - gamma * angle_s(N)
    Ns_N = satellite_coords(start_sat, :) - satellite_coords(current_sat, :); 
    Nd_N = satellite_coords(end_sat, :) - satellite_coords(current_sat, :);
    d_n_ns = norm(Ns_N);
    d_n_nd = norm(Nd_N);
    angle_n = acos(dot(Ns_N, Nd_N) / sqrt(dot(Ns_N, Ns_N) * dot(Nd_N, Nd_N)));

    cost = d_n_nd - d_n_ns - gamma * angle_n;
end