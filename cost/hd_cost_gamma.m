function cost = hd_cost_gamma(satellite_coords, current_sat, start_sat, end_sat, gamma)
    % hd(N) = d(N, Ns) - d(N, Nd) - gamma * angle_d(N)
    Ns_N = satellite_coords(start_sat, :) - satellite_coords(current_sat, :); 
    Nd_N = satellite_coords(end_sat, :) - satellite_coords(current_sat, :);
    d_n_ns = norm(Ns_N);
    d_n_nd = norm(Nd_N);
    angle_d = acos(dot(Nd_N, Ns_N) / sqrt(dot(Ns_N, Ns_N) * dot(Nd_N, Nd_N)));

    cost = d_n_ns - d_n_nd - gamma * angle_d;
end