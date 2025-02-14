function neighbors = find_neighbors_Dmax(satellite_coords, current_satellite, earth_radius, atm_high, orbit_altitudes, n_sat_list)
    neighbors = [];
    h2 = get_sat_h_by_idx(current_satellite, orbit_altitudes, n_sat_list);
    
    for N = 1:size(satellite_coords, 1)
        if N == current_satellite
            continue;
        end

        % formula 12
        h1 = get_sat_h_by_idx(N, orbit_altitudes, n_sat_list);
        Dmax = sqrt((earth_radius + h1) ^ 2 - (earth_radius + atm_high) ^ 2) + ...
                sqrt((earth_radius + h2) ^ 2 - (earth_radius + atm_high) ^ 2);

        % pseudo code line 9 and 22
        if distance_between(satellite_coords, current_satellite, N) <= Dmax
            neighbors = [neighbors; N];
        end
    end
end


function height = get_sat_h_by_idx(sat_idx, orbit_altitudes, n_sat_list)
    alt_idx = 0;
    while sat_idx > 0 && alt_idx + 1 <= length(n_sat_list)
        sat_idx = sat_idx - n_sat_list(alt_idx + 1);
        alt_idx = alt_idx + 1;
    end

    % illegal sat_idx
    if alt_idx > length(n_sat_list)
        fprintf('[get_sat_h_by_idx] warning: illegal sat_idx %d\n', sat_idx);
        height = 540;
        return;
    end 

    height = orbit_altitudes(alt_idx);
end

