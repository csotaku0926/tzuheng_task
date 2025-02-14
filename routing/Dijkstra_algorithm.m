function [path, total_distance, elapsed_time, n_visited, iter_count, update_cost_count] = Dijkstra_algorithm(satellite_coords, start_satellite, end_satellite, total_satellites, ...
                                                                            earth_radius, atm_high, orbit_altitudes, n_sat_list, L_batt)
    tic; % 開始計時
    unvisited = 1:total_satellites;
    visited = transpose(L_batt);
    n_visited = 0;
    iter_count = 0;
    update_cost_count = 0;

    distance = inf(1, total_satellites);
    distance(start_satellite) = 0;

    previous = zeros(1, total_satellites);

    while ~isempty(unvisited)
        [~, idx] = min(distance(unvisited));
        current_satellite = unvisited(idx);

        iter_count = iter_count + 1;

        if current_satellite == end_satellite
            path = reconstruct_path_to_end(previous, current_satellite);
            total_distance = distance(current_satellite);
            elapsed_time = toc; % 結束計時
            return;
        end

        unvisited(idx) = [];
        visited = [visited, current_satellite];
        n_visited = n_visited + 1;

        neighbors = find_neighbors_Dmax(satellite_coords, current_satellite, earth_radius, atm_high, orbit_altitudes, n_sat_list);
        for i = 1:length(neighbors)
            neighbor = neighbors(i);
            if ismember(neighbor, visited)
                continue;
            end

            n_visited = n_visited + 1;

            alt = distance(current_satellite) + distance_between(satellite_coords, current_satellite, neighbor);
            if alt < distance(neighbor)
                distance(neighbor) = alt;
                previous(neighbor) = current_satellite;
                update_cost_count = update_cost_count + 1;
            end
        end
    end

    % 若未找到路徑
    path = [];
    total_distance = 0;
    elapsed_time = toc;
end
