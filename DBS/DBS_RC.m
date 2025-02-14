function [rc_path, w, elapsed_time, n_visited, iter_count, update_cost_count] = DBS_RC(L_batt, P, satellite_coords, gamma, ...
    earth_radius, atm_high, orbit_altitudes, n_sat_list)
    tic;
    n_visited = 0;
    w = 0;
    i = 1;
    rc_path = [];

    iter_count = 0;
    update_cost_count = 0;

    while i <= length(P)
        p_i = P(i);

        % safe
        if ~ismember(p_i, L_batt)
            i = i + 1;
            rc_path = [rc_path; p_i];
            continue; 
        end

        % encounter a node pi in L_batt
        prev_idx = i - 1;
        if prev_idx < 1
            % re-select preceding satellites (in index) that not in L_batt
            p_prev = P(1) - 1;
            while ismember(p_prev, L_batt)
                p_prev = p_prev - 1;
            end
        else
            p_prev = P(prev_idx);
        end


        next_idx = i + 1;
        while next_idx <= length(P)
            p_next = P(next_idx);
            if ~ismember(p_next, L_batt)
                break;
            end
            next_idx = next_idx + 1;
        end

        if next_idx > length(P)
            p_next = P(length(P)) + 1;
            while ismember(p_next, L_batt)
                p_next = p_next + 1;
            end
        end

        % reconstruct path using p_prev as start and p_next as destination
        [new_path, ~, ~, fs_visited, fs_iter_count, fs_update_count] = DBS_FS(satellite_coords, p_prev, p_next, L_batt, gamma, ...
        earth_radius, atm_high, orbit_altitudes, n_sat_list);
        % start sat is already in rc_path 
        if prev_idx >= 1
            new_path(1) = [];
        end
        % insert new path segment
        rc_path = [rc_path; new_path];

        n_visited = n_visited + fs_visited;
        iter_count = iter_count + fs_iter_count;
        update_cost_count = update_cost_count + fs_update_count;
        i = next_idx + 1;
    end

    % calc path distance
    % disp(rc_path);
    p_ = rc_path(1);
    for i = 1:length(rc_path)
        w = w + distance_between(satellite_coords, rc_path(i), p_);
        p_ = rc_path(i);
    end

    elapsed_time = toc;
end
