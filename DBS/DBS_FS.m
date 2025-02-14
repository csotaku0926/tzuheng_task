function [path, w, elapsed_time, n_visited, iter_count, update_cost_count] = DBS_FS(satellite_coords, start_sat, end_sat, L_batt, gamma, ...
    earth_radius, atm_high, orbit_altitudes, n_sat_list)
    % start countdown
    tic;
    % fs(N) = gs(N) + hs(N) 
    % -> gs = min(gs(N), gs(Ncs) + d(Ncs, N))
    % -> hs: remaining estimated cost from N to Ns
    n_sat = sum(n_sat_list);
    n_inct = n_sat + 1;
    g_forward_cost = ones(n_sat + 1, 1) * Inf;
    f_forward_cost = ones(n_sat, 1) * Inf; % init as infinte
    g_backward_cost = ones(n_sat + 1, 1) * Inf;
    f_backward_cost = ones(n_sat, 1) * Inf; % init as infinte
    parents = zeros(n_sat, 1); % record each satellite's parent
    back_parents = zeros(n_sat, 1); % as above but in reverse direction

    % init cost of start sat and end sat as 0
    f_forward_cost(start_sat) = 0;
    f_backward_cost(end_sat) = 0;
    % also g cost set to 0 (search start from them)
    g_forward_cost(start_sat) = 0;
    g_backward_cost(end_sat) = 0;


    open_forward_list = 1:n_sat; % set of all satellites
    open_backward_list = 1:n_sat;
    % L_sat is for DBS_RC 
    closed_forward_list = L_batt;
    closed_backward_list = L_batt;

    n_visited = 0;
    visited_hash = zeros(n_sat, 1);
    n_tol = 0; %5 * length(n_sat_list);

    iter_count = 0;
    update_cost_count = 0;

    while ~isempty(open_forward_list) || ~isempty(open_backward_list)

        %% forward direction
        % get N s.t. f_s^o(N) is the minimum
        [~, min_idx] = min(f_forward_cost(open_forward_list));
        current_N = open_forward_list(min_idx);

        % exclude min_N from open list
        open_forward_list(min_idx) = [];
        closed_forward_list = [closed_forward_list; current_N];
        n_visited = n_visited + 1;
        visited_hash(current_N) = 1;

        iter_count = iter_count + 1;

        % traverse all connectable neighbors 
        neighbors = find_neighbors_Dmax(satellite_coords, current_N, earth_radius, atm_high, orbit_altitudes, n_sat_list);

        for i = 1:length(neighbors)
            neighbor = neighbors(i);
            % check if `neighbor` already in closed list, prevent re-visit
            if ismember(neighbor, closed_forward_list)
                continue;
            end

            % init, current_N is the starting node
            tentative_g_cost = g_forward_cost(current_N) + distance_between(satellite_coords, current_N, neighbor);
            if tentative_g_cost >= g_forward_cost(neighbor)
                continue;
            end

            n_visited = n_visited + 1;
            visited_hash(neighbor) = 1;

            % record current parent of N
            parents(neighbor) = current_N;
            % update g_s(N) and other costs
            g_forward_cost(neighbor) = tentative_g_cost;
            f_forward_cost(neighbor) = g_forward_cost(neighbor) + hs_cost_gamma(satellite_coords, neighbor, start_sat, end_sat, gamma);
            
            update_cost_count = update_cost_count + 1;
            
            % intersect with backward direction
            if g_backward_cost(neighbor) < inf
                % update shortest path
                g_inct = g_forward_cost(n_inct) + g_backward_cost(n_inct);
                g_new = g_forward_cost(neighbor) + g_backward_cost(neighbor);

                if g_new < g_inct
                    n_inct = neighbor;
                    break;
                end
            end
        end

        %% backward direction
        [~, min_idx] = min(f_backward_cost(open_backward_list));
        current_back_N = open_backward_list(min_idx);

        % exclude min_N from open list
        open_backward_list(min_idx) = [];
        closed_backward_list = [closed_backward_list; current_back_N];
        n_visited = n_visited + 1;
        visited_hash(current_back_N) = 1;

        iter_count = iter_count + 1;

        % traverse all connectable neighbors 
        neighbors = find_neighbors_Dmax(satellite_coords, current_back_N, ...
        earth_radius, atm_high, orbit_altitudes, n_sat_list);

        for i = 1:length(neighbors)
            neighbor = neighbors(i);
            % check if `neighbor` already in closed list, prevent re-visit
            if ismember(neighbor, closed_backward_list)
                continue;
            end

            % init, current_back_N is the starting node
            tentative_g_cost = g_backward_cost(current_back_N) + distance_between(satellite_coords, current_back_N, neighbor);
            if tentative_g_cost >= g_backward_cost(neighbor)
                continue;
            end

            % record current parent of N
            back_parents(neighbor) = current_back_N;
            % update g_d(N) and other costs
            g_backward_cost(neighbor) = tentative_g_cost;
            f_backward_cost(neighbor) = g_backward_cost(neighbor) + hd_cost_gamma(satellite_coords, neighbor, start_sat, end_sat, gamma);
            
            update_cost_count = update_cost_count + 1;

            % intersect with forward direction
            if g_forward_cost(neighbor) < inf
                % update shortest path
                g_new = g_forward_cost(neighbor) + g_backward_cost(neighbor);
                g_inct = g_forward_cost(n_inct) + g_backward_cost(n_inct);

                if g_new < g_inct
                    n_inct = neighbor;
                    break;
                end
            end
        end

        %% end condition
        % check if found intersecting satellite
        if n_inct < n_sat + 1
            if n_tol > 0
                n_tol = n_tol - 1;
                continue;
            end

            sat = n_inct;
            while sat ~= end_sat
                parents(back_parents(sat)) = sat;
                sat = back_parents(sat);
            end

            w = g_forward_cost(n_inct) + g_backward_cost(n_inct);
            path = reconstruct_path_to_end(parents, end_sat);
            n_visited = sum(visited_hash);
            elapsed_time = toc; % 結束計時
            return;
        end

    end
end