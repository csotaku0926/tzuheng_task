function [path, w, elapsed_time, n_visited, iter_count, update_cost_count] = DBS_OP(satellite_coords, start_sat, end_sat, gamma, ...
    earth_radius, atm_high, orbit_altitudes, n_sat_list)
    % start countdown
    tic;
    % fs(N) = gs(N) + hs(N) 
    % -> gs = min(gs(N), gs(Ncs) + d(Ncs, N))
    % -> hs: remaining estimated cost from N to Ns
    n_sat = sum(n_sat_list);
    g_forward_cost = ones(n_sat, 1) * Inf;
    f_forward_cost = ones(n_sat, 1) * Inf; % init as infinte
    g_backward_cost = ones(n_sat, 1) * Inf;
    f_backward_cost = ones(n_sat, 1) * Inf; % init as infinte
    parents = zeros(n_sat, 1); % record each satellite's parent
    back_parents = zeros(n_sat, 1); % as above but in reverse direction

    % init cost of start sat and end sat as 0
    f_forward_cost(start_sat) = 0;
    f_backward_cost(end_sat) = 0;
    % also g cost set to 0 (search start from them)
    g_forward_cost(start_sat) = 0;
    g_backward_cost(end_sat) = 0;

    w = inf;

    open_forward_list = [start_sat]; % set of all satellites
    open_backward_list = [end_sat];
    closed_forward_list = [];
    closed_backward_list = [];
    intersect_list = []; % L_isct
    intersect_sat = 0;

    n_visited = 0;

    visited_hash = zeros(n_sat, 1);

    iter_count = 0;
    update_cost_count = 0;

    while ~isempty(open_forward_list) || ~isempty(open_backward_list)
        % check if end coniditon reach
        w_old = w;

        %% forward direction
        % get N s.t. f_s^o(N) is the minimum
        [~, min_idx] = min(f_forward_cost(open_forward_list));
        current_N = open_forward_list(min_idx);

        iter_count = iter_count + 1;

        % exclude min_N from open list
        open_forward_list(min_idx) = [];
        closed_forward_list = [closed_forward_list; current_N];
        n_visited = n_visited + 1;
        visited_hash(current_N) = 1;

        % traverse all connectable neighbors 
        neighbors = find_neighbors_Dmax(satellite_coords, current_N, ...
        earth_radius, atm_high, orbit_altitudes, n_sat_list);

        for i = 1:length(neighbors)
            neighbor = neighbors(i);
            % check if `neighbor` already in closed list, prevent re-visit
            if ismember(neighbor, closed_forward_list)
                continue;
            end

            n_visited = n_visited + 1;
            visited_hash(neighbor) = 1;

            % init, current_N is the starting node
            tentative_g_cost = g_forward_cost(current_N) + distance_between(satellite_coords, current_N, neighbor);

            if ~ismember(neighbor, open_forward_list)
                open_forward_list = [open_forward_list; neighbor];
            elseif tentative_g_cost >= g_forward_cost(neighbor)
                continue;
            end

            % record current parent of N
            parents(neighbor) = current_N;
            % update g_s(N) and other costs
            g_forward_cost(neighbor) = tentative_g_cost;
            f_forward_cost(neighbor) = g_forward_cost(neighbor) + hs_cost_gamma(satellite_coords, neighbor, start_sat, end_sat, gamma);

            update_cost_count = update_cost_count + 1;

        end

        % intersect with backward direction
        if g_backward_cost(current_N) < inf
            intersect_list = [intersect_list; current_N];
            % update shortest path
            w_new = g_forward_cost(current_N) + g_backward_cost(current_N);

            if w_new < w
                w = w_new;
                intersect_sat = current_N;
                % fprintf('[forward] sat %d update w with %.4f\n', current_N, w);
            end
        end

        %% backward direction
        [~, min_idx] = min(f_backward_cost(open_backward_list));
        current_back_N = open_backward_list(min_idx);
            
        % fprintf('\n[DBS_OP backward] current back N: %d\n', current_back_N);

        % exclude min_N from open list
        open_backward_list(min_idx) = [];
        closed_backward_list = [closed_backward_list; current_back_N];
        n_visited = n_visited + 1;
        visited_hash(current_back_N) = 1;
        iter_count = iter_count + 1;

        % traverse all connectable neighbors 
        neighbors = find_neighbors_Dmax(satellite_coords, current_back_N, ...
        earth_radius, atm_high, orbit_altitudes, n_sat_list);

        % fprintf('[DBS_OP backward] %d sat: found %d neighbors\n', current_back_N, length(neighbors));
        for i = 1:length(neighbors)
            neighbor = neighbors(i);
            % check if `neighbor` already in closed list, prevent re-visit
            if ismember(neighbor, closed_backward_list)
                continue;
            end

            n_visited = n_visited + 1;
            visited_hash(neighbor) = 1;

            % init, current_back_N is the starting node
            tentative_g_cost = g_backward_cost(current_back_N) + distance_between(satellite_coords, current_back_N, neighbor);

            if ~ismember(neighbor, open_backward_list)
                open_backward_list = [open_backward_list; neighbor];
            elseif tentative_g_cost >= g_backward_cost(neighbor)
                continue;
            end

            % record current parent of N
            back_parents(neighbor) = current_back_N;
            % update g_d(N) and other costs
            g_backward_cost(neighbor) = tentative_g_cost;
            f_backward_cost(neighbor) = g_backward_cost(neighbor) + hs_cost_gamma(satellite_coords, neighbor, end_sat, start_sat, gamma);

            update_cost_count = update_cost_count + 1;

        end

        % intersect with forward direction
        if g_forward_cost(current_back_N) < inf
            intersect_list = [intersect_list; current_back_N];
            % update shortest path
            w_new = g_forward_cost(current_back_N) + g_backward_cost(current_back_N);

            if w_new < w
                w = w_new;
                intersect_sat = current_back_N;
                % fprintf('[backward] sat %d update w with %.4f\n', current_back_N, w);
            end
        end

        %% end condition
        % check if w gets updated (when gs(N) + gd(N) >= w stop)
        if w < inf && w_old == w 
            sat = intersect_sat;
            while sat ~= end_sat
                parents(back_parents(sat)) = sat;
                sat = back_parents(sat);
            end

            path = reconstruct_path_to_end(parents, end_sat);
            n_visited = sum(visited_hash);
            elapsed_time = toc; % 結束計時
            return;
        end

    end
end