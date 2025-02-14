function [path, total_distance, elapsed_time, visited_nodes_dijk, search_count_dijk] = BA_DBS_uv_3_1(satellite_coords, start_satellite, end_satellite, total_satellites_sum)

    tic; % 開始計時
    unvisited_start = 1:total_satellites_sum;
    unvisited_end = 1:total_satellites_sum;
    visited_nodes_dijk_start = [];
    visited_nodes_dijk_end = [];
    
    distance_start = inf(1, total_satellites_sum);
    distance_end = inf(1, total_satellites_sum);
    distance_start(start_satellite) = 0;
    distance_end(end_satellite) = 0;
    
    previous_start = zeros(1, total_satellites_sum);
    previous_end = zeros(1, total_satellites_sum);
    
    search_count_dijk = 0; % 初始化搜索次數
    mu = inf;
    intersect_node = -1; % 初始化相交節點
    
    while ~isempty(unvisited_start) && ~isempty(unvisited_end)
        search_count_dijk = search_count_dijk + 1; % 每次搜索時增加搜索次數

        [~, idx_start] = min(distance_start(unvisited_start) + estimate_cost(satellite_coords, unvisited_start, end_satellite, start_satellite));
        current_satellite_start = unvisited_start(idx_start);
        unvisited_start(idx_start) = [];
        visited_nodes_dijk_start = [visited_nodes_dijk_start; current_satellite_start];
        
        neighbors_start = find_neighbors(satellite_coords, current_satellite_start);
        for i = 1:length(neighbors_start)
            neighbor = neighbors_start(i);
            alt_start = distance_start(current_satellite_start) + distance_between(satellite_coords, current_satellite_start, neighbor);
            if alt_start < distance_start(neighbor)
                distance_start(neighbor) = alt_start;
                previous_start(neighbor) = current_satellite_start;
            end
            if ismember(neighbor, visited_nodes_dijk_end) && distance_start(neighbor) + distance_end(neighbor) < mu
                mu = distance_start(neighbor) + distance_end(neighbor);
                intersect_node = neighbor;
            end
        end
        
        [~, idx_end] = min(distance_end(unvisited_end) + estimate_cost(satellite_coords, unvisited_end, start_satellite, end_satellite));
        current_satellite_end = unvisited_end(idx_end);
        unvisited_end(idx_end) = [];
        visited_nodes_dijk_end = [visited_nodes_dijk_end; current_satellite_end];

        neighbors_end = find_neighbors(satellite_coords, current_satellite_end);
        for i = 1:length(neighbors_end)
            neighbor = neighbors_end(i);
            alt_end = distance_end(current_satellite_end) + distance_between(satellite_coords, current_satellite_end, neighbor);
            if alt_end < distance_end(neighbor)
                distance_end(neighbor) = alt_end;
                previous_end(neighbor) = current_satellite_end;
            end
            if ismember(neighbor, visited_nodes_dijk_start) && distance_end(neighbor) + distance_start(neighbor) < mu
                mu = distance_end(neighbor) + distance_start(neighbor);
                intersect_node = neighbor;
            end
        end
        disp(intersect_node);
        if distance_start(current_satellite_start) + distance_end(current_satellite_end) >= mu
            if intersect_node ~= -1
                path_from_start = reconstruct_path(previous_start, intersect_node);
                path_from_end = flip(reconstruct_path(previous_end, intersect_node));
                path = [path_from_start; path_from_end(2:end)];
                total_distance = distance_start(intersect_node) + distance_end(intersect_node);
            else
                % 若未找到路徑
                path = [];
                total_distance = 0;
            end
            elapsed_time = toc; % 結束計時
            visited_nodes_dijk = union(visited_nodes_dijk_start, visited_nodes_dijk_end);  % 將從兩邊開始的已訪問節點合併
            return;
        end
    end

    % 若未找到路徑
    path = [];
    total_distance = 0;
    elapsed_time = toc; % 結束計時
    visited_nodes_dijk = union(visited_nodes_dijk_start, visited_nodes_dijk_end);  % 將從兩邊開始的已訪問節點合併
end

function cost = estimate_cost(satellite_coords, nodes, target, start)
    cost = zeros(1, length(nodes));
    target_coords = satellite_coords(target, 1:3);
    start_coords = satellite_coords(start, 1:3);
    for i = 1:length(nodes)
        node_coords = satellite_coords(nodes(i), 1:3);
        distance_cost = norm(node_coords - target_coords);
        v1 = start_coords - node_coords;
        v2 = target_coords - node_coords;
        angle_cost = angle_between(v1, v2);

        cost(i) = distance_cost - angle_cost;
    end
end

function angle = angle_between(v1, v2)
    dot_product = dot(v1, v2);
    norms = norm(v1) * norm(v2);
    cos_theta = dot_product / norms;
    angle = acos(cos_theta); 
end

function path = reconstruct_path(came_from, current_satellite)
    total_path = [current_satellite];
    while came_from(current_satellite) > 0
        current_satellite = came_from(current_satellite);
        total_path = [current_satellite; total_path];
    end
    path = total_path;
end
