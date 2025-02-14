% 雙向 A* 輔助函數
function [path, total_distance, elapsed_time, visited_nodes_dijk, search_count_dijk] = BA_DBS_uv_2(satellite_coords, start_satellite, end_satellite, total_satellites_sum)

    tic; % 開始計時
    distance_weight = 1;
    angle_weight = 1;
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

        % 從起始點開始搜索
        [~, idx_start] = min(distance_start(unvisited_start) + estimate_cost(satellite_coords, unvisited_start, start_satellite, end_satellite, distance_weight, angle_weight));
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
        
        % 從目標點開始搜索
        [~, idx_end] = min(distance_end(unvisited_end) + estimate_cost(satellite_coords, unvisited_end, end_satellite, start_satellite, distance_weight, angle_weight));
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
        
        % 檢查是否已經可以停止搜索
        if distance_start(current_satellite_start) + distance_end(current_satellite_end) >= mu
            path_from_start = reconstruct_path(previous_start, intersect_node);
            path_from_end = flip(reconstruct_path(previous_end, intersect_node));
            % 將兩部分路徑合併，並確保交集節點不重複
            path = [path_from_start; path_from_end(2:end)];
            total_distance = distance_start(intersect_node) + distance_end(intersect_node);
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

% 估計函數，包含距離與角度的考量
function cost = estimate_cost(satellite_coords, nodes, source, target, distance_weight, angle_weight)
    cost = zeros(1, length(nodes));
    source_coords = satellite_coords(source, 1:3);
    target_coords = satellite_coords(target, 1:3);

    for i = 1:length(nodes)
        node_coords = satellite_coords(nodes(i), 1:3);
        distance_cost = norm(node_coords - target_coords);
        node_coords_relative = node_coords - source_coords;
        target_coords_relative = target_coords - node_coords;
        angle_cost = angle_between(node_coords_relative, target_coords_relative);
        cost(i) = (distance_weight * distance_cost) * (angle_cost);
    end
end

% 計算兩個向量之間的角度
function angle = angle_between(v1, v2)
    dot_product = dot(v1, v2);
    norms = norm(v1) * norm(v2);
    cos_theta = dot_product / norms;
    angle = acos(cos_theta);  % 輸出為弧度制
end