% 單向 A* 輔助函數
function [path, total_distance, elapsed_time, visited_nodes_dijk, search_count_dijk] = A_star_uv(satellite_coords, start_satellite, end_satellite, total_satellites_sum)
    
    tic; % 開始計時
    unvisited_start = 1:total_satellites_sum;
    visited_nodes_dijk_start = [];
    
    distance_start = inf(1, total_satellites_sum);
    distance_start(start_satellite) = 0;
    
    previous_start = zeros(1, total_satellites_sum);
    
    search_count_dijk = 0; % 初始化搜索次數

    while ~isempty(unvisited_start)
        search_count_dijk = search_count_dijk + 1; % 每次搜索時增加搜索次數

        % 從起始點開始搜索
        [~, idx_start] = min(distance_start(unvisited_start) + estimate_cost(satellite_coords, unvisited_start, end_satellite));
        current_satellite_start = unvisited_start(idx_start);
        unvisited_start(idx_start) = [];
        visited_nodes_dijk_start = [visited_nodes_dijk_start; current_satellite_start];

        % 檢查是否已到達目標
        if current_satellite_start == end_satellite
            path = reconstruct_path(previous_start, end_satellite);
            total_distance = distance_start(end_satellite);
            elapsed_time = toc; % 結束計時
            visited_nodes_dijk = visited_nodes_dijk_start;
            return;
        end

        neighbors_start = find_neighbors(satellite_coords, current_satellite_start);
        for i = 1:length(neighbors_start)
            neighbor = neighbors_start(i);
            alt_start = distance_start(current_satellite_start) + distance_between(satellite_coords, current_satellite_start, neighbor);
            if alt_start < distance_start(neighbor)
                distance_start(neighbor) = alt_start;
                previous_start(neighbor) = current_satellite_start;
            end
        end
    end

    % 若未找到路徑
    path = [];
    total_distance = 0;
    elapsed_time = toc; % 結束計時
    visited_nodes_dijk = visited_nodes_dijk_start;
end

% 估計函數
function cost = estimate_cost(satellite_coords, nodes, target)
    cost = zeros(1, length(nodes));
    target_coords = satellite_coords(target, 1:3);
    for i = 1:length(nodes)
        cost(i) = norm(satellite_coords(nodes(i), 1:3) - target_coords);
    end
end
