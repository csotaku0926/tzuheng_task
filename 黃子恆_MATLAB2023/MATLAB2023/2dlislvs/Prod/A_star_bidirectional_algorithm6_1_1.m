% 雙向 A* 輔助函數
function [path, total_distance, elapsed_time, visited_nodes_astar, search_count_astar] = A_star_bidirectional_algorithm6_1(satellite_coords, start_satellite, end_satellite, total_satellites_sum)
    tic; % 開始計時
    unvisited_start = 1:total_satellites_sum;
    unvisited_end = 1:total_satellites_sum;
    visited_nodes_astar_start = [];
    visited_nodes_astar_end = [];
    
    distance_start = inf(1, total_satellites_sum);
    distance_end = inf(1, total_satellites_sum);
    distance_start(start_satellite) = 0;
    distance_end(end_satellite) = 0;
    
    previous_start = zeros(1, total_satellites_sum);
    previous_end = zeros(1, total_satellites_sum);
    
    search_count_astar = 0; % 初始化搜索次數
    mu = inf;
    intersect_node = -1; % 初始化相交節點

    while ~isempty(unvisited_start) && ~isempty(unvisited_end)
        search_count_astar = search_count_astar + 1; % 每次搜索時增加搜索次數

        % 從起始點開始搜索
        [~, idx_start] = min(distance_start(unvisited_start) + heuristic_cost(satellite_coords, unvisited_start, end_satellite, false));
        current_satellite_start = unvisited_start(idx_start);
        unvisited_start(idx_start) = [];
        visited_nodes_astar_start = [visited_nodes_astar_start; current_satellite_start];
        
        neighbors_start = find_neighbors(satellite_coords, current_satellite_start);
        for i = 1:length(neighbors_start)
            neighbor = neighbors_start(i);
            alt_start = distance_start(current_satellite_start) + distance_between(satellite_coords, current_satellite_start, neighbor);
            if alt_start < distance_start(neighbor)
                distance_start(neighbor) = alt_start;
                previous_start(neighbor) = current_satellite_start;
            end
            if ismember(neighbor, visited_nodes_astar_end) && distance_start(neighbor) + distance_end(neighbor) < mu
                mu = distance_start(neighbor) + distance_end(neighbor);
                intersect_node = neighbor;
            end
        end
        
        % 從目標點開始搜索
        [~, idx_end] = min(distance_end(unvisited_end) + heuristic_cost(satellite_coords, unvisited_end, start_satellite, true));
        current_satellite_end = unvisited_end(idx_end);
        unvisited_end(idx_end) = [];
        visited_nodes_astar_end = [visited_nodes_astar_end; current_satellite_end];

        neighbors_end = find_neighbors(satellite_coords, current_satellite_end);
        for i = 1:length(neighbors_end)
            neighbor = neighbors_end(i);
            alt_end = distance_end(current_satellite_end) + distance_between(satellite_coords, current_satellite_end, neighbor);
            if alt_end < distance_end(neighbor)
                distance_end(neighbor) = alt_end;
                previous_end(neighbor) = current_satellite_end;
            end
            if ismember(neighbor, visited_nodes_astar_start) && distance_end(neighbor) + distance_start(neighbor) < mu
                mu = distance_end(neighbor) + distance_start(neighbor);
                intersect_node = neighbor;
            end
        end
        
        % 檢查是否已經可以停止搜索
        if distance_start(current_satellite_start) + heuristic_cost(satellite_coords, current_satellite_start, end_satellite, false) + distance_end(current_satellite_end) + heuristic_cost(satellite_coords, current_satellite_end, start_satellite, true) >= mu
            path_from_start = reconstruct_path(previous_start, intersect_node);
            path_from_end = flip(reconstruct_path(previous_end, intersect_node));
            % 將兩部分路徑合併，並確保交集節點不重複
            path = [path_from_start; path_from_end(2:end)];
            total_distance = distance_start(intersect_node) + distance_end(intersect_node);
            elapsed_time = toc; % 結束計時
            visited_nodes_astar = union(visited_nodes_astar_start, visited_nodes_astar_end);  % 將從兩邊開始的已訪問節點合併
            return;
        end
    end

    % 若未找到路徑
    path = [];
    total_distance = 0;
    elapsed_time = toc; % 結束計時
    visited_nodes_astar = union(visited_nodes_astar_start, visited_nodes_astar_end);  % 將從兩邊開始的已訪問節點合併
end


% 啟發式函數，點與點之間直線連接使用歐式距離
function cost = heuristic_cost(satellite_coords, current_satellite,start_satellite , end_satellite, isReverse)
    pi_t = norm(satellite_coords(current_satellite, 1:3) - satellite_coords(end_satellite, 1:3));
    pi_s = norm(satellite_coords(end_satellite, 1:3) - satellite_coords(current_satellite, 1:3));
    vector1 = satellite_coords(start_satellite, :) - satellite_coords(current_satellite, :);
    vector2 = satellite_coords(end_satellite, :) - satellite_coords(current_satellite, :);
    angle_t = angle_between(vector1,vector2);
    angle_s = angle_between(vector2,vector1);
    if isReverse
        cost = -(pi_t - pi_s) / 2 - angle_t;
    else
        cost = (pi_t - pi_s) / 2 + angle_s;
    end
end

% 輔助函數:角度約束
function angle = angle_between(v1, v2)
    cos_theta = dot(v1, v2) / (norm(v1) * norm(v2));
    angle = acosd(cos_theta);  % 使用acosd來得到角度（非弧度）
end