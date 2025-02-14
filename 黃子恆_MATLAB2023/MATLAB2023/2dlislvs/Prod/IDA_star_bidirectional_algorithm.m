% 雙向IDA*輔助函數
function [path, total_distance, elapsed_time, visited_nodes, search_count] = IDA_star_bidirectional_algorithm(satellite_coords, start_satellite, end_satellite, total_satellites_sum)
    tic; % 開始計時
    start_threshold = heuristic_cost(satellite_coords, start_satellite, end_satellite);
    end_threshold = heuristic_cost(satellite_coords, end_satellite, start_satellite);
    
    while true
        visited_nodes = []; % 紀錄訪問的節點
        search_count = 0; % 初始化搜索次數
        [path, g_cost, found, new_threshold, visited_nodes, search_count] = search_path(satellite_coords, start_satellite, end_satellite, 0, start_threshold, [], visited_nodes, search_count);
        
        if found 
            total_distance = g_cost;
            elapsed_time = toc; % 結束計時
            return;
        elseif isinf(new_threshold)
            % 若未找到路徑
            path = [];
            total_distance = 0;
            elapsed_time = toc;
            fprintf('Bi-Directional IDA* algorithm 找不到路徑\n');
            return;
        end
        
        [path_r, g_cost_r, found_r, new_threshold_r, visited_nodes, search_count] = search_path(satellite_coords, end_satellite, start_satellite, 0, end_threshold, [], visited_nodes, search_count);
        
        if found_r
            path = flipud(path_r);
            total_distance = g_cost_r;
            elapsed_time = toc; % 結束計時
            return;
        elseif isinf(new_threshold_r)
            % 若未找到路徑
            path = [];
            total_distance = 0;
            elapsed_time = toc;
            fprintf('Bi-Directional IDA* algorithm 找不到路徑\n');
            return;
        end
        
        start_threshold = new_threshold;
        end_threshold = new_threshold_r;
    end
end

% 輔助函數:搜尋路徑
function [path, g_cost, found, new_threshold, visited_nodes, search_count] = search_path(satellite_coords, current_satellite, target_satellite, g_cost, threshold, path, visited_nodes, search_count)
    search_count = search_count + 1; % 每次搜索時增加搜索次數
    visited_nodes = [visited_nodes; current_satellite]; % 將當前節點加入到訪問的節點列表中
    f_cost = g_cost + heuristic_cost(satellite_coords, current_satellite, target_satellite); % 計算f值

    if f_cost > threshold
        found = false;
        new_threshold = f_cost;
        return;
    end

    if current_satellite == target_satellite
        path = [path; current_satellite];
        found = true;
        new_threshold = threshold;
        return;
    end

    neighbors = find_neighbors(satellite_coords, current_satellite);
    new_threshold = inf;
    for i = 1:length(neighbors)
        neighbor = neighbors(i);
        if ismember(neighbor, path)
            continue;
        end
        
        [path_next, g_cost_next, found_next, new_threshold_next, visited_nodes, search_count] = search_path(satellite_coords, neighbor, target_satellite, g_cost + distance_between(satellite_coords, current_satellite, neighbor), threshold, [path; current_satellite], visited_nodes, search_count);
        
        if found_next
            path = path_next;
            g_cost = g_cost_next;
            found = true;
            new_threshold = threshold;
            return;
        end

        new_threshold = min(new_threshold, new_threshold_next);
    end

    path = [];
    found = false;
    return;
end
