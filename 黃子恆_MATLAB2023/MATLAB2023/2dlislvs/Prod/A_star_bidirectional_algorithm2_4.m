% 雙向A*輔助函數
function [path, total_distance, elapsed_time, visited_nodes, search_count] = A_star_bidirectional_algorithm2_4(satellite_coords, start_satellite, end_satellite, total_satellites_sum)
    tic; % 開始計時
    N = 1.3; % 可以根據需要更改
    open_list_start = [start_satellite];
    open_list_end = [end_satellite];
    closed_list_start = [];
    closed_list_end = [];
    
    g_cost_start = inf(total_satellites_sum, 1);
    g_cost_end = inf(total_satellites_sum, 1);
    
    f_cost_start = inf(total_satellites_sum, 1);
    f_cost_end = inf(total_satellites_sum, 1);
    
    came_from_start = zeros(total_satellites_sum, 1);
    came_from_end = zeros(total_satellites_sum, 1);
    
    g_cost_start(start_satellite) = 0;
    g_cost_end(end_satellite) = 0;
    
    f_cost_start(start_satellite) = heuristic_cost(satellite_coords, start_satellite, end_satellite);
    f_cost_end(end_satellite) = heuristic_cost(satellite_coords, end_satellite, start_satellite);
    
    search_count = 0; % 初始化搜索次數
    search_limit = inf; % 初始化搜索上限

    best_total_distance = inf; % 初始化最佳總距離
    best_path = []; % 初始化最佳路徑
    while ~isempty(open_list_start) && ~isempty(open_list_end) && search_count <= search_limit
        search_count = search_count + 1; % 每次搜索時增加搜索次數

        % 開始方向搜尋
        [~, min_index_start] = min(f_cost_start(open_list_start));
        current_satellite_start = open_list_start(min_index_start);
        open_list_start(min_index_start) = [];
        closed_list_start = [closed_list_start; current_satellite_start];
        neighbors_start = find_neighbors(satellite_coords, current_satellite_start);
        
        % 終點方向搜尋
        [~, min_index_end] = min(f_cost_end(open_list_end));
        current_satellite_end = open_list_end(min_index_end);
        open_list_end(min_index_end) = [];
        closed_list_end = [closed_list_end; current_satellite_end];
        neighbors_end = find_neighbors(satellite_coords, current_satellite_end);

        % 檢查是否相交
        intersect_nodes = intersect(open_list_start, open_list_end);
        if ~isempty(intersect_nodes)
            % 計算所有交集點的總路徑成本
            total_costs = arrayfun(@(node) g_cost_start(node) + g_cost_end(node), intersect_nodes);
            % 找出具有最小總成本的交集節點
            [~, min_cost_index] = min(total_costs);
            total_distance = total_costs(min_cost_index);
            intersect_node = intersect_nodes(min_cost_index);
            
            if total_distance < best_total_distance
                % 若找到更好的路徑則更新最佳路徑和最佳距離
                path_from_start = reconstruct_path(came_from_start, intersect_node);
                path_from_end = flip(reconstruct_path(came_from_end, intersect_node));
                % 將兩部分路徑合併，並確保交集節點不重複
                best_path = [path_from_start; path_from_end(2:end)];
                best_total_distance = total_distance;
                
                % 更新搜索上限
                search_limit = search_count * N;
            end
        end

        % 擴展從起始點方向的節點
        [g_cost_start, f_cost_start, open_list_start, came_from_start] = expand_node(satellite_coords, neighbors_start, current_satellite_start, end_satellite, g_cost_start, f_cost_start, open_list_start, closed_list_start, came_from_start);
        
        % 擴展從終點方向的節點
        [g_cost_end, f_cost_end, open_list_end, came_from_end] = expand_node(satellite_coords, neighbors_end, current_satellite_end, start_satellite, g_cost_end, f_cost_end, open_list_end, closed_list_end, came_from_end);
    end
    
    % 若未找到路徑
    if isempty(best_path)
        path = [];
        total_distance = 0;
        fprintf('BA* algorithm 找不到路徑\n');
    else
        path = best_path;
        total_distance = best_total_distance;
    end

    elapsed_time = toc; % 結束計時
    visited_nodes = union(closed_list_start, closed_list_end);  % 紀錄經過的節點
end



% 輔助函數:擴展節點
function [g_cost, f_cost, open_list, came_from] = expand_node(satellite_coords, neighbors, current_satellite, target_satellite, g_cost, f_cost, open_list, closed_list, came_from)
    for i = 1:length(neighbors)
        neighbor = neighbors(i);
        if ismember(neighbor, closed_list)
            continue;
        end
        
        tentative_g_cost = g_cost(current_satellite) + distance_between(satellite_coords, current_satellite, neighbor);
        
        if ~ismember(neighbor, open_list)
            open_list = [open_list; neighbor];
        elseif tentative_g_cost >= g_cost(neighbor)
            continue;
        end
        
        came_from(neighbor) = current_satellite;
        g_cost(neighbor) = tentative_g_cost;
        f_cost(neighbor) = g_cost(neighbor) + heuristic_cost(satellite_coords, neighbor, target_satellite);
    end
end

% 輔助函數:重新建構路徑
function [path] = reconstruct_path(came_from, current_node)
    path = [current_node];
    while came_from(current_node) ~= 0
        current_node = came_from(current_node);
        path = [current_node; path];
    end
end
