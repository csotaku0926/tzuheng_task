function [path, total_distance, elapsed_time, visited_nodes, search_count] = A_star_bidirectional_algorithm4(satellite_coords, start_satellite, end_satellite, total_satellites_sum, BA_search_count_limit)
    
    max_search_count = BA_search_count_limit;
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
    intersect_nodes = []; % 新增一個用於儲存交集節點的空陣列
    tic; % 開始計時
    while ~isempty(open_list_start) && ~isempty(open_list_end)
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
        intersect_candidate = intersect(open_list_start, open_list_end);
        if ~isempty(intersect_candidate)
            % 計算所有交集點的總路徑成本
            total_costs = arrayfun(@(node) g_cost_start(node) + g_cost_end(node), intersect_candidate);
            % 將交集節點和對應的成本加入到交集節點列表中
            intersect_nodes = [intersect_nodes; [intersect_candidate, total_costs]];
        end
        
        if search_count >= max_search_count  % 如果搜索次數達到上限，則停止搜索
            break;
        end

        % 擴展從起始點方向的節點
        [g_cost_start, f_cost_start, open_list_start, came_from_start] = expand_node(satellite_coords, neighbors_start, current_satellite_start, end_satellite, g_cost_start, f_cost_start, open_list_start, closed_list_start, came_from_start);
        
        % 擴展從終點方向的節點
        [g_cost_end, f_cost_end, open_list_end, came_from_end] = expand_node(satellite_coords, neighbors_end, current_satellite_end, start_satellite, g_cost_end, f_cost_end, open_list_end, closed_list_end, came_from_end);
    end
    
    if ~isempty(intersect_nodes)
        % 從交集節點中選擇總成本最小的節點
        [~, min_cost_index] = min(intersect_nodes(:, 2));
        intersect_node = intersect_nodes(min_cost_index, 1);
        path_from_start = reconstruct_path(came_from_start, intersect_node);
        path_from_end = flip(reconstruct_path(came_from_end, intersect_node));
        % 將兩部分路徑合併，並確保交集節點不重複
        path = [path_from_start; path_from_end(2:end)];
        total_distance = g_cost_start(intersect_node) + g_cost_end(intersect_node);
    else
        % 若未找到路徑
        path = [];
        total_distance = 0;
    end

    elapsed_time = toc;
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
