% Dijkstra輔助函數
function [path, total_distance, elapsed_time, visited_nodes_dijk, search_count_dijk] = Dijkstra_algorithm(satellite_coords, start_satellite, end_satellite, total_satellites_sum)

tic; % 開始計時
    unvisited = 1:total_satellites_sum;
    visited_nodes_dijk = [];
    
    distance = inf(1, total_satellites_sum);
    distance(start_satellite) = 0;
    
    previous = zeros(1, total_satellites_sum);
    
    search_count_dijk = 0; % 初始化搜索次數
    while ~isempty(unvisited)
        search_count_dijk = search_count_dijk + 1; % 每次搜索時增加搜索次數
        [~, idx] = min(distance(unvisited));
        current_satellite = unvisited(idx);
        
        if current_satellite == end_satellite
            path = reconstruct_path(previous, current_satellite);
            total_distance = distance(current_satellite);
            elapsed_time = toc; % 結束計時
            return;
        end
        
        unvisited(idx) = [];
        visited_nodes_dijk = [visited_nodes_dijk; current_satellite];  % 將訪問過的節點添加到 visited_nodes_dijk 陣列中
        
        neighbors = find_neighbors(satellite_coords, current_satellite);
        for i = 1:length(neighbors)
            neighbor = neighbors(i);
            if ismember(neighbor, visited_nodes_dijk)
                continue;
            end
            
            alt = distance(current_satellite) + distance_between(satellite_coords, current_satellite, neighbor);
            if alt < distance(neighbor)
                distance(neighbor) = alt;
                previous(neighbor) = current_satellite;
            end
        end
        elapsed_time = toc; % 結束計時
    end
    
    % 若未找到路徑
    path = [];
    total_distance = 0;
    elapsed_time = toc;
end
