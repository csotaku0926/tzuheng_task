% 雙向A* 演算法
function [path, total_distance, elapsed_time, visited_nodes_bidir, search_count_bidir] = A_star_bidirectional_algorithm2_5(satellite_coords, start_satellite, end_satellite, total_satellites_sum)

tic; % 開始計時
    unvisited_from_start = 1:total_satellites_sum;
    unvisited_from_end = 1:total_satellites_sum;
    
    distance_from_start = inf(1, total_satellites_sum);
    distance_from_start(start_satellite) = 0;
    distance_from_end = inf(1, total_satellites_sum);
    distance_from_end(end_satellite) = 0;
    
    heuristic_from_start = euclidean_distance_3D(satellite_coords, start_satellite, end_satellite);
    heuristic_from_end = euclidean_distance_3D(satellite_coords, end_satellite, start_satellite);
    
    previous_from_start = zeros(1, total_satellites_sum);
    previous_from_end = zeros(1, total_satellites_sum);
    
    visited_nodes_bidir_from_start = [];
    visited_nodes_bidir_from_end = [];
    
    search_count_bidir = 0; % 初始化搜索次數
    
    while ~isempty(unvisited_from_start) && ~isempty(unvisited_from_end)
        search_count_bidir = search_count_bidir + 1; % 每次搜索時增加搜索次數
        
        % 從起始點開始的搜索
        if ~isempty(unvisited_from_start)
            [~, idx_from_start] = min(distance_from_start(unvisited_from_start) + heuristic_from_start(unvisited_from_start));
            current_satellite_from_start = unvisited_from_start(idx_from_start);
            unvisited_from_start(idx_from_start) = [];
            visited_nodes_bidir_from_start = [visited_nodes_bidir_from_start; current_satellite_from_start];  % 將訪問過的節點添加到 visited_nodes_bidir_from_start 陣列中
            neighbors_from_start = find_neighbors(satellite_coords, current_satellite_from_start);
            for i = 1:length(neighbors_from_start)
                neighbor = neighbors_from_start(i);
                if ismember(neighbor, visited_nodes_bidir_from_start)
                    continue;
                end
                alt = distance_from_start(current_satellite_from_start) + distance_between(satellite_coords, current_satellite_from_start, neighbor);
                if alt < distance_from_start(neighbor)
                    distance_from_start(neighbor) = alt;
                    previous_from_start(neighbor) = current_satellite_from_start;
                    heuristic_from_start(neighbor) = euclidean_distance_3D(satellite_coords, neighbor, end_satellite);
                end
            end
        end
        
        % 從終點開始的搜索
        if ~isempty(unvisited_from_end)
            [~, idx_from_end] = min(distance_from_end(unvisited_from_end) + heuristic_from_end(unvisited_from_end));
            current_satellite_from_end = unvisited_from_end(idx_from_end);
            unvisited_from_end(idx_from_end) = [];
            visited_nodes_bidir_from_end = [visited_nodes_bidir_from_end; current_satellite_from_end];  % 將訪問過的節點添加到 visited_nodes_bidir_from_end 陣列中
            neighbors_from_end = find_neighbors(satellite_coords, current_satellite_from_end);
            for i = 1:length(neighbors_from_end)
                neighbor = neighbors_from_end(i);
                if ismember(neighbor, visited_nodes_bidir_from_end)
                    continue;
                end
                alt = distance_from_end(current_satellite_from_end) + distance_between(satellite_coords, current_satellite_from_end, neighbor);
                if alt < distance_from_end(neighbor)
                    distance_from_end(neighbor) = alt;
                    previous_from_end(neighbor) = current_satellite_from_end;
                    heuristic_from_end(neighbor) = euclidean_distance_3D(satellite_coords, neighbor, start_satellite);
                end
            end
        end
        
        % 檢查是否有交集
        if ismember(current_satellite_from_start, visited_nodes_bidir_from_end) || ismember(current_satellite_from_end, visited_nodes_bidir_from_start)
            [intersect_nodes,~,~] = intersect(visited_nodes_bidir_from_start, visited_nodes_bidir_from_end);
            [~, idx] = min(distance_from_start(intersect_nodes) + distance_from_end(intersect_nodes));
            intersect_node = intersect_nodes(idx);
            [path, total_distance] = reconstruct_bidirectional_path(previous_from_start, previous_from_end, intersect_node, distance_from_start, distance_from_end);
            elapsed_time = toc; % 結束計時
            visited_nodes_bidir = union(visited_nodes_bidir_from_start, visited_nodes_bidir_from_end);
            return;
        end
    end
    
    % 若未找到路徑
    path = [];
    total_distance = 0;
    elapsed_time = toc;
    visited_nodes_bidir = union(visited_nodes_bidir_from_start, visited_nodes_bidir_from_end);
end

% 重構雙向路徑
function [path, total_distance] = reconstruct_bidirectional_path(previous_from_start, previous_from_end, intersect_node, distance_from_start, distance_from_end)
    path_from_start = [];
    path_from_end = [];
    current_node = intersect_node;
    
    while current_node ~= 0
        path_from_start = [current_node, path_from_start];
        current_node = previous_from_start(current_node);
    end
    
    current_node = intersect_node;
    while current_node ~= 0
        path_from_end = [current_node, path_from_end];
        current_node = previous_from_end(current_node);
    end
    
    path = [path_from_start, path_from_end(2:end)];
    total_distance = distance_from_start(intersect_node) + distance_from_end(intersect_node);
end

% 計算三維歐式距離
function distance = euclidean_distance_3D(coords, node1, node2)
    distance = sqrt(sum((coords(node1,:) - coords(node2,:)).^2));
end

% 尋找鄰居節點
function neighbors = find_neighbors(coords, node)
    distance_threshold = 200; % 定義鄰居的閾值，根據實際情況調整
    distances = sqrt(sum((coords - coords(node,:)).^2, 2));
    neighbors = find(distances <= distance_threshold & distances > 0);
end

% 計算兩個節點之間的距離
function distance = distance_between(coords, node1, node2)
    distance = sqrt(sum((coords(node1,:) - coords(node2,:)).^2));
end
