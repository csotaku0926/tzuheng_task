% 主函數
function [path, total_distance, elapsed_time, visited_nodes, search_count] = A_star_bidirectional_algorithm_dynamic(satellite_coords, start_satellite, end_satellite, total_satellites_sum)

    % 計算原始路徑
    [path, total_distance, elapsed_time, visited_nodes, search_count] = A_star_bidirectional_algorithm3(satellite_coords, start_satellite, end_satellite, total_satellites_sum);

    % 刪除一個節點，並從該節點的前後兩個節點開始計算最短路徑
    if length(path) > 2
        % 隨機選取一個節點刪除（除了起點和終點）
        rand_idx = randi([2, length(path)-1]);
        deleted_node = path(rand_idx);
        % 重新定義起點和終點
        start_satellite = path(rand_idx-1);
        end_satellite = path(rand_idx+1);
        % 計算新的路徑
        %[new_path, new_total_distance, new_elapsed_time, new_visited_nodes, new_search_count] = A_star_bidirectional_algorithm2(satellite_coords, start_satellite, end_satellite, total_satellites_sum);
        [new_path, new_total_distance, new_elapsed_time, new_visited_nodes, new_search_count] = A_star_algorithm_delete(satellite_coords, start_satellite, end_satellite, total_satellites_sum, deleted_node);
        % 更新路徑，總距離，經過的時間，訪問的節點，和搜索次數
        path = [path(1:rand_idx-1); new_path; path(rand_idx+2:end)];
        %total_distance = total_distance - distance_between(satellite_coords, path(rand_idx-1), deleted_node) - distance_between(satellite_coords, deleted_node, path(rand_idx+1)) + new_total_distance;
        % 初始化總距離變數
        total_distance = 0;

        % 遍歷 path 中的每個索引，並計算相鄰座標之間的距離
        for i = 1:length(path) - 1
            % 從 satellite_coords 中提取座標
            coord1 = satellite_coords(path(i), :);
            coord2 = satellite_coords(path(i+1), :);

            % 計算座標之間的距離
            distance = norm(coord1 - coord2);

            % 將距離加入總距離
            total_distance = total_distance + distance;
        end
        %fprintf('整段路徑的長度為：%.2f單位\n', total_distance); % 輸出總距離

        elapsed_time = new_elapsed_time;
        %visited_nodes = union(visited_nodes, new_visited_nodes);
        visited_nodes = new_visited_nodes;
        search_count = new_search_count;

        % 移除路徑中的重複節點
        [~, idx] = unique(path, 'stable');
        path = path(idx);
    end
end
