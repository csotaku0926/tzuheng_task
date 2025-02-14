% 主函數
function [path, total_distance, elapsed_time, visited_nodes, search_count] = A_star_bidirectional_algorithm_dynamic_2(satellite_coords, start_satellite, end_satellite, total_satellites_sum)

    % 計算原始路徑
    [path, total_distance, elapsed_time, visited_nodes, search_count] = A_star_bidirectional_algorithm2(satellite_coords, start_satellite, end_satellite, total_satellites_sum);

    % 刪除一個節點，並從該節點的前後兩個節點開始計算最短路徑
    if length(path) > 2
        % 隨機選取一個節點刪除（除了起點和終點）
        rand_idx = randi([2, length(path)-1]);
        deleted_node = path(rand_idx);
        % 重新定義起點和終點
        start_satellite = path(rand_idx-1);
        end_satellite = path(rand_idx+1);
        % 計算新的路徑
        [new_path, new_total_distance, new_elapsed_time, new_visited_nodes, new_search_count] = A_star_bidirectional_algorithm2(satellite_coords, start_satellite, end_satellite, total_satellites_sum);
        % 更新路徑，總距離，經過的時間，訪問的節點，和搜索次數
        path = [path(1:rand_idx-1); new_path; path(rand_idx+2:end)];
        total_distance = total_distance - distance_between(satellite_coords, path(rand_idx-1), deleted_node) - distance_between(satellite_coords, deleted_node, path(rand_idx+1)) + new_total_distance;
        elapsed_time = elapsed_time + new_elapsed_time;
        visited_nodes = union(visited_nodes, new_visited_nodes);
        search_count = search_count + new_search_count;
    end
end
