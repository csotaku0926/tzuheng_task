% Bellman-Ford 輔助函數
function [path, total_distance, elapsed_time, visited_nodes_bell, search_count_bell] = BellmanFord_algorithm(satellite_coords, start_satellite, end_satellite, total_satellites, max_dis_perhop)

tic; % 開始計時

visited_nodes_bell = [];
distance = inf(1, total_satellites);
distance(start_satellite) = 0;

previous = zeros(1, total_satellites);
search_count_bell = 0; % 初始化搜索次數

change_flag = 1; % 用來檢查是否所有節點都被檢查過的標誌位
for i = 1:total_satellites - 1
    if ~change_flag
        break; % 如果所有節點都被檢查過，提前跳出迴圈
    end
    change_flag = 0;
    for current_satellite = 1:total_satellites
        neighbors = find_neighbors(satellite_coords, current_satellite, max_dis_perhop);
        for neighbor_idx = 1:length(neighbors)
            neighbor = neighbors(neighbor_idx);
            alt = distance(current_satellite) + distance_between(satellite_coords, current_satellite, neighbor);
            if alt < distance(neighbor)
                distance(neighbor) = alt;
                previous(neighbor) = current_satellite;
                change_flag = 1; % 如果有節點的距離被更新，設定標誌位
            end
        end
    end
    search_count_bell = search_count_bell + 1; % 每次搜索時增加搜索次數
end

if isnan(distance(end_satellite))
    % 若未找到路徑
    path = [];
    total_distance = 0;
    elapsed_time = toc; % 結束計時
    return;
else
    path = reconstruct_path(previous, end_satellite);
    total_distance = distance(end_satellite);
    visited_nodes_bell = unique(path);
    elapsed_time = toc; % 結束計時
end
end
