%function [search_count, path, total_distance, visited_satellites, execution_time, searched_satellites] = BellmanFord_algorithm_2(satellite_coords, start_satellite, end_satellite, total_satellites, max_dis_perhop)
function [search_count, path, total_distance, visited_satellites, execution_time, searched_satellites] = BellmanFord_algorithm_2(satellite_coords, start_satellite, end_satellite, total_satellites, max_dis_perhop)

tic; % 開始計時
% 初始化變數
dist = inf(1, total_satellites); % 距離矩陣
dist(start_satellite) = 0; % 起始衛星距離為0
predecessor = zeros(1, total_satellites); % 前驅矩陣
searched_satellites = []; % 搜索過的衛星

% 優化後的鄰接矩陣
edges = [];
for i = 1:total_satellites
    for j = i+1:total_satellites
        dis = norm(satellite_coords(i, :) - satellite_coords(j, :));
        if dis <= max_dis_perhop
            edges = [edges; i, j, dis]; % 儲存邊的信息（起點，終點，距離）
            searched_satellites = [searched_satellites; i, j]; % 儲存已搜索的衛星
        end
    end
end

% Bellman-Ford演算法主體
for i = 1:(total_satellites-1)
    for edge = edges'
        if dist(edge(1)) + edge(3) < dist(edge(2))
            dist(edge(2)) = dist(edge(1)) + edge(3);
            predecessor(edge(2)) = edge(1);
        end
    end
end

% 檢查負權重循環
for edge = edges'
    if dist(edge(1)) + edge(3) < dist(edge(2))
        error('圖中存在負權重循環');
    end
end

% 從終點反向追蹤找出路徑
path = [];
temp_node = end_satellite;
while temp_node ~= start_satellite
    path = [temp_node, path];
    temp_node = predecessor(temp_node);
end
path = [start_satellite, path]; % 將起始節點加入到路徑中
total_distance = dist(end_satellite); % 總路徑長度
visited_satellites = find(dist ~= inf); % 已訪問的節點
search_count = length(edges); % 搜索次數
execution_time = toc; % 計算執行時間
end
