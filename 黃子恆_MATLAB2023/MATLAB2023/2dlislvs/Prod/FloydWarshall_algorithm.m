function [dist, elapsed_time, search_count] = FloydWarshall_algorithm(satellite_coords, total_satellites, max_dis_perhop)

tic; % 開始計時
search_count = 0; % 初始化搜索次數

% 初始化距離矩陣
dist = inf(total_satellites, total_satellites);
for satellite = 1:total_satellites
    dist(satellite, satellite) = 0;  % 自身到自身的距離為0
    neighbors = find_neighbors(satellite_coords, satellite, max_dis_perhop);
    for i = 1:length(neighbors)
        neighbor = neighbors(i);
        dist(satellite, neighbor) = distance_between(satellite_coords, satellite, neighbor);
    end
end

% Floyd-Warshall 主循環
for k = 1:total_satellites
    for i = 1:total_satellites
        for j = 1:total_satellites
            search_count = search_count + 1; % 每次搜索時增加搜索次數
            if dist(i, j) > dist(i, k) + dist(k, j)
                dist(i, j) = dist(i, k) + dist(k, j);
            end
        end
    end
end

elapsed_time = toc; % 結束計時
end
