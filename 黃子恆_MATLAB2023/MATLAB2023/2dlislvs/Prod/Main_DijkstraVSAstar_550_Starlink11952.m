clc
clear
% 參數設定
earth_radius = 6378;            % 地球半徑
orbit_altitude = 550;           % 衛星海拔(km)
total_satellites = 11952;       % 衛星數量
num_orbits = 72;                % 衛星軌道數
inclination = 53;               % 軌道傾角
atm_high = 80;                  % 含水氣之大氣層海拔高度
light_speed = 299792.458;       % 光速
% 根據衛星高度與大氣層厚度計算最長hop距離
max_dis_perhop = floor(sqrt((earth_radius + orbit_altitude)^2 - (earth_radius + atm_high)^2) * 2); 
% 輸出參數資訊
fprintf('衛星海拔高度: %.fkm\n', orbit_altitude);
fprintf('衛星數量: %.f顆\n', total_satellites);
fprintf('最大LISL: %.fkm\n', max_dis_perhop);

% 創建 WGS84 參考橢球體
wgs84 = referenceEllipsoid('WGS84');

% Calculate satellite positions
satellites_per_orbit = total_satellites / num_orbits; % 計算每條軌道的衛星數量
satellite_coords = zeros(total_satellites, 3);        % 建立衛星三維座標陣列
index = 1;

% 生成衛星座標
for orbit = 1:num_orbits
    RAAN = (orbit - 1) * 360 / num_orbits;
    for satellite = 1:satellites_per_orbit
        true_anomaly = (satellite - 1) * 360 / satellites_per_orbit;
        [x, y, z] = calculate_satellite_coordinates(earth_radius, orbit_altitude, RAAN, inclination, true_anomaly);
        satellite_coords(index, :) = [x, y, z];
        index = index + 1;
    end
end

% 定義地面站座標
gs_s_coords = [40.7128, -74.0060];  % 起始地面站
gs_d_coords = [-33.8688, 151.2093]; % 目標地面站

% 轉換地面站座標為三維座標
gs_s_3D = zeros(1, 3);
gs_d_3D = zeros(1, 3);

[gs_s_3D(1), gs_s_3D(2), gs_s_3D(3)] = llh2xyz(gs_s_coords(1), gs_s_coords(2), 0, earth_radius);
[gs_d_3D(1), gs_d_3D(2), gs_d_3D(3)] = llh2xyz(gs_d_coords(1), gs_d_coords(2), 0, earth_radius);

% 找到距離紐約和雪梨最近的衛星
start_satellite = 0;
end_satellite = 0;
min_distance_gs_s = inf;
min_distance_gs_d = inf;

for i = 1:total_satellites
    distance_gs_s = norm(gs_s_3D - satellite_coords(i, :));
    distance_gs_d = norm(gs_d_3D - satellite_coords(i, :));
    
    if distance_gs_s < min_distance_gs_s
        min_distance_gs_s = distance_gs_s;
        start_satellite = i;
    end
    
    if distance_gs_d < min_distance_gs_d
        min_distance_gs_d = distance_gs_d;
        end_satellite = i;
    end
end
% 將衛星的經緯度高程轉換為地球中心地球固定（ECEF）坐標
satellite_ecef_coords = zeros(total_satellites, 3);
for i = 1:total_satellites
    [x, y, z] = geodetic2ecef(wgs84, satellite_coords(i, 1), satellite_coords(i, 2), satellite_coords(i, 3));
    satellite_ecef_coords(i, :) = [x, y, z];
end

% 建立一個陣列來保存所有邊的信息_Bellman-Ford
%fprintf('Bellman-Ford algorithm Edge calcu.\n');
%edges = [];
%count = 0;
%count2 = 0;
%for i = 1:total_satellites
%    for j = i+1:total_satellites
%        count = count + 1;
%        fprintf('Bellman-Ford algorithm Edge calcu ij %d .\n', count);
%        distance = norm(satellite_coords(i, :) - satellite_coords(j, :));
%        if distance <= max_dis_perhop
%            edges = [edges; i j distance];
%            count2 = count2 + 1;
%            fprintf('Bellman-Ford algorithm Edge edge refresh %d .\n', count2);
%        end
%    end
%end
%fprintf('Bellman-Ford algorithm Edge calcu end.\n');

% 使用Bellman-Ford演算法找到最短衛星路徑
%[satellite_path_Bellman, total_satellite_distance_Bellman, Bellman_time, visited_nodes_bell, search_count_bell] = BellmanFord_algorithm_2(satellite_coords, start_satellite, end_satellite, total_satellites, max_dis_perhop);
%[search_count_bell, path, total_satellite_distance_Bellman, visited_satellites, execution_time, searched_satellites] = BellmanFord_algorithm_2(satellite_coords, start_satellite, end_satellite, total_satellites, max_dis_perhop);
%[satellite_path_Bellman, total_satellite_distance_Bellman, Bellman_time, visited_nodes_bell, search_count_bell] = BellmanFord_algorithm(satellite_coords, start_satellite, end_satellite, total_satellites, max_dis_perhop);
%[satellite_path_Bellman, total_satellite_distance_Bellman, Bellman_Ford_time] = BellmanFord_algorithm_3(edges, start_satellite, end_satellite, total_satellites);

%fprintf('Bellman-Ford algorithm executed %d searches.\n',Bellman_Ford_time);
% 計算Bellman-Ford路徑總距離，包括地面站到最近衛星的距離
%total_distance_bell = total_satellite_distance_Bellman + min_distance_gs_s + min_distance_gs_d;
%fprintf('Bellman-Ford演算法花費時間: %.4f秒\n', Bellman_time);
%fprintf('Bellman-Ford algorithm executed %d searches.\n', search_count_bell);
%fprintf('總路徑距離: %.2f公里\n', total_distance_bell);

% 使用A*演算法找到最短衛星路徑
[satellite_path_A_star, total_satellite_distance_A_star, A_star_time, visited_nodes, search_count_a_star] = A_star_algorithm(satellite_coords, start_satellite, end_satellite, total_satellites, max_dis_perhop);
fprintf('A* algorithm executed %d searches.\n', search_count_a_star);

% 使用Dijkstra演算法找到最短衛星路徑
[satellite_path_Dijkstra, total_satellite_distance_Dijkstra, Dijkstra_time, visited_nodes_dijk, search_count_dijk] = Dijkstra_algorithm(satellite_coords, start_satellite, end_satellite, total_satellites, max_dis_perhop);
fprintf('Dijkstra algorithm executed %d searches.\n', search_count_dijk);

% 使用Floyd-Warshall演算法找到最短衛星路徑
%[satellite_path_Floyd, total_satellite_distance_Floyd, Floyd_time, visited_nodes_floyd] = FloydWarshall_algorithm(satellite_coords, start_satellite, end_satellite, total_satellites, max_dis_perhop);
%fprintf('Floyd-Warshall algorithm executed.\n');

% 計算Dijkstra路徑總距離，包括地面站到最近衛星的距離
total_distance_Dijkstra = total_satellite_distance_Dijkstra + min_distance_gs_s + min_distance_gs_d;

% 輸出Dijkstra結果
fprintf('Dijkstra演算法花費時間: %.4f秒\n', Dijkstra_time);
fprintf('經過的衛星節點數（包含起始與終點）: %d\n', length(satellite_path_Dijkstra));
fprintf('總路徑距離: %.2f公里\n', total_distance_Dijkstra);

% 計算A*路徑總距離，包括地面站到最近衛星的距離
total_distance_A_Star = total_satellite_distance_A_star + min_distance_gs_s + min_distance_gs_d;

% 輸出A*結果
fprintf('A*演算法花費時間: %.4f秒\n', A_star_time);
fprintf('經過的衛星節點數（包含起始與終點）: %d\n', length(satellite_path_A_star));
fprintf('總路徑距離: %.2f公里\n', total_distance_A_Star);
RTT = (total_distance_A_Star/light_speed)*1000;
fprintf('OneWay Latency: %.2fms\n', RTT);
reduced_time_perc = ((Dijkstra_time - A_star_time)/Dijkstra_time)*100;
fprintf('A Star節省時間: %.2f%%\n', reduced_time_perc);


fprintf('Path 0: 地面站 紐約 到 衛星%d, 距離：%.2f公里\n', satellite_path_A_star(1), min_distance_gs_s);
for i = 1:length(satellite_path_A_star) - 1
    distance = distance_between(satellite_coords, satellite_path_A_star(i), satellite_path_A_star(i+1));
    fprintf('Path %d: 衛星%d 到 衛星%d, 距離：%.2f公里\n', i, satellite_path_A_star(i), satellite_path_A_star(i+1), distance);
end
fprintf('Path %d: 衛星%d 到 地面站 雪梨, 距離：%.2f公里\n', length(satellite_path_A_star), satellite_path_A_star(end), min_distance_gs_d);

% 繪製3D座標圖 - A* 路徑
figure;
scatter3(satellite_coords(:, 1), satellite_coords(:, 2), satellite_coords(:, 3), 'b.');
hold on;
% 繪製地球範圍
earth_radius = 6378;
[x, y, z] = sphere(50);
x = x * earth_radius;
y = y * earth_radius;
z = z * earth_radius;
surf(x, y, z, 'FaceColor', 'green', 'FaceAlpha', 0.1, 'EdgeColor', 'none');

% 繪製起點和終點
scatter3(gs_s_3D(1), gs_s_3D(2), gs_s_3D(3), 100, 'ro', 'filled');
scatter3(gs_d_3D(1), gs_d_3D(2), gs_d_3D(3), 100, 'go', 'filled');

% 標示起點和終點名稱
text(gs_s_3D(1), gs_s_3D(2), gs_s_3D(3), ' NewYork', 'FontSize', 16);
text(gs_d_3D(1), gs_d_3D(2), gs_d_3D(3), ' Sydney', 'FontSize', 16);

% 繪製路徑上的節點，用紅色表示
scatter3(satellite_coords(visited_nodes, 1), satellite_coords(visited_nodes, 2), satellite_coords(visited_nodes, 3), 'g');
% 繪製路徑
path_coords = [gs_s_3D; satellite_coords(satellite_path_A_star, :); gs_d_3D];
plot3(path_coords(:, 1), path_coords(:, 2), path_coords(:, 3), 'r', 'LineWidth', 2);
for i = 1:length(satellite_path_A_star) - 1
    distance = distance_between(satellite_coords, satellite_path_A_star(i), satellite_path_A_star(i+1));
    mid_point = (satellite_coords(satellite_path_A_star(i), :) + satellite_coords(satellite_path_A_star(i+1), :)) / 2;
    text(mid_point(1), mid_point(2), mid_point(3), sprintf(' %.2f公里', distance), 'FontSize', 12);
end
% 設定座標軸標籤
xlabel('X (km)');
ylabel('Y (km)');
zlabel('Z (km)');

% 啟用旋轉模式
rotate3d on;

% 繪製3D座標圖 - Dijkstra 路徑
figure;
scatter3(satellite_coords(:, 1), satellite_coords(:, 2), satellite_coords(:, 3), 'b.');
hold on;
% 繪製地球範圍
surf(x, y, z, 'FaceColor', 'green', 'FaceAlpha', 0.1, 'EdgeColor', 'none');

% 繪製起點和終點
scatter3(gs_s_3D(1), gs_s_3D(2), gs_s_3D(3), 100, 'ro', 'filled');
scatter3(gs_d_3D(1), gs_d_3D(2), gs_d_3D(3), 100, 'go', 'filled');

% 標示起點和終點名稱
text(gs_s_3D(1), gs_s_3D(2), gs_s_3D(3), ' NewYork', 'FontSize', 20);
text(gs_d_3D(1), gs_d_3D(2), gs_d_3D(3), ' Sydney', 'FontSize', 20);

% 繪製路徑上的節點，用粉色表示
scatter3(satellite_coords(visited_nodes_dijk, 1), satellite_coords(visited_nodes_dijk, 2), satellite_coords(visited_nodes_dijk, 3), 'p');
% 繪製路徑
path_coords = [gs_s_3D; satellite_coords(satellite_path_Dijkstra, :); gs_d_3D];
plot3(path_coords(:, 1), path_coords(:, 2), path_coords(:, 3), 'g', 'LineWidth', 2);
for i = 1:length(satellite_path_Dijkstra) - 1
    distance = distance_between(satellite_coords, satellite_path_Dijkstra(i), satellite_path_Dijkstra(i+1));
    mid_point = (satellite_coords(satellite_path_Dijkstra(i), :) + satellite_coords(satellite_path_Dijkstra(i+1), :)) / 2;
    text(mid_point(1), mid_point(2), mid_point(3), sprintf(' %.2f公里', distance), 'FontSize', 12);
end
% 設定座標軸標籤
xlabel('X (km)');
ylabel('Y (km)');
zlabel('Z (km)');

% 啟用旋轉模式
rotate3d on;




% 繪製3D座標圖 - Bellman-Ford 路徑
%figure;
%scatter3(satellite_coords(:, 1), satellite_coords(:, 2), satellite_coords(:, 3), 'b.');
%hold on;
% 繪製地球範圍
%earth_radius = 6378;
%[x, y, z] = sphere(50);
%x = x * earth_radius;
%y = y * earth_radius;
%z = z * earth_radius;
%surf(x, y, z, 'FaceColor', 'green', 'FaceAlpha', 0.1, 'EdgeColor', 'none');

% 繪製起點和終點
%scatter3(gs_s_3D(1), gs_s_3D(2), gs_s_3D(3), 100, 'ro', 'filled');
%scatter3(gs_d_3D(1), gs_d_3D(2), gs_d_3D(3), 100, 'go', 'filled');

% 標示起點和終點名稱
%text(gs_s_3D(1), gs_s_3D(2), gs_s_3D(3), ' NewYork', 'FontSize', 16);
%text(gs_d_3D(1), gs_d_3D(2), gs_d_3D(3), ' Sydney', 'FontSize', 16);

% 繪製路徑上的節點，用紅色表示
%scatter3(satellite_coords(visited_nodes, 1), satellite_coords(visited_nodes, 2), satellite_coords(visited_nodes, 3), 'r');
% 繪製路徑
%path_coords = [gs_s_3D; satellite_coords(satellite_path_Bellman, :); gs_d_3D];
%plot3(path_coords(:, 1), path_coords(:, 2), path_coords(:, 3), 'r', 'LineWidth', 2);
%for i = 1:length(satellite_path_Bellman) - 1
%    distance = distance_between(satellite_coords, satellite_path_Bellman(i), satellite_path_Bellman(i+1));
%    mid_point = (satellite_coords(satellite_path_Bellman(i), :) + satellite_coords(satellite_path_Bellman(i+1), :)) / 2;
%    text(mid_point(1), mid_point(2), mid_point(3), sprintf(' %.2f公里', distance), 'FontSize', 12);
%end
% 設定座標軸標籤
%xlabel('X (km)');
%ylabel('Y (km)');
%zlabel('Z (km)');

% 啟用旋轉模式
%rotate3d on;
