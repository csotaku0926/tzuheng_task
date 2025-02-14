% 定義一些常數
close
clc
clear
% 開始將命令窗口的輸出記錄到 'output.txt' 文件中
diary('output.txt')
% 參數設定
earth_radius = 6378;            % 地球半徑(km)
orbit_altitude = 550;           % 衛星海拔(km)
gravitational_parameter = 398600.4418; % 地球的重力參數 (km³/s²)
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

% 定義地面站座標
gs_s_coords = [40.7128, -74.0060];  % 起始地面站
gs_d_coords = [-33.8688, 151.2093]; % 目標地面站

% 轉換地面站座標為三維座標
gs_s_3D = zeros(1, 3);
gs_d_3D = zeros(1, 3);

% 計算半長軸長度
semi_major_axis = earth_radius + orbit_altitude; % km

% 轉換半長軸長度為米
semi_major_axis_m = semi_major_axis * 10^3; % m

% 計算軌道週期
T = 2 * pi * sqrt((semi_major_axis_m^3) / (gravitational_parameter * 10^9)); % 秒

% Calculate satellite positions
satellites_per_orbit = total_satellites / num_orbits; % 計算每條軌道的衛星數量
satellite_coords = zeros(total_satellites, 3);        % 建立衛星三維座標陣列
index = 1;

% 在迴圈開始前初始化Dijkstra_lengths和A_star_lengths矩陣
visited_nodes_dijk_list = [];
A_visited_nodes_list = [];
BA_visited_nodes_list = [];
Dijkstra_hops_list = [];
A_star_hops_list = [];
BA_star_hops_list = [];
RTT_dijk_list = [];
RTT_a_list = [];
RTT_ba_list = [];

% 每5分鐘更新一次衛星位置
for minute = 0:5:5
    t = minute*60; %時間，單位是秒
    % 根據地求自轉計算地面站座標
    [gs_s_3D(1), gs_s_3D(2), gs_s_3D(3)] = calculate_gs_coordinates(earth_radius, gs_s_coords(1), gs_s_coords(2), t);
    [gs_d_3D(1), gs_d_3D(2), gs_d_3D(3)] = calculate_gs_coordinates(earth_radius, gs_d_coords(1), gs_d_coords(2), t);
    fprintf('Source X：%f, Y：%f, Z：%f\n', gs_s_3D(1), gs_s_3D(2), gs_s_3D(3))
    fprintf('Destination X：%f, Y：%f, Z：%f\n', gs_d_3D(1), gs_d_3D(2), gs_d_3D(3))

    % 更新衛星座標
    for orbit = 1:num_orbits
        RAAN = (orbit - 1) * 360 / num_orbits;
        for satellite = 1:satellites_per_orbit
            % 更新真實異常
            initial_true_anomaly = (satellite - 1) * 360 / satellites_per_orbit;
            true_anomaly = initial_true_anomaly + (360/T)*(minute*60); % 衛星每分鐘在軌道上移動的角度為360/T
            true_anomaly = mod(true_anomaly, 360); % 確保真實異常在0到360度之間
            
            % 計算新的衛星座標
            [x, y, z] = calculate_satellite_coordinates(earth_radius, orbit_altitude, RAAN, inclination, true_anomaly);
            
            % 計算當前衛星的索引
            index = (orbit - 1) * satellites_per_orbit + satellite;
            
            % 更新當前衛星的座標
            satellite_coords(index, :) = [x, y, z];
        end
    end

    % 在這裡加入想要每次做的操作，比如計算和繪製衛星軌道
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
    % 將衛星的經緯高度資訊轉換為地球中心地球固定（ECEF）坐標
    satellite_ecef_coords = zeros(total_satellites, 3);
    for i = 1:total_satellites
        [x, y, z] = geodetic2ecef(wgs84, satellite_coords(i, 1), satellite_coords(i, 2), satellite_coords(i, 3));
        satellite_ecef_coords(i, :) = [x, y, z];
    end
    % 使用Dijkstra演算法找到最短衛星路徑
    [satellite_path_Dijkstra, total_satellite_distance_Dijkstra, Dijkstra_time, visited_nodes_dijk, search_count_dijk] = Dijkstra_algorithm(satellite_coords, start_satellite, end_satellite, total_satellites, max_dis_perhop);
    % 使用A*演算法找到最短衛星路徑
    [satellite_path_A_star, total_satellite_distance_A_star, A_star_time, A_visited_nodes, search_count_a_star] = A_star_algorithm(satellite_coords, start_satellite, end_satellite, total_satellites, max_dis_perhop);
    % 使用雙向A*演算法找到最短衛星路徑
    [satellite_path_BA_star, total_satellite_distance_BA_star, BA_star_time, BA_visited_nodes, search_count_ba_star, segment_distances] = A_star_bidirectional_algorithm2(satellite_coords, start_satellite, end_satellite, total_satellites, max_dis_perhop);
    %learning_rate = 0.000001;
    %[satellite_path_HA_star, total_satellite_distance_HA_star, HA_star_time, HA_visited_nodes, search_count_ha_star] = A_star_adaptive_algorithm(satellite_coords, start_satellite, end_satellite, total_satellites, max_dis_perhop, learning_rate);

    fprintf('Dijkstra algorithm executed %d searches.\n', search_count_dijk);
    fprintf('A* algorithm executed %d searches.\n', search_count_a_star);
    fprintf('BA* algorithm executed %d searches.\n', search_count_ba_star);

    % 計算路徑總距離，包括地面站到最近衛星的距離
    total_distance_dijk = total_satellite_distance_Dijkstra + min_distance_gs_s + min_distance_gs_d;
    total_distance_A_Star = total_satellite_distance_A_star + min_distance_gs_s + min_distance_gs_d;
    total_distance_BA_Star = total_satellite_distance_BA_star + min_distance_gs_s + min_distance_gs_d;
    
    % 輸出結果
    fprintf('Dijkstra演算法花費時間: %.4f秒\n', Dijkstra_time);
    fprintf('A*演算法花費時間: %.4f秒\n', A_star_time);
    fprintf('BA*演算法花費時間: %.4f秒\n', BA_star_time);
    fprintf('Dijkstra經過的衛星節點數（包含起始與終點）: %d\n', length(satellite_path_Dijkstra));
    fprintf('A*經過的衛星節點數（包含起始與終點）: %d\n', length(satellite_path_A_star));
    fprintf('BA*經過的衛星節點數（包含起始與終點）: %d\n', length(satellite_path_BA_star));
    Dijkstra_hops_list = [Dijkstra_hops_list, length(satellite_path_Dijkstra)];
    A_star_hops_list = [A_star_hops_list, length(satellite_path_A_star)];
    BA_star_hops_list = [BA_star_hops_list, length(satellite_path_BA_star)];
    fprintf('Dijkstra搜尋過的節點數: %d\n', length(visited_nodes_dijk));
    fprintf('A*搜尋過的節點數: %d\n', length(A_visited_nodes));
    fprintf('BA*搜尋過的節點數: %d\n', length(BA_visited_nodes));
    visited_nodes_dijk_list = [visited_nodes_dijk_list, length(visited_nodes_dijk)];
    A_visited_nodes_list = [A_visited_nodes_list, length(A_visited_nodes)];
    BA_visited_nodes_list = [BA_visited_nodes_list, length(BA_visited_nodes)];
    fprintf('Dijkstra總路徑距離: %.2f公里\n', total_distance_dijk);
    fprintf('A*總路徑距離: %.2f公里\n', total_distance_A_Star);
    fprintf('BA*總路徑距離: %.2f公里\n', total_distance_BA_Star);
    RTT_dijk = (total_distance_dijk/light_speed)*1000+length(satellite_path_Dijkstra);
    RTT_a = (total_distance_A_Star/light_speed)*1000+length(satellite_path_A_star);
    RTT_ba = (total_distance_BA_Star/light_speed)*1000+length(satellite_path_BA_star);
    RTT_dijk_list = [RTT_dijk_list, RTT_dijk];
    RTT_a_list = [RTT_a_list, RTT_a];
    RTT_ba_list = [RTT_ba_list, RTT_ba];
    fprintf('Dijkstra OneWay Latency: %.2fms\n', RTT_dijk);
    fprintf('A* OneWay Latency: %.2fms\n', RTT_a);
    fprintf('BA* OneWay Latency: %.2fms\n', RTT_ba);
%{
    %Dijkstra每段距離
    fprintf('Dijkstra Path 0: 地面站 紐約 到 衛星%d, 距離：%.2f公里,', satellite_path_Dijkstra(1), min_distance_gs_s);
    for i = 1:length(satellite_path_Dijkstra) - 1
        distance = distance_between(satellite_coords, satellite_path_Dijkstra(i), satellite_path_Dijkstra(i+1));
        fprintf('Path %d: 衛星%d 到 衛星%d, 距離：%.2f公里,', i, satellite_path_Dijkstra(i), satellite_path_Dijkstra(i+1), distance);
    end
    fprintf('Path %d: 衛星%d 到 地面站 雪梨, 距離：%.2f公里\n', length(satellite_path_Dijkstra), satellite_path_Dijkstra(end), min_distance_gs_d);
    %A*每段距離
    fprintf('A* Path 0: 地面站 紐約 到 衛星%d, 距離：%.2f公里,', satellite_path_A_star(1), min_distance_gs_s);
    for i = 1:length(satellite_path_A_star) - 1
        distance = distance_between(satellite_coords, satellite_path_A_star(i), satellite_path_A_star(i+1));
        fprintf('Path %d: 衛星%d 到 衛星%d, 距離：%.2f公里,', i, satellite_path_A_star(i), satellite_path_A_star(i+1), distance);
    end
    fprintf('Path %d: 衛星%d 到 地面站 雪梨, 距離：%.2f公里\n', length(satellite_path_A_star), satellite_path_A_star(end), min_distance_gs_d);
    %BA*每段距離
    fprintf('BA* Path 0: 地面站 紐約 到 衛星%d, 距離：%.2f公里,', satellite_path_BA_star(1), min_distance_gs_s);
    for i = 1:length(satellite_path_BA_star) - 1
        distance = distance_between(satellite_coords, satellite_path_BA_star(i), satellite_path_BA_star(i+1));
        fprintf('Path %d: 衛星%d 到 衛星%d, 距離：%.2f公里,', i, satellite_path_BA_star(i), satellite_path_BA_star(i+1), distance);
    end
    fprintf('Path %d: 衛星%d 到 地面站 雪梨, 距離：%.2f公里\n', length(satellite_path_BA_star), satellite_path_BA_star(end), min_distance_gs_d);


    % 繪製3D座標圖 - Dijkstra 路徑
    
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
    
    % 繪製Dijkstra路徑上的節點，用紅色表示
    scatter3(satellite_coords(visited_nodes_dijk, 1), satellite_coords(visited_nodes_dijk, 2), satellite_coords(visited_nodes_dijk, 3), 'g');
    % 繪製路徑
    path_coords = [gs_s_3D; satellite_coords(satellite_path_Dijkstra, :); gs_d_3D];
    plot3(path_coords(:, 1), path_coords(:, 2), path_coords(:, 3), 'r', 'LineWidth', 2);
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
    
    % 繪製A*路徑上的節點，用紅色表示
    scatter3(satellite_coords(A_visited_nodes, 1), satellite_coords(A_visited_nodes, 2), satellite_coords(A_visited_nodes, 3), 'g');
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

    % 繪製3D座標圖 - BA* 路徑
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
    
    % 繪製BA*路徑上的節點，用紅色表示
    scatter3(satellite_coords(BA_visited_nodes, 1), satellite_coords(BA_visited_nodes, 2), satellite_coords(BA_visited_nodes, 3), 'g');
    % 繪製路徑
    path_coords = [gs_s_3D; satellite_coords(satellite_path_BA_star, :); gs_d_3D];
    plot3(path_coords(:, 1), path_coords(:, 2), path_coords(:, 3), 'r', 'LineWidth', 2);
    for i = 1:length(satellite_path_BA_star) - 1
        distance = distance_between(satellite_coords, satellite_path_BA_star(i), satellite_path_BA_star(i+1));
        mid_point = (satellite_coords(satellite_path_BA_star(i), :) + satellite_coords(satellite_path_BA_star(i+1), :)) / 2;
        text(mid_point(1), mid_point(2), mid_point(3), sprintf(' %.2f公里', distance), 'FontSize', 12);
    end
    % 設定座標軸標籤
    xlabel('X (km)');
    ylabel('Y (km)');
    zlabel('Z (km)');
    
    % 啟用旋轉模式
    rotate3d on;
%}
end

%組合所有要展示的數據
RTT_all_data = [RTT_dijk_list', RTT_a_list', RTT_ba_list'];
hops_all_data = [Dijkstra_hops_list', A_star_hops_list', BA_star_hops_list'];
visited_all_data = [visited_nodes_dijk_list', A_visited_nodes_list', BA_visited_nodes_list'];
%箱型圖 - Latency
figure
boxplot(RTT_all_data, 'Labels', {'Dijkstra', 'A*', 'BA*'})
title('OneWay Latency Comparison (24h)')

figure
boxplot(hops_all_data, 'Labels', {'Dijkstra', 'A*', 'BA*'})
title('Hops Comparison (24h)')

figure
boxplot(visited_all_data, 'Labels', {'Dijkstra', 'A*', 'BA*'})
title('Visited nodes Comparison (24h)')

% 停止將命令窗口的輸出記錄到文件中
diary off  