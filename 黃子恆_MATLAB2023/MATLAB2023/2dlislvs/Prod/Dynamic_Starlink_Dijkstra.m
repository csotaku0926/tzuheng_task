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
atm_high = 80;                  % 含水氣之大氣層海拔高度
light_speed = 299792.458;       % 光速
% 根據衛星高度與大氣層厚度計算最長hop距離
% max_dis_perhop = floor(sqrt((earth_radius + orbit_altitude)^2 - (earth_radius + atm_high)^2) * 2);

% 創建 WGS84 參考橢球體
wgs84 = referenceEllipsoid('WGS84');

% 定義地面站座標
gs_s_coords = [40.7128, -74.0060];  % 起始地面站
gs_d_coords = [-33.8688, 151.2093]; % 目標地面站

% 轉換地面站座標為三維座標
gs_s_3D = zeros(1, 3);
gs_d_3D = zeros(1, 3);

% 初始化當前衛星索引值
index = 1;

% 準備衛星資料
planes = [540 550 560 570 335.9 340.8 345.6]; % 各平面衛星高度 (km)
inclinations = [53.2 53 97.6 70 42 48 53]; % 各平面的軌道傾角
total_satellites = [1584 1584 520 720 2493 2478 2547]; % 各平面的總衛星數量
num_orbits = [72 72 10 36 9 42 9]; % 各平面的軌道數量

% 準備總衛星數量
total_satellites_sum = sum(total_satellites);
fprintf('衛星數量%d\n', total_satellites_sum);

% 初始化四維座標陣列 (x, y, z, height)
satellite_coords = zeros(total_satellites_sum, 4);

% 在迴圈開始前初始化矩陣
visited_nodes_dijk_list = [];
Dijkstra_hops_list = [];
RTT_dijk_list = [];

% 每5分鐘更新一次衛星位置
for minute = 0:5:5
    t = minute*60; %時間，單位是秒
    % 根據地求自轉計算地面站座標
    [gs_s_3D(1), gs_s_3D(2), gs_s_3D(3)] = calculate_gs_coordinates(earth_radius, gs_s_coords(1), gs_s_coords(2), t);
    [gs_d_3D(1), gs_d_3D(2), gs_d_3D(3)] = calculate_gs_coordinates(earth_radius, gs_d_coords(1), gs_d_coords(2), t);

    for plane = 1:7
        orbit_altitude = planes(plane);
        inclination = inclinations(plane);
        total_satellites_plane = total_satellites(plane);
        num_orbits_plane = num_orbits(plane);
        satellites_per_orbit = total_satellites_plane / num_orbits_plane; % 計算每條軌道的衛星數量
        semi_major_axis = earth_radius + orbit_altitude; % km
        semi_major_axis_m = semi_major_axis * 10^3; % m
        T = 2 * pi * sqrt((semi_major_axis_m^3) / (gravitational_parameter * 10^9)); % 秒

        for orbit = 1:num_orbits_plane
            RAAN = (orbit - 1) * 360 / num_orbits_plane;
            for satellite = 1:satellites_per_orbit
                initial_true_anomaly = (satellite - 1) * 360 / satellites_per_orbit;
                true_anomaly = initial_true_anomaly + (360/T)*(minute*60); % 衛星每分鐘在軌道上移動的角度為360/T
                true_anomaly = mod(true_anomaly, 360); % 確保真實異常在0到360度之間
                
                % 計算新的衛星座標
                [x, y, z] = calculate_satellite_coordinates(earth_radius, orbit_altitude, RAAN, inclination, true_anomaly);
                
                % 更新當前衛星的座標
                satellite_coords(index, :) = [x, y, z, orbit_altitude];
                index = index + 1;
            end
        end
    end
    index = 1;
    % 找到距離紐約和雪梨最近的衛星
    start_satellite = 0;
    end_satellite = 0;
    min_distance_gs_s = inf;
    min_distance_gs_d = inf;
    
    for i = 1:total_satellites_sum
        distance_gs_s = norm(gs_s_3D - satellite_coords(i, 1:3));
        distance_gs_d = norm(gs_d_3D - satellite_coords(i, 1:3));
        
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
    satellite_ecef_coords = zeros(total_satellites_sum, 3);
    for i = 1:total_satellites_sum
        [x, y, z] = geodetic2ecef(wgs84, satellite_coords(i, 1), satellite_coords(i, 2), satellite_coords(i, 3));
        satellite_ecef_coords(i, :) = [x, y, z];
    end
    % 使用Dijkstra演算法找到最短衛星路徑
    [satellite_path_Dijkstra, total_satellite_distance_Dijkstra, Dijkstra_time, visited_nodes_dijk, search_count_dijk] = Dijkstra_algorithm(satellite_coords, start_satellite, end_satellite, total_satellites_sum);

    fprintf('Dijkstra algorithm executed %d searches.\n', search_count_dijk);

    % 計算路徑總距離，包括地面站到最近衛星的距離
    total_distance_dijk = total_satellite_distance_Dijkstra + min_distance_gs_s + min_distance_gs_d;
    
    % 輸出結果
    fprintf('Dijkstra演算法花費時間: %.4f秒\n', Dijkstra_time);
    fprintf('Dijkstra經過的衛星節點數（包含起始與終點）: %d\n', length(satellite_path_Dijkstra));
    Dijkstra_hops_list = [Dijkstra_hops_list, length(satellite_path_Dijkstra)];
    fprintf('Dijkstra搜尋過的節點數: %d\n', length(visited_nodes_dijk));
    visited_nodes_dijk_list = [visited_nodes_dijk_list, length(visited_nodes_dijk)];
    fprintf('Dijkstra總路徑距離: %.2f公里\n', total_distance_dijk);
    RTT_dijk = (total_distance_dijk/light_speed)*1000+length(satellite_path_Dijkstra);
    RTT_dijk_list = [RTT_dijk_list, RTT_dijk];
    fprintf('Dijkstra OneWay Latency: %.2fms\n', RTT_dijk);

    %Dijkstra每段距離
    fprintf('Dijkstra Path 0: 地面站 紐約 到 衛星%d, 距離：%.2f公里,', satellite_path_Dijkstra(1), min_distance_gs_s);
    for i = 1:length(satellite_path_Dijkstra) - 1
        distance = distance_between(satellite_coords, satellite_path_Dijkstra(i), satellite_path_Dijkstra(i+1));
        fprintf('Path %d: 衛星%d 到 衛星%d, 距離：%.2f公里,', i, satellite_path_Dijkstra(i), satellite_path_Dijkstra(i+1), distance);
    end
    fprintf('Path %d: 衛星%d 到 地面站 雪梨, 距離：%.2f公里\n', length(satellite_path_Dijkstra), satellite_path_Dijkstra(end), min_distance_gs_d);

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
    path_coords = [gs_s_3D; satellite_coords(satellite_path_Dijkstra, 1:3); gs_d_3D];
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
    
end

%組合所有要展示的數據
RTT_all_data = [RTT_dijk_list'];
hops_all_data = [Dijkstra_hops_list'];
visited_all_data = [visited_nodes_dijk_list'];
%箱型圖 - Latency
figure
boxplot(RTT_all_data, 'Labels', {'Dijkstra'})
title('OneWay Latency Comparison (24h)')

figure
boxplot(hops_all_data, 'Labels', {'Dijkstra'})
title('Hops Comparison (24h)')

figure
boxplot(visited_all_data, 'Labels', {'Dijkstra'})
title('Visited nodes Comparison (24h)')

% 停止將命令窗口的輸出記錄到文件中
diary off  