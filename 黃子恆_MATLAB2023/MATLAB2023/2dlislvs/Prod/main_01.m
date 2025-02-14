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
gs_s_coords = [40.7128, -74.0060];  % 起始地面站起點
%gs_d_coords = [-33.8688, 151.2093]; % 目標地面站終點
gs_d_coords = [-26.2041, 28.0473]; % 目標地面站約翰尼斯堡
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
search_count_dijk_list = [];
search_time_dijk_list = [];
Dijkstra_hops_list = [];
RTT_dijk_list = [];
total_distance_dijk_list = [];

A_visited_nodes_list = [];
search_count_A_star_list = [];
search_time_A_star_list = [];
A_star_hops_list = [];
RTT_a_list = [];
total_distance_a_list = [];

BA_visited_nodes_list = [];
search_count_BA_star_list = [];
search_time_BA_star_list = [];
BA_star_hops_list = [];
RTT_ba_list = [];
total_distance_ba_list = [];

DBA_visited_nodes_list = [];
search_count_DBA_star_list = [];
search_time_DBA_star_list = [];
DBA_star_hops_list = [];
RTT_dba_list = [];
total_distance_dba_list = [];
%{
DBA2_visited_nodes_list = [];
search_count_DBA2_star_list = [];
search_time_DBA2_star_list = [];
DBA2_star_hops_list = [];
RTT_dba2_list = [];
total_distance_dba2_list = [];

DBA3_visited_nodes_list = [];
search_count_DBA3_star_list = [];
search_time_DBA3_star_list = [];
DBA3_star_hops_list = [];
RTT_dba3_list = [];
total_distance_dba3_list = [];
%}
different_count = 0;
% 每5分鐘更新一次衛星位置
for minute = 1:1:1
    t = minute*60; %時間，單位是秒
    % 根據地求自轉計算地面站座標
    [gs_s_3D(1), gs_s_3D(2), gs_s_3D(3)] = calculate_gs_coordinates(earth_radius, gs_s_coords(1), gs_s_coords(2), t);
    [gs_d_3D(1), gs_d_3D(2), gs_d_3D(3)] = calculate_gs_coordinates(earth_radius, gs_d_coords(1), gs_d_coords(2), t);

    for plane = 1:2

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
    % 找到距離起點和終點最近的衛星
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
    % 使用A*演算法找到最短衛星路徑
    [satellite_path_A_star, total_satellite_distance_A_star, A_star_time, A_visited_nodes, search_count_a_star] = A_star_uv(satellite_coords, start_satellite, end_satellite, total_satellites_sum);
    fprintf('A* algorithm executed %d searches.\n', search_count_a_star);
    search_count_A_star_list = [search_count_A_star_list, search_count_a_star];
    % 使用雙向A*演算法找到最短衛星路徑
    [satellite_path_BA_star, total_satellite_distance_BA_star, BA_star_time, BA_visited_nodes, search_count_ba_star] = BA_DBS_uv_3_2(satellite_coords, start_satellite, end_satellite, total_satellites_sum);
    fprintf('BA* algorithm executed %d searches.\n', search_count_ba_star);
    search_count_BA_star_list = [search_count_BA_star_list, search_count_ba_star];
    % 比較兩種方法產生的路徑是否相同，若不同則記錄
    if ~isequal(satellite_path_A_star, satellite_path_BA_star)
        different_count = different_count + 1;
    end
    % 使用DBS演算法找到最短衛星路徑
    [satellite_path_DBA_star, total_satellite_distance_DBA_star, DBA_star_time, DBA_visited_nodes, search_count_dba_star] = A_star_bidirectional_algorithm2(satellite_coords, start_satellite, end_satellite, total_satellites_sum);
    fprintf('DBS algorithm executed %d searches.\n', search_count_dba_star);
    search_count_DBA_star_list = [search_count_DBA_star_list, search_count_dba_star];

    % 計算路徑總距離，包括地面站到最近衛星的距離
    total_distance_A_Star = total_satellite_distance_A_star + min_distance_gs_s + min_distance_gs_d;
    total_distance_a_list = [total_distance_a_list,total_distance_A_Star];
    total_distance_BA_Star = total_satellite_distance_BA_star + min_distance_gs_s + min_distance_gs_d;
    total_distance_ba_list = [total_distance_ba_list,total_distance_BA_Star];
    total_distance_DBA_Star = total_satellite_distance_DBA_star + min_distance_gs_s + min_distance_gs_d;
    total_distance_dba_list = [total_distance_dba_list,total_distance_DBA_Star];

    % 輸出結果 - A Star
    fprintf('A*演算法花費時間: %.4f秒\n', A_star_time);
    search_time_A_star_list = [search_time_A_star_list, A_star_time];
    fprintf('A*經過的衛星節點數（包含起始與終點）: %d\n', length(satellite_path_A_star));
    A_star_hops_list = [A_star_hops_list, length(satellite_path_A_star)];
    fprintf('A*搜尋過的節點數: %d\n', length(A_visited_nodes));
    A_visited_nodes_list = [A_visited_nodes_list, length(A_visited_nodes)];
    fprintf('A*總路徑距離: %.2fkm\n', total_distance_A_Star);
    RTT_a = (total_distance_A_Star/light_speed)*1000;
    RTT_a_list = [RTT_a_list, RTT_a];
    fprintf('A* OneWay Latency: %.2fms\n', RTT_a);

    % 輸出結果 - BA Star
    fprintf('BA*演算法花費時間: %.4f秒\n', BA_star_time);
    search_time_BA_star_list = [search_time_BA_star_list, BA_star_time];
    fprintf('BA*經過的衛星節點數（包含起始與終點）: %d\n', length(satellite_path_BA_star));
    BA_star_hops_list = [BA_star_hops_list, length(satellite_path_BA_star)];
    fprintf('BA*搜尋過的節點數: %d\n', length(BA_visited_nodes));
    BA_visited_nodes_list = [BA_visited_nodes_list, length(BA_visited_nodes)];
    fprintf('BA*總路徑距離: %.2fkm\n', total_distance_BA_Star);
    RTT_ba = (total_distance_BA_Star/light_speed)*1000;
    RTT_ba_list = [RTT_ba_list, RTT_ba];
    fprintf('BA* OneWay Latency: %.2fms\n', RTT_ba);
    % 輸出結果 - DBA Star
    fprintf('DBS演算法花費時間: %.4f秒\n', DBA_star_time);
    search_time_DBA_star_list = [search_time_DBA_star_list, DBA_star_time];
    fprintf('DBS經過的衛星節點數（包含起始與終點）: %d\n', length(satellite_path_DBA_star));
    DBA_star_hops_list = [DBA_star_hops_list, length(satellite_path_DBA_star)];
    fprintf('DBS搜尋過的節點數: %d\n', length(DBA_visited_nodes));
    DBA_visited_nodes_list = [DBA_visited_nodes_list, length(DBA_visited_nodes)];
    fprintf('DBS總路徑距離: %.2fkm\n', total_distance_DBA_Star);
    RTT_dba = (total_distance_DBA_Star/light_speed)*1000;
    RTT_dba_list = [RTT_dba_list, RTT_dba];
    fprintf('DBS OneWay Latency: %.2fms\n', RTT_dba);
    
    disp(satellite_path_A_star);
    disp(satellite_path_BA_star);
    disp(satellite_path_DBA_star);
    %A*每段距離
    fprintf('A* Path 0: 地面站 起點 到 衛星%d, 距離：%.2fkm,', satellite_path_A_star(1), min_distance_gs_s);
    for i = 1:length(satellite_path_A_star) - 1
        distance = distance_between(satellite_coords, satellite_path_A_star(i), satellite_path_A_star(i+1));
        fprintf('Path %d: 衛星%d 到 衛星%d, 距離：%.2fkm,', i, satellite_path_A_star(i), satellite_path_A_star(i+1), distance);
    end
    fprintf('Path %d: 衛星%d 到 地面站 終點, 距離：%.2fkm\n', length(satellite_path_A_star), satellite_path_A_star(end), min_distance_gs_d);

    %BA*每段距離
    fprintf('BA* Path 0: 地面站 起點 到 衛星%d, 距離：%.2fkm,', satellite_path_BA_star(1), min_distance_gs_s);
    for i = 1:length(satellite_path_BA_star) - 1
        distance = distance_between(satellite_coords, satellite_path_BA_star(i), satellite_path_BA_star(i+1));
        fprintf('Path %d: 衛星%d 到 衛星%d, 距離：%.2fkm,', i, satellite_path_BA_star(i), satellite_path_BA_star(i+1), distance);
    end
    fprintf('Path %d: 衛星%d 到 地面站 終點, 距離：%.2fkm\n', length(satellite_path_BA_star), satellite_path_BA_star(end), min_distance_gs_d);

    %DBS每段距離
    fprintf('DBS Path 0: 地面站 起點 到 衛星%d, 距離：%.2fkm,', satellite_path_DBA_star(1), min_distance_gs_s);
    for i = 1:length(satellite_path_DBA_star) - 1
        distance = distance_between(satellite_coords, satellite_path_DBA_star(i), satellite_path_DBA_star(i+1));
        fprintf('Path %d: 衛星%d 到 衛星%d, 距離：%.2fkm,', i, satellite_path_DBA_star(i), satellite_path_DBA_star(i+1), distance);
    end
    fprintf('Path %d: 衛星%d 到 地面站 終點, 距離：%.2fkm\n', length(satellite_path_DBA_star), satellite_path_DBA_star(end), min_distance_gs_d);

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
    text(gs_s_3D(1), gs_s_3D(2), gs_s_3D(3), ' Source', 'FontSize', 16);
    text(gs_d_3D(1), gs_d_3D(2), gs_d_3D(3), ' Destination', 'FontSize', 16);
    
    % 繪製A*路徑上的節點，用紅色表示
    scatter3(satellite_coords(A_visited_nodes, 1), satellite_coords(A_visited_nodes, 2), satellite_coords(A_visited_nodes, 3), 'r');
    % 繪製路徑
    path_coords = [gs_s_3D; satellite_coords(satellite_path_A_star, 1:3); gs_d_3D];
    plot3(path_coords(:, 1), path_coords(:, 2), path_coords(:, 3), 'r', 'LineWidth', 2);
    for i = 1:length(satellite_path_A_star) - 1
        distance = distance_between(satellite_coords, satellite_path_A_star(i), satellite_path_A_star(i+1));
        mid_point = (satellite_coords(satellite_path_A_star(i), :) + satellite_coords(satellite_path_A_star(i+1), :)) / 2;
        text(mid_point(1), mid_point(2), mid_point(3), sprintf(' %.2fkm', distance), 'FontSize', 12);
    end
    % 設定座標軸標籤
    xlabel('X (km)');
    ylabel('Y (km)');
    zlabel('Z (km)');
    
    % 啟用旋轉模式
    rotate3d on;

    % 繪製3D座標圖 - DBS 路徑
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
    scatter3(satellite_coords(BA_visited_nodes, 1), satellite_coords(BA_visited_nodes, 2), satellite_coords(BA_visited_nodes, 3), 'r');
    % 繪製路徑
    path_coords = [gs_s_3D; satellite_coords(satellite_path_BA_star, 1:3); gs_d_3D];
    plot3(path_coords(:, 1), path_coords(:, 2), path_coords(:, 3), 'r', 'LineWidth', 2);
    for i = 1:length(satellite_path_BA_star) - 1
        distance = distance_between(satellite_coords, satellite_path_BA_star(i), satellite_path_BA_star(i+1));
        mid_point = (satellite_coords(satellite_path_BA_star(i), :) + satellite_coords(satellite_path_BA_star(i+1), :)) / 2;
        text(mid_point(1), mid_point(2), mid_point(3), sprintf(' %.2fkm', distance), 'FontSize', 12);
    end
    % 設定座標軸標籤
    xlabel('X (km)');
    ylabel('Y (km)');
    zlabel('Z (km)');
    
    % 啟用旋轉模式
    rotate3d on;

    % 繪製3D座標圖 - DBS Re 路徑
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
    
    % 繪製BAD路徑上的節點，用紅色表示
    scatter3(satellite_coords(DBA_visited_nodes, 1), satellite_coords(DBA_visited_nodes, 2), satellite_coords(DBA_visited_nodes, 3), 'r');
    % 繪製路徑
    path_coords = [gs_s_3D; satellite_coords(satellite_path_DBA_star, 1:3); gs_d_3D];
    plot3(path_coords(:, 1), path_coords(:, 2), path_coords(:, 3), 'r', 'LineWidth', 2);
    for i = 1:length(satellite_path_DBA_star) - 1
        distance = distance_between(satellite_coords, satellite_path_DBA_star(i), satellite_path_DBA_star(i+1));
        mid_point = (satellite_coords(satellite_path_DBA_star(i), :) + satellite_coords(satellite_path_DBA_star(i+1), :)) / 2;
        text(mid_point(1), mid_point(2), mid_point(3), sprintf(' %.2fkm', distance), 'FontSize', 12);
    end
    % 設定座標軸標籤
    xlabel('X (km)');
    ylabel('Y (km)');
    zlabel('Z (km)');
    
    % 啟用旋轉模式
    rotate3d on;

end

%

fprintf('A* vs BA*:%.2f\n', different_count);
%組合所有要展示的數據
RTT_all_data = [RTT_dijk_list', RTT_a_list', RTT_ba_list', RTT_dba_list'];
hops_all_data = [Dijkstra_hops_list', BA_star_hops_list', A_star_hops_list', DBA_star_hops_list'];
visited_all_data = [visited_nodes_dijk_list', A_visited_nodes_list', BA_visited_nodes_list', DBA_visited_nodes_list'];
search_count_all_data = [search_count_dijk_list', search_count_A_star_list', search_count_BA_star_list', search_count_DBA_star_list'];
search_time_all_data = [search_time_dijk_list', search_time_A_star_list', search_time_BA_star_list', search_time_DBA_star_list'];
total_distance_all_data = [total_distance_dijk_list', total_distance_a_list', total_distance_ba_list', total_distance_dba_list'];

%計算平均值
RTT_all_data_mean = mean(RTT_all_data);
hops_all_data_mean = mean(hops_all_data);
visited_all_data_mean = mean(visited_all_data);
search_count_all_data_mean = mean(search_count_all_data);
search_time_all_data_mean = mean(search_time_all_data);
total_distance_all_data_mean = mean(total_distance_all_data);

%顯示平均值
fprintf('OneWay Latency Average (ms): A*: %.2f, BA*: %.2f, DBS: %.2f\n', RTT_all_data_mean);
fprintf('Hops Average: A*: %.1f, BA*: %.1f, DBS: %.1f\n', hops_all_data_mean);
fprintf('Visited nodes Average: A*: %.1f, BA*: %.1f, DBS: %.1f\n', visited_all_data_mean);
fprintf('Search count Average: A*: %.1f, BA*: %.1f, DBS: %.1f\n', search_count_all_data_mean);
fprintf('Search time Average: A*: %.2f, BA*: %.2f, DBS: %.2f\n', search_time_all_data_mean);
fprintf('Total distance Average (km): A*: %.2f, BA*: %.2f, DBS: %.2f\n', total_distance_all_data_mean);

%{
%箱型圖 - Latency
figure
boxplot(RTT_all_data, 'Labels', {'Dijkstra', 'A*', 'DBS', 'DBS(Reconstruct)'})
title('OneWay Latency Comparison (ms)')

figure
boxplot(hops_all_data, 'Labels', {'Dijkstra', 'A*', 'DBS', 'DBS(Reconstruct)'})
title('Hops Comparison')

figure
boxplot(visited_all_data, 'Labels', {'Dijkstra', 'A*', 'DBS', 'DBS(Reconstruct)'})
title('Visited nodes Comparison')

figure
boxplot(search_count_all_data, 'Labels', {'Dijkstra', 'A*', 'DBS', 'DBS(Reconstruct)'})
title('Search count Comparison')

figure
boxplot(total_distance_all_data, 'Labels', {'Dijkstra', 'A*', 'DBS', 'DBS(Reconstruct)'})
title('Total distance Comparison (km)')
%}
% 停止將命令窗口的輸出記錄到文件中
diary off  