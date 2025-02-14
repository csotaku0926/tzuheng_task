clc
clear
% 參數設定
earth_radius = 6378;
orbit_altitude = 550;
total_satellites = 2000;
num_orbits = 50;
satellites_per_orbit = total_satellites / num_orbits;

% 創建 WGS84 參考橢球體
wgs84 = referenceEllipsoid('WGS84');

% 生成衛星座標
satellite_coords = zeros(total_satellites, 3);
for orbit = 1:num_orbits
    % 設定軌道傾角為0，即沿赤道
    inclination = 0;
    
    for satellite = 1:satellites_per_orbit
        index = (orbit - 1) * satellites_per_orbit + satellite;
        theta = 2 * pi / satellites_per_orbit * (satellite - 1);
        phi = (2 * pi / num_orbits) * (orbit - 1);
        
        satellite_coords(index, 1) = (earth_radius + orbit_altitude) * cosd(inclination) * cos(phi) * cos(theta) - (earth_radius + orbit_altitude) * sind(inclination) * sin(phi);
        satellite_coords(index, 2) = (earth_radius + orbit_altitude) * cosd(inclination) * cos(phi) * sin(theta) + (earth_radius + orbit_altitude) * sind(inclination) * cos(phi);
        satellite_coords(index, 3) = (earth_radius + orbit_altitude) * sind(inclination) * cos(phi) * cos(theta) + (earth_radius + orbit_altitude) * cosd(inclination) * sin(phi);
    end
end



% 定義地面站座標
New_York_coords = [40.7128, -74.0060];
Sydney_coords = [-33.8688, 151.2093];

% 轉換地面站座標為三維座標
New_York_3D = zeros(1, 3);
Sydney_3D = zeros(1, 3);

[New_York_3D(1), New_York_3D(2), New_York_3D(3)] = llh2xyz(New_York_coords(1), New_York_coords(2), 0, earth_radius);
[Sydney_3D(1), Sydney_3D(2), Sydney_3D(3)] = llh2xyz(Sydney_coords(1), Sydney_coords(2), 0, earth_radius);

% 找到距離紐約和雪梨最近的衛星
start_satellite = 0;
end_satellite = 0;
min_distance_New_York = inf;
min_distance_Sydney = inf;

for i = 1:total_satellites
    distance_New_York = norm(New_York_3D - satellite_coords(i, :));
    distance_Sydney = norm(Sydney_3D - satellite_coords(i, :));
    
    if distance_New_York < min_distance_New_York
        min_distance_New_York = distance_New_York;
        start_satellite = i;
    end
    
    if distance_Sydney < min_distance_Sydney
        min_distance_Sydney = distance_Sydney;
        end_satellite = i;
    end
end
% 將衛星的經緯度高程轉換為地球中心地球固定（ECEF）坐標
satellite_ecef_coords = zeros(total_satellites, 3);
for i = 1:total_satellites
    [x, y, z] = geodetic2ecef(wgs84, satellite_coords(i, 1), satellite_coords(i, 2), satellite_coords(i, 3));
    satellite_ecef_coords(i, :) = [x, y, z];
end

% 使用A*演算法找到最短衛星路徑
%[satellite_path, total_satellite_distance, A_star_time] = A_star_algorithm(satellite_coords, start_satellite, end_satellite);
[satellite_path, total_satellite_distance, A_star_time] = A_star_algorithm(satellite_coords, start_satellite, end_satellite, total_satellites);

% 計算路徑總距離，包括地面站到最近衛星的距離
total_distance = total_satellite_distance + min_distance_New_York + min_distance_Sydney;

% 輸出結果
fprintf('A*演算法花費時間: %.4f秒\n', A_star_time);
fprintf('經過的衛星節點數（不含起始與終點）: %d\n', length(satellite_path) - 2);
fprintf('總路徑距離: %.2f公里\n', total_distance);

for i = 1:length(satellite_path) - 1
    distance = distance_between(satellite_coords, satellite_path(i), satellite_path(i+1));
    fprintf('Hop %d: 衛星%d 到 衛星%d, 距離：%.2f公里\n', i, satellite_path(i), satellite_path(i+1), distance);
end
% 繪製3D座標圖
figure;
scatter3(satellite_coords(:, 1), satellite_coords(:, 2), satellite_coords(:, 3), 'b.');
hold on;
% 繪製地球範圍
earth_radius = 6378;
[x, y, z] = sphere(50);
x = x * earth_radius;
y = y * earth_radius;
z = z * earth_radius;
surf(x, y, z, 'FaceColor', 'blue', 'FaceAlpha', 0.1, 'EdgeColor', 'none');

% 繪製起點和終點
scatter3(New_York_3D(1), New_York_3D(2), New_York_3D(3), 100, 'ro', 'filled');
scatter3(Sydney_3D(1), Sydney_3D(2), Sydney_3D(3), 100, 'go', 'filled');

% 標示起點和終點名稱
text(New_York_3D(1), New_York_3D(2), New_York_3D(3), ' 紐約', 'FontSize', 12);
text(Sydney_3D(1), Sydney_3D(2), Sydney_3D(3), ' 雪梨', 'FontSize', 12);

% 繪製路徑
path_coords = [New_York_3D; satellite_coords(satellite_path, :); Sydney_3D];
plot3(path_coords(:, 1), path_coords(:, 2), path_coords(:, 3), 'r', 'LineWidth', 2);

% 設定座標軸標籤
xlabel('X (km)');
ylabel('Y (km)');
zlabel('Z (km)');

% 啟用旋轉模式
rotate3d on;

% 地理座標轉換為直角座標
function [x, y, z] = llh2xyz(lat, lon, alt, earth_radius)
    lat_rad = deg2rad(lat);
    lon_rad = deg2rad(lon);
    x = (earth_radius + alt) * cos(lat_rad) * cos(lon_rad);
    y = (earth_radius + alt) * cos(lat_rad) * sin(lon_rad);
    z = (earth_radius + alt) * sin(lat_rad);
end


% A*輔助函數
%function [path, total_distance, elapsed_time] = A_star_algorithm(satellite_coords, start_satellite, end_satellite)
function [path, total_distance, elapsed_time] = A_star_algorithm(satellite_coords, start_satellite, end_satellite, total_satellites)

    tic; % 開始計時
    open_list = [start_satellite];
    closed_list = [];
    
    g_cost = zeros(total_satellites, 1);
    h_cost = zeros(total_satellites, 1);
    f_cost = zeros(total_satellites, 1);
    came_from = zeros(total_satellites, 1);
    
    g_cost(start_satellite) = 0;
    h_cost(start_satellite) = heuristic_cost(satellite_coords, start_satellite, end_satellite);
    f_cost(start_satellite) = g_cost(start_satellite) + h_cost(start_satellite);
    
    while ~isempty(open_list)
        % 從open_list中找到f_cost最小的節點
        [~, min_index] = min(f_cost(open_list));
        current_satellite = open_list(min_index);
        
        if current_satellite == end_satellite
            path = reconstruct_path(came_from, current_satellite);
            total_distance = g_cost(current_satellite);
            elapsed_time = toc; % 結束計時
            return;
        end
        
        open_list(min_index) = [];
        closed_list = [closed_list; current_satellite];
        
        % 遍歷當前節點的鄰居節點
        neighbors = find_neighbors(satellite_coords, current_satellite);
        for i = 1:length(neighbors)
            neighbor = neighbors(i);
            if ismember(neighbor, closed_list)
                continue;
            end
            
            tentative_g_cost = g_cost(current_satellite) + distance_between(satellite_coords, current_satellite, neighbor);
            
            if ~ismember(neighbor, open_list)
                open_list = [open_list; neighbor];
            elseif tentative_g_cost >= g_cost(neighbor)
                continue;
            end
            
            came_from(neighbor) = current_satellite;
            g_cost(neighbor) = tentative_g_cost;
            h_cost(neighbor) = heuristic_cost(satellite_coords, neighbor, end_satellite);
            f_cost(neighbor) = g_cost(neighbor) + h_cost(neighbor);
        end
    end
    
    % 若未找到路徑
    path = [];
    total_distance = 0;
    elapsed_time = toc;
end

function cost = heuristic_cost(satellite_coords, current_satellite, end_satellite)
    cost = norm(satellite_coords(current_satellite, :) - satellite_coords(end_satellite, :));
end

function neighbors = find_neighbors(satellite_coords, current_satellite)
    neighbors = [];
    for i = 1:size(satellite_coords, 1)
        if i ~= current_satellite && distance_between(satellite_coords, current_satellite, i) <= 5016
            neighbors = [neighbors; i];
        end
    end
end

function distance = distance_between(satellite_coords, current_satellite, neighbor)
    distance = norm(satellite_coords(current_satellite, :) - satellite_coords(neighbor, :));
end

function path = reconstruct_path(came_from, current_satellite)
    total_path = [current_satellite];
    while came_from(current_satellite) > 0
        current_satellite = came_from(current_satellite);
        total_path = [current_satellite; total_path];
    end
    path = total_path;
end

