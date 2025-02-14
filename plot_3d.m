
% % 繪製3D座標圖
% figure;
% scatter3(satellite_coords(:, 1), satellite_coords(:, 2), satellite_coords(:, 3), 'b.');
% hold on;
% % 繪製地球範圍
% earth_radius = 6378;
% [x, y, z] = sphere(50);
% x = x * earth_radius;
% y = y * earth_radius;
% z = z * earth_radius;
% surf(x, y, z, 'FaceColor', 'blue', 'FaceAlpha', 0.1, 'EdgeColor', 'none');

% % 繪製起點和終點
% scatter3(New_York_3D(1), New_York_3D(2), New_York_3D(3), 100, 'ro', 'filled');
% scatter3(Johannesburg_3D(1), Johannesburg_3D(2), Johannesburg_3D(3), 100, 'go', 'filled');

% % 標示起點和終點名稱
% text(New_York_3D(1), New_York_3D(2), New_York_3D(3), ' NewYork', 'FontSize', 16);
% text(Johannesburg_3D(1), Johannesburg_3D(2), Johannesburg_3D(3), ' Johannesburg', 'FontSize', 16);

% % 繪製路徑
% path_coords = [New_York_3D; satellite_coords(satellite_path, :); Johannesburg_3D];
% plot3(path_coords(:, 1), path_coords(:, 2), path_coords(:, 3), 'r', 'LineWidth', 2);
% for i = 1:length(satellite_path) - 1
%     distance = distance_between(satellite_coords, satellite_path(i), satellite_path(i+1));
%     mid_point = (satellite_coords(satellite_path(i), :) + satellite_coords(satellite_path(i+1), :)) / 2;
%     text(mid_point(1), mid_point(2), mid_point(3), sprintf(' %.2f公里', distance), 'FontSize', 8);
% end
% % 設定座標軸標籤
% xlabel('X (km)');
% ylabel('Y (km)');
% zlabel('Z (km)');

% % 啟用旋轉模式
% rotate3d on;



% plot latency
% lgds = {};
% for i = 1:length(n_sat_list)
%     lgds = [lgds, ""];
% end

% % Set legend names
% for i = 1:length(n_sat_list)
%     alts = orbit_altitude{i};
%     n_sats_i = n_sat_list{i};
%     n_orbit_i = num_orbits{i};
%     inc_i = inclinations{i};
%     gamma_i = gammas{i};
    
%     lgds(i) = out_of_batt_strs(i);
%     % lgds(i) = strcat(lgds(i), 'gamma = ', sci_notation(gamma_i));

%     % for j = 1:length(n_sats_i)
%     %     alt_txt_j = num2str(alts(j));
%     %     n_sat_txt_j = num2str(n_sats_i(j));
%     %     n_orbit_txt_j = num2str(n_orbit_i(j));
%     %     inc_txt_j = num2str(inc_i(j));
%     %     lgds(i) = strcat(lgds(i), alt_txt_j, ':', n_sat_txt_j, '(', n_orbit_txt_j, '/', inc_txt_j, '°)', {' '}); % preserve blank spaces
%     % end

% end