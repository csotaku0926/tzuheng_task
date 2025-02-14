function output_path(alg_name, total_distance, light_speed, ...
    satellite_coords, satellite_path, ...
    min_distance_New_York, min_distance_Johannesburg)

    RTT = (total_distance/light_speed)*1000;
    fprintf('%s path summary:\n', alg_name);
    fprintf('OneWay Latency: %.2fms\n', RTT);
    % save_time_perc = ((Dijkstra_time - A_star_time)/Dijkstra_time)*100;
    % fprintf('A Star節省時間: %.2f%%\n', save_time_perc);
    % fprintf('\n');

    fprintf('Path 0: 地面站 紐約 到 衛星%d, 距離：%.2f公里\n', satellite_path(1), min_distance_New_York);
    for i = 1:length(satellite_path) - 1
        distance = distance_between(satellite_coords, satellite_path(i), satellite_path(i+1));
        fprintf('Path %d: 衛星%d 到 衛星%d, 距離：%.2f公里\n', i, satellite_path(i), satellite_path(i+1), distance);
    end
    fprintf('Path %d: 衛星%d 到 地面站 雪梨, 距離：%.2f公里\n', length(satellite_path), satellite_path(end), min_distance_Johannesburg);
end
