clc
clear
addpath('routing\')
addpath('utils\')
addpath('DBS\')
addpath('cost\')

% 參數設定
earth_radius = 6378;
atm_high = 80;
light_speed = 299792.458;
batt_ratio = [0.01, 0.05, 0.1]; is_ratio_not_number = true;
fixed_batt_ratio = 0.0;
is_test_rc = false;
lgds = {};
dsts = {};
title = 'One-way average latency (ms)';

% 定義地面站座標
New_York_coords = [40.7128, -74.0060]; 
Johannesburg_coords = [-26.195246, 28.034088]; % dist = 122

%% Constellation Scale & DBS_FS advantage
% title = 'Average Elapsed Time (s)'; % for DBS_FS advantage
% n_layers = 10;
% % orbit_altitude = {[500], [500, 1000], [500, 1000, 1500], [500, 1000, 1500, 2000]};
% min_alt = 500; max_alt = 3000;
% dalt = round((max_alt - min_alt) / n_layers);
% alts = min_alt:dalt:max_alt;
% orbit_altitude = {}; 
% % n_sat_list = {[4400], [4400, 2000], [4400, 2000, 2000], [4400, 2000, 2000, 2000]}; % numbers of satellies on each layer (define how many layers) 
% n_sat = 2000; n_sats = [];
% n_sat_list = {};
% % num_orbits = {[100], [100, 80], [100, 80, 80], [100, 80, 80, 80]}; 
% n_orbit = 200; n_orbits = [];
% num_orbits = {};
% % inclinations = {[0], [0, 30], [0, 30, 60], [0, 30, 60, 90]}; 
% inc = 60; incs = [];
% inclinations = {};
% gammas = []; gamma = 0;

% for i = 1:n_layers
%     n_sats(i) = n_sat;
%     n_orbits(i) = n_orbit;
%     incs(i) = inc;
    
%     orbit_altitude{i} = alts(1:i);
%     n_sat_list{i} = n_sats;
%     num_orbits{i} = n_orbits;
%     inclinations{i} = incs;
%     gammas(i) = gamma;
% end

% saved_prefix = 'saves/nlayers_';
% x_plot_axis = 1:1:n_layers;
% x_tick_lbl = string(x_plot_axis);
% xlbl = "number of layers";
% ylbl = "latency";
% what_to_plot = 'latency'; % either 'latency', 'elapsed_time', 'update_count' or 'n_visited'


%% number of layers
alts = 500:250:2750;
orbit_altitude = {}; 
n_sats = [1500, 1500, 1500, 2400, 750, 2500, 4500, 2400, 2700, 2460];
n_sat_list = {};
n_orbits = [75, 75, 75, 20, 25, 50, 25, 75, 30, 20];
num_orbits = {};
incs = [30, 40, 40, 45, 95.5, 70, 40, 48, 72, 120];
inclinations = {};
gammas = [0, 10^2, 10^3, int32(10^3.5), 10^4, int32(10^4.5), 10^5, 10^6];
n_layers = length(alts);
n_runs = n_layers;

% set up other parameters
low_layer = 1;
max_layer = 4;
for i = 1:length(gammas)
    orbit_altitude{i} = alts(low_layer:max_layer);
    n_sat_list{i} = n_sats(low_layer:max_layer);
    num_orbits{i} = n_orbits(low_layer:max_layer);
    inclinations{i} = incs(low_layer:max_layer);
end


% structureness
% indices = {[6,7,8,9]};

% for i = 1:length(indices)
%     orbit_altitude{i} = alts(indices{i});
%     n_sat_list{i} = n_sats(indices{i});
%     num_orbits{i} = n_orbits(indices{i});
%     inclinations{i} = incs(indices{i});
%     gammas(i) = gamma;
% end

xlbl = "gamma";
ylbl = "n visited";
x_plot_axis = 1:length(gammas);
x_tick_lbl = string(gammas);
saved_prefix = 'saves/gamma_FS_';
what_to_plot = 'n_visited'; % either 'latency', 'elapsed_time', 'update_count' or 'n_visited'


%% section 2
% common settings
% gamma = 1e5; gammas = [];
% alts = [500, 900, 1100]; orbit_altitude = {}; 
% n_sats = [2000, 2000, 2000]; n_sat_list = {};
% n_orbits = [400, 400, 400]; num_orbits = {};
% incs = [30, 30, 30]; inclinations = {};

% == gamma settings ==
% gammas = [1e3, 1e4, 1e5, 1e6, 1e7];
% n_runs = length(gammas);
% fixed_batt_ratio = 0.1;

% == distance btwn src-dst == 
% Paris_coords = [48.864716, 2.349014]; % 74
% Tokyo_coords = [35.652832, 139.839478]; % 213
% Sydney_coords = [-33.8688, 151.2093]; % 237
% dsts = {Paris_coords, Johannesburg_coords, Tokyo_coords, Sydney_coords};
% n_runs = length(dsts);
% fixed_batt_ratio = 0.1;

% % set up param
% for i = 1:n_runs
%     orbit_altitude{i} = alts;
%     n_sat_list{i} = n_sats;
%     num_orbits{i} = n_orbits;
%     inclinations{i} = incs;
%     gammas(i) = gamma; % comment this line when gamma settings
% end

% title = 'Average Search Space';
% xlbl = "destination";
% ylbl = "updating cost count";
% x_plot_axis = 1:1:length(dsts);
% x_tick_lbl = ["Paris", "Johannesburg", "Tokyo", "Sydney"];
% saved_prefix = 'saves/dst_';
% % search space may be reflected in terms of times of updating costs or iteration count
% what_to_plot = 'update_count'; % either 'latency', 'elapsed_time', 'update_count' or 'n_visited'


%% section 3: battery drained nodes -- RC
% alts = [500, 1000, 1500, 2000]; orbit_altitude = {}; 
% n_sats = [1000, 1000, 1000, 1000]; n_sat_list = {};
% n_orbits = [100, 100, 100, 100]; num_orbits = {};
% incs = [60, 60, 60, 60]; inclinations = {};
% n_layers = length(alts);
% gamma = 0; gammas = [];
% is_test_rc = true;
% n_runs = length(batt_ratio);

% for i = 1:n_runs
%     orbit_altitude{i} = alts;
%     n_sat_list{i} = n_sats;
%     num_orbits{i} = n_orbits;
%     inclinations{i} = incs;
%     gammas(i) = gamma;
% end

% title = 'Average Reconstructed Time (s)';
% xlbl = "ratio of battery-drained node";
% ylbl = "elapsed time";
% x_plot_axis = batt_ratio;
% x_tick_lbl = string(batt_ratio);
% saved_prefix = 'saves/outBattery_';
% what_to_plot = 'elapsed_time';


%% Real-World Scenario (config. problematic)
% orbit_altitude = {[540, 700, 900, 1200], [600, 700, 900, 1200, 335.9, 340.8, 345.6]};
% n_sat_list = {[1600, 1600, 520, 720], [1000, 1000, 520, 720, 500, 2440, 2500]}; % numbers of satellies on each layer (define how many layers) 
% num_orbits = {[80, 80, 10, 30], [100, 100, 10, 30, 10, 40, 50]}; 
% inclinations = {[53, 53, 97, 70], [53, 53, 97, 70, 40, 23.5, 47]}; 
% gammas = [0, 0];
% n_runs = 2;
% fixed_batt_ratio = 0.1;
% is_test_rc = true;

% title = 'Average Search Space';
% xlbl = "configuration";
% ylbl = "elapsed time";
% x_plot_axis = [4, 7];
% x_tick_lbl = ["4 planes", "7 planes"];
% saved_prefix = 'saves/RW_rc_';
% what_to_plot = 'elapsed_time';


% % 轉換地面站座標為三維座標
New_York_3D = zeros(1, 3);
[New_York_3D(1), New_York_3D(2), New_York_3D(3)] = llh2xyz(New_York_coords(1), New_York_coords(2), 0, earth_radius);


%% main testing function

% which alg. to test for
xs = ["DBS_{FS}"];
N_item = length(xs);
run_n = length(n_sat_list);

total_latencies = zeros(N_item, run_n);
total_n_visited = zeros(N_item, run_n);
total_elapsed_time = zeros(N_item, run_n);
total_iter_cnts = zeros(N_item, run_n);
total_update_cnts = zeros(N_item, run_n);

for i = 1:run_n

    if is_test_rc
        my_batt_ratio = batt_ratio(i);
    else
        my_batt_ratio = fixed_batt_ratio;
    end

    dst_x = Johannesburg_coords(1); dst_y = Johannesburg_coords(2);
    if ~isempty(dsts)
        dst_x = dsts{i}(1); dst_y = dsts{i}(2);
    end

    Johannesburg_3D = zeros(1, 3);
    [Johannesburg_3D(1), Johannesburg_3D(2), Johannesburg_3D(3)] = ...
        llh2xyz(dst_x, dst_y, 0, earth_radius);

    [latencies, n_visited_list, elapsed_times, iter_cnts, update_cnts] = trial(earth_radius, atm_high, light_speed, ...
                        orbit_altitude{i}, num_orbits{i}, n_sat_list{i}, inclinations{i}, my_batt_ratio, is_ratio_not_number, is_test_rc, ...
                        gammas(i), gammas(i), gammas(i), ... 
                        New_York_3D, Johannesburg_3D, xs);

    total_latencies(:, i) = latencies;
    total_n_visited(:, i) = n_visited_list;
    total_elapsed_time(:, i) = elapsed_times;
    total_iter_cnts(:, i) = iter_cnts;
    total_update_cnts(:, i) = update_cnts;
end

% save my pathetic statstics
saved_lat_csvname = strcat(saved_prefix, 'latency.csv');
saved_visited_csvname = strcat(saved_prefix, 'visited.csv');
saved_time_csvname = strcat(saved_prefix, 'time.csv');
saved_iter_cnts_csvname = strcat(saved_prefix, 'iter_cnt.csv');
saved_update_cnts_csvname = strcat(saved_prefix, 'update_cnt.csv');

csvwrite(saved_lat_csvname, total_latencies);
csvwrite(saved_visited_csvname, total_n_visited);
csvwrite(saved_time_csvname, total_elapsed_time);
csvwrite(saved_iter_cnts_csvname, total_iter_cnts);
csvwrite(saved_update_cnts_csvname, total_update_cnts);

% out-of-battery label text
% out_of_batt_strs = [];
% for i = 1:length(batt_ratio)
%     tmp = strcat("out of battery ratio: ", num2str(batt_ratio(i)));
%     out_of_batt_strs = [out_of_batt_strs, tmp];
% end

% plot lines
switch what_to_plot
    case 'latency'
        plotted_y = total_latencies;
    case 'elapsed_time'
        plotted_y = total_elapsed_time;
    case 'update_count'
        plotted_y = total_update_cnts;
    otherwise
        plotted_y = total_n_visited;
end
    
plot_line(x_plot_axis, plotted_y, xs, xlbl, ylbl, x_tick_lbl);


%% trials

function [latencies, n_visited_list, elapsed_times, iter_costs, update_costs] = trial(earth_radius, atm_high, light_speed, ...
                        orbit_altitude, num_orbits, n_sat_list, inclination, out_of_batt_ratio, is_ratio_not_num, is_test_RC, ...
                        gamma_fs, gamma_op, gamma_rc, ...
                        New_York_3D, Johannesburg_3D, xs)
    
    total_satellites = sum(n_sat_list); 
    satellite_coords = zeros(total_satellites, 3);
    start_i = 1;
    end_i = 0;
    N_item = length(xs);
    % how many time performing RC
    N_avg = 50;

    latencies = zeros(1, N_item);
    n_visited_list = zeros(1, N_item);
    elapsed_times = zeros(1, N_item);
    iter_costs = zeros(1, N_item);
    update_costs = zeros(1, N_item);

    for i = 1:length(n_sat_list)
        n_sat_i = n_sat_list(i);
        n_orbit_i = num_orbits(i);
        alt_i = orbit_altitude(i);
        
        inc_i = 0;
        if ~isempty(inclination)
            inc_i = inclination(i);
        end

        % append coords from each layer to `satellite_coords`
        end_i = end_i + n_sat_i;
        sat_coords_i = generate_coords(i, n_sat_i, n_orbit_i, alt_i, inc_i, earth_radius, atm_high);
        satellite_coords(start_i:end_i, :) = sat_coords_i;
        start_i = start_i + n_sat_i;
    end


    % 找到距離紐約和雪梨最近的衛星
    start_satellite = 0;
    end_satellite = 0;
    min_distance_New_York = inf;
    min_distance_Johannesburg = inf;

    for i = 1:total_satellites
        distance_New_York = norm(New_York_3D - satellite_coords(i, :));
        distance_Johannesburg = norm(Johannesburg_3D - satellite_coords(i, :));
        
        if distance_New_York < min_distance_New_York
            min_distance_New_York = distance_New_York;
            start_satellite = i;
        end
        
        if distance_Johannesburg < min_distance_Johannesburg
            min_distance_Johannesburg = distance_Johannesburg;
            end_satellite = i;
        end
    end

    % for reconstruct test

    empty_L_batt = [];
    [satellite_path_FS, ~, ~, ~] = DBS_FS(satellite_coords, start_satellite, end_satellite, empty_L_batt, gamma_fs, ...
        earth_radius, atm_high, orbit_altitude, n_sat_list);
    P = satellite_path_FS;

    % random determine out-of-battery satellites
    if (is_ratio_not_num)
        N_batt = round(total_satellites * out_of_batt_ratio);
    else
        N_batt = round(out_of_batt_ratio);
    end

    for n_i = 1:N_item
        % tmp registers
        this_lat = 0;
        this_vis = 0;
        this_time = 0;
        this_iter_cnt = 0;
        this_update_cnt = 0;

        switch xs(n_i)

            % 使用Dijkstra演算法找到最短衛星路徑
            case 'Dijkstra'
                
                if is_test_RC
                    % total count
                    path_cnt_Dij = 0.0;
                    vis_cnt_Dij = 0.0;
                    time_cnt_Dij = 0.0;
                    iter_cnt_Dij = 0.0;
                    update_cnt_Dij = 0.0;
                    N_avg = 10; % less number or it will be long AF

                    for i=1:N_avg
                        if (is_ratio_not_num)
                            n_batt_idx = randperm(total_satellites, N_batt);
                            L_batt = transpose(n_batt_idx);
                        else
                            n_batt_idx = randperm(length(P), N_batt);
                            L_batt = P(n_batt_idx);
                        end
    
                        [~, path_cnt, Dijk_time, vis_cnt, dij_iter_cnt, dij_update_cnt] = ...
                                Dijkstra_RC(L_batt, P, satellite_coords, ...
                                        earth_radius, atm_high, orbit_altitude, n_sat_list);
                        
                        path_cnt_Dij = path_cnt_Dij + path_cnt;
                        vis_cnt_Dij = vis_cnt_Dij + vis_cnt;
                        time_cnt_Dij = time_cnt_Dij + Dijk_time;
                        iter_cnt_Dij = iter_cnt_Dij + dij_iter_cnt;
                        update_cnt_Dij = update_cnt_Dij + dij_update_cnt; 
                    end

                    total_distance_Dijkstra = (path_cnt_Dij / N_avg) + min_distance_New_York + min_distance_Johannesburg;
                    this_lat = (total_distance_Dijkstra / light_speed) * 1000;
                    this_vis = vis_cnt_Dij / N_avg;
                    this_time = time_cnt_Dij / N_avg;
                    this_iter_cnt = iter_cnt_Dij / N_avg;
                    this_update_cnt = update_cnt_Dij / N_avg;
                    fprintf("Dijkstra RC avg complete time: %.4f", this_time);
                
                else
                    [satellite_path_Dijkstra, total_satellite_distance_Dijkstra, Dijkstra_time, Dijkstra_n_visited, this_iter_cnt, this_update_cnt] = Dijkstra_algorithm(satellite_coords, start_satellite, end_satellite, total_satellites, ...
                                                                                                                                        earth_radius, atm_high, orbit_altitude, n_sat_list, empty_L_batt);
                    % 計算路徑總距離，包括地面站到最近衛星的距離
                    total_distance_Dijkstra = total_satellite_distance_Dijkstra + min_distance_New_York + min_distance_Johannesburg;

                    % 輸出結果
                    output_summary('Dikstra', Dijkstra_time, satellite_path_Dijkstra, total_distance_Dijkstra, Dijkstra_n_visited);
                    latency_Dij = (total_distance_Dijkstra / light_speed) * 1000;
                    
                    this_lat = latency_Dij;
                    this_vis = Dijkstra_n_visited;
                    this_time = Dijkstra_time;
                end


            % 使用A*演算法找到最短衛星路徑
            case 'A star'
                
                if is_test_RC
                    % total count
                    path_cnt_Astar = 0.0;
                    vis_cnt_Astar = 0.0;
                    time_cnt_Astar = 0.0;
                    iter_cnt_Astar = 0.0;
                    update_cnt_Astar = 0.0;
                    
                    for i=1:N_avg
                        if (is_ratio_not_num)
                            n_batt_idx = randperm(total_satellites, N_batt);
                            L_batt = transpose(n_batt_idx);
                        else
                            n_batt_idx = randperm(length(P), N_batt);
                            L_batt = P(n_batt_idx);
                        end
    
                        [~, path_cnt, Astar_time, vis_cnt, Astar_iter_cnt, Astar_update_cnt] = ...
                                A_star_RC(L_batt, P, satellite_coords, ...
                                        earth_radius, atm_high, orbit_altitude, n_sat_list);
                        time_cnt_Astar = time_cnt_Astar + Astar_time;
                        update_cnt_Astar = Astar_update_cnt + update_cnt_Astar;
                        iter_cnt_Astar = Astar_iter_cnt + iter_cnt_Astar;
                        path_cnt_Astar = path_cnt_Astar + path_cnt;
                        vis_cnt_Astar = vis_cnt_Astar + vis_cnt;
                    end

                    total_distance = (path_cnt_Astar / N_avg) + min_distance_New_York + min_distance_Johannesburg;
                    this_lat = (total_distance / light_speed) * 1000;
                    this_vis = vis_cnt_Astar / N_avg;
                    this_time = (time_cnt_Astar / N_avg) * 10;
                    this_update_cnt = update_cnt_Astar / N_avg;
                    this_iter_cnt = iter_cnt_Astar / N_avg;
                
                else
                    [~, total_satellite_distance, A_star_time, A_n_visited, this_iter_cnt, Astar_update_cnt] = A_star_algorithm(satellite_coords, start_satellite, end_satellite, total_satellites, ...
                                                                                                                earth_radius, atm_high, orbit_altitude, n_sat_list, empty_L_batt);
                    % 計算路徑總距離，包括地面站到最近衛星的距離
                    total_distance = total_satellite_distance + min_distance_New_York + min_distance_Johannesburg;
                    latency_Astar = (total_distance / light_speed) * 1000;

                    this_lat = latency_Astar;
                    this_vis = A_n_visited;
                    this_time = A_star_time;
                    this_update_cnt = Astar_update_cnt;
                end

            % result of DBS_OP
            case 'DBS_{OP}'
                [satellite_path_OP, total_satellite_distance_OP, OP_time, OP_n_visited, this_iter_cnt, OP_update_cnt] = ...
                        DBS_OP(satellite_coords, start_satellite, end_satellite, gamma_op, ...
                                earth_radius, atm_high, orbit_altitude, n_sat_list);

                % 計算路徑總距離，包括地面站到最近衛星的距離
                total_distance = total_satellite_distance_OP + min_distance_New_York + min_distance_Johannesburg;

                % 輸出結果
                % output_summary('DBS_OP', OP_time, satellite_path_OP, total_distance, OP_n_visited);
                % output_path('DBS_OP', total_distance, light_speed, ...
                            % satellite_coords, satellite_path_OP, ...
                            % min_distance_New_York, min_distance_Johannesburg);

                latency_OP = (total_distance / light_speed) * 1000;

                this_lat = latency_OP;
                this_vis = OP_n_visited;
                this_time = OP_time;
                this_update_cnt = OP_update_cnt;
                

            % result of DBS_FS
            case 'DBS_{FS}'
                L_batt = [];
                [satellite_path_FS, total_satellite_distance_FS, FS_time, FS_n_visited, this_iter_cnt, FS_update_cnt] = ...
                        DBS_FS(satellite_coords, start_satellite, end_satellite, L_batt, gamma_fs, ...
                                earth_radius, atm_high, orbit_altitude, n_sat_list);

                % 計算路徑總距離，包括地面站到最近衛星的距離
                total_distance = total_satellite_distance_FS + min_distance_New_York + min_distance_Johannesburg;

                % 輸出結果
                output_summary('DBS_FS', FS_time, satellite_path_FS, total_distance, FS_n_visited);
                output_path('DBS_FS', total_distance, light_speed, ...
                            satellite_coords, satellite_path_FS, ...
                            min_distance_New_York, min_distance_Johannesburg);

                latency_FS = (total_distance / light_speed) * 1000;
        
                this_lat = latency_FS;
                this_vis = FS_n_visited;
                this_time = FS_time;
                this_update_cnt = FS_update_cnt;


            %% result of DBS_RC (the impact of battery-drained nodes)
            case 'DBS_{RC}'

                % total count
                latency_cnt = 0.0;
                n_vis_cnt = 0.0;
                time_cnt = 0.0;
                update_cnt = 0.0;
                iter_cnt = 0.0;
                
                % take average of `DBS_RC`
                for avg_i = 1:N_avg
                    if (is_ratio_not_num)
                        n_batt_idx = randperm(total_satellites, N_batt);
                        L_batt = transpose(n_batt_idx);
                    else
                        n_batt_idx = randperm(length(P), N_batt);
                        L_batt = P(n_batt_idx);
                    end

                    [~, total_satellite_distance_RC, rc_time, n_visited_RC, RC_iter_cnt, RC_update_cnt] = ...
                            DBS_RC(L_batt, P, satellite_coords, gamma_rc, ...
                                    earth_radius, atm_high, orbit_altitude, n_sat_list);

                    % 計算路徑總距離，包括地面站到最近衛星的距離
                    total_distance = total_satellite_distance_RC + min_distance_New_York + min_distance_Johannesburg;

                    latency_cnt = latency_cnt + (total_distance / light_speed) * 1000;
                    n_vis_cnt = n_vis_cnt + n_visited_RC;
                    time_cnt = time_cnt + rc_time;
                    update_cnt = update_cnt + RC_update_cnt;
                    iter_cnt = iter_cnt + RC_iter_cnt;
                end
                
                this_lat = latency_cnt / N_avg;
                this_vis = round(n_vis_cnt / N_avg);
                this_time = time_cnt / N_avg;
                this_update_cnt = round(update_cnt / N_avg);
                this_iter_cnt = round(iter_cnt / N_avg);

            otherwise
                fprintf('[TRIALS WARNING] Unexpected item for trial: %s\n', xs(n_i));
                
        end 

        latencies(1, n_i) = this_lat;
        n_visited_list(1, n_i) = this_vis;
        elapsed_times(1, n_i) = this_time;
        update_costs(1, n_i) = this_update_cnt;
        iter_costs(1, n_i) = this_iter_cnt;
    end

    % n_visited_list = log2(n_visited_list);

end


