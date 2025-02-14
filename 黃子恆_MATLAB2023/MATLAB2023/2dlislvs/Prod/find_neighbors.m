% 當前衛星節點搜尋其鄰居節點
%
%{
function neighbors = find_neighbors(satellite_coords, current_satellite)
    neighbors = [];
    for i = 1:size(satellite_coords, 1)
        if i ~= current_satellite 
            max_dis_perhop = calculate_max_dis(satellite_coords(current_satellite, 4), satellite_coords(i, 4));
            if distance_between(satellite_coords, current_satellite, i) <= max_dis_perhop
                neighbors = [neighbors; i];
            end
        end
    end
end

function max_dis = calculate_max_dis(h1, h2)
    earth_radius = 6378; % 地球半徑(km)
    atm_high = 80; % 含水氣之大氣層海拔高度(km)
    if h1 ~= h2
        max_dis = floor(sqrt((earth_radius + h1)^2 - (earth_radius + atm_high)^2) + sqrt((earth_radius + h2)^2 - (earth_radius + atm_high)^2));
    else
        max_dis = floor(sqrt((earth_radius + h1)^2 - (earth_radius + atm_high)^2)*2);
    end
end
%}


function neighbors = find_neighbors(satellite_coords, current_satellite)
    neighbors = [];
    for i = 1:size(satellite_coords, 1)
        if i ~= current_satellite
            satellite_LISL_sum = satellite_coords(current_satellite, 4) + satellite_coords(i, 4);
            if satellite_LISL_sum == 1080
                max_dis_perhop = 4961;
                if distance_between(satellite_coords, current_satellite, i) <= max_dis_perhop
                    neighbors = [neighbors; i];
                end
            end
            if satellite_LISL_sum == 1090
                max_dis_perhop = 4988;
                if distance_between(satellite_coords, current_satellite, i) <= max_dis_perhop
                    neighbors = [neighbors; i];
                end
            end
            if satellite_LISL_sum == 1100
                max_dis_perhop = 5016;
                if distance_between(satellite_coords, current_satellite, i) <= max_dis_perhop
                    neighbors = [neighbors; i];
                end
            end
            if satellite_LISL_sum == 1110
                max_dis_perhop = 5043;
                if distance_between(satellite_coords, current_satellite, i) <= max_dis_perhop
                    neighbors = [neighbors; i];
                end
            end
            if satellite_LISL_sum == 875.9
                max_dis_perhop = 4316;
                if distance_between(satellite_coords, current_satellite, i) <= max_dis_perhop
                    neighbors = [neighbors; i];
                end
            end
            if satellite_LISL_sum == 880.8
                max_dis_perhop = 4334;
                if distance_between(satellite_coords, current_satellite, i) <= max_dis_perhop
                    neighbors = [neighbors; i];
                end
            end
            if satellite_LISL_sum == 885.6
                max_dis_perhop = 4351;
                if distance_between(satellite_coords, current_satellite, i) <= max_dis_perhop
                    neighbors = [neighbors; i];
                end
            end
            if satellite_LISL_sum == 1120
                max_dis_perhop = 5071;
                if distance_between(satellite_coords, current_satellite, i) <= max_dis_perhop
                    neighbors = [neighbors; i];
                end
            end
            if satellite_LISL_sum == 1130
                max_dis_perhop = 5098;
                if distance_between(satellite_coords, current_satellite, i) <= max_dis_perhop
                    neighbors = [neighbors; i];
                end
            end
            if satellite_LISL_sum == 1140
                max_dis_perhop = 5125;
                if distance_between(satellite_coords, current_satellite, i) <= max_dis_perhop
                    neighbors = [neighbors; i];
                end
            end
            if satellite_LISL_sum == 885.9
                max_dis_perhop = 4344;
                if distance_between(satellite_coords, current_satellite, i) <= max_dis_perhop
                    neighbors = [neighbors; i];
                end
            end
            if satellite_LISL_sum == 895.9
                max_dis_perhop = 4371;
                if distance_between(satellite_coords, current_satellite, i) <= max_dis_perhop
                    neighbors = [neighbors; i];
                end
            end
            if satellite_LISL_sum == 905.9
                max_dis_perhop = 4398;
                if distance_between(satellite_coords, current_satellite, i) <= max_dis_perhop
                    neighbors = [neighbors; i];
                end
            end
            if satellite_LISL_sum == 671.8
                max_dis_perhop = 3671;
                if distance_between(satellite_coords, current_satellite, i) <= max_dis_perhop
                    neighbors = [neighbors; i];
                end
            end
            if satellite_LISL_sum == 890.8
                max_dis_perhop = 4362;
                if distance_between(satellite_coords, current_satellite, i) <= max_dis_perhop
                    neighbors = [neighbors; i];
                end
            end
            if satellite_LISL_sum == 900.8
                max_dis_perhop = 4389;
                if distance_between(satellite_coords, current_satellite, i) <= max_dis_perhop
                    neighbors = [neighbors; i];
                end
            end
            if satellite_LISL_sum == 910.8
                max_dis_perhop = 4416;
                if distance_between(satellite_coords, current_satellite, i) <= max_dis_perhop
                    neighbors = [neighbors; i];
                end
            end
            if satellite_LISL_sum == 676.7
                max_dis_perhop = 3689;
                if distance_between(satellite_coords, current_satellite, i) <= max_dis_perhop
                    neighbors = [neighbors; i];
                end
            end
            if satellite_LISL_sum == 681.6
                max_dis_perhop = 3707;
                if distance_between(satellite_coords, current_satellite, i) <= max_dis_perhop
                    neighbors = [neighbors; i];
                end
            end
            if satellite_LISL_sum == 895.6
                max_dis_perhop = 4379;
                if distance_between(satellite_coords, current_satellite, i) <= max_dis_perhop
                    neighbors = [neighbors; i];
                end
            end
            if satellite_LISL_sum == 905.6
                max_dis_perhop = 4406;
                if distance_between(satellite_coords, current_satellite, i) <= max_dis_perhop
                    neighbors = [neighbors; i];
                end
            end
            if satellite_LISL_sum == 915.6
                max_dis_perhop = 4434;
                if distance_between(satellite_coords, current_satellite, i) <= max_dis_perhop
                    neighbors = [neighbors; i];
                end
            end
            if satellite_LISL_sum == 681.5
                max_dis_perhop = 3707;
                if distance_between(satellite_coords, current_satellite, i) <= max_dis_perhop
                    neighbors = [neighbors; i];
                end
            end
            if satellite_LISL_sum == 686.4
                max_dis_perhop = 3724;
                if distance_between(satellite_coords, current_satellite, i) <= max_dis_perhop
                    neighbors = [neighbors; i];
                end
            end
            if satellite_LISL_sum == 691.2
                max_dis_perhop = 3742;
                if distance_between(satellite_coords, current_satellite, i) <= max_dis_perhop
                    neighbors = [neighbors; i];
                end
            end
        end
    end
end

