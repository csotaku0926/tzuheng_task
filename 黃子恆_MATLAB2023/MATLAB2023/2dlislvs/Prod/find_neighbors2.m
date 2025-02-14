function neighbors = find_neighbors2(satellite_coords, current_satellite, intersect_radius)
    neighbors = [];
    for i = 1:size(satellite_coords, 1)
        if i ~= current_satellite
            satellite_LISL_sum = satellite_coords(current_satellite, 4) + satellite_coords(i, 4);
            if satellite_LISL_sum == 1080
                max_dis_perhop = min(4961, intersect_radius);
            elseif satellite_LISL_sum == 1090
                max_dis_perhop = min(4988, intersect_radius);
            elseif satellite_LISL_sum == 1100
                max_dis_perhop = min(5016, intersect_radius);
            elseif satellite_LISL_sum == 1110
                max_dis_perhop = min(5043, intersect_radius);
            elseif satellite_LISL_sum == 875.9
                max_dis_perhop = min(4316, intersect_radius);
            elseif satellite_LISL_sum == 880.8
                max_dis_perhop = min(4334, intersect_radius);
            elseif satellite_LISL_sum == 885.6
                max_dis_perhop = min(4351, intersect_radius);
            elseif satellite_LISL_sum == 1120
                max_dis_perhop = min(5071, intersect_radius);
            elseif satellite_LISL_sum == 1130
                max_dis_perhop = min(5098, intersect_radius);
            elseif satellite_LISL_sum == 1140
                max_dis_perhop = min(5125, intersect_radius);
            elseif satellite_LISL_sum == 885.9
                max_dis_perhop = min(4344, intersect_radius);
            elseif satellite_LISL_sum == 895.9
                max_dis_perhop = min(4371, intersect_radius);
            elseif satellite_LISL_sum == 905.9
                max_dis_perhop = min(4398, intersect_radius);
            elseif satellite_LISL_sum == 671.8
                max_dis_perhop = min(3671, intersect_radius);
            elseif satellite_LISL_sum == 890.8
                max_dis_perhop = min(4362, intersect_radius);
            elseif satellite_LISL_sum == 900.8
                max_dis_perhop = min(4389, intersect_radius);
            elseif satellite_LISL_sum == 910.8
                max_dis_perhop = min(4416, intersect_radius);
            elseif satellite_LISL_sum == 676.7
                max_dis_perhop = min(3689, intersect_radius);
            elseif satellite_LISL_sum == 681.6
                max_dis_perhop = min(3707, intersect_radius);
            elseif satellite_LISL_sum == 895.6
                max_dis_perhop = min(4379, intersect_radius);
            elseif satellite_LISL_sum == 905.6
                max_dis_perhop = min(4406, intersect_radius);
            elseif satellite_LISL_sum == 915.6
                max_dis_perhop = min(4434, intersect_radius);
            elseif satellite_LISL_sum == 681.5
                max_dis_perhop = min(3707, intersect_radius);
            elseif satellite_LISL_sum == 686.4
                max_dis_perhop = min(3724, intersect_radius);
            elseif satellite_LISL_sum == 691.2
                max_dis_perhop = min(3742, intersect_radius);
            else
                continue;
            end
            
            if distance_between(satellite_coords, current_satellite, i) <= max_dis_perhop
                neighbors = [neighbors; i];
            end
        end
    end
end
