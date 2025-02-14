function distance_matrix = calculate_distance_matrix(satellite_coords, total_satellites)
    distance_matrix = inf(total_satellites, total_satellites);
    for i = 1 : total_satellites
        for j = 1 : total_satellites
            if i ~= j
                distance = norm(satellite_coords(i, :) - satellite_coords(j, :));
                if distance <= max_dis_perhop
                    distance_matrix(i, j) = distance;
                end
            end
        end
    end
end