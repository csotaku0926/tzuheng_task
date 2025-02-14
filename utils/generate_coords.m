function satellite_coords = generate_coords(layer_i, n_sat, num_orbits, orbit_altitude, inclination, earth_radius, atm_high)
    % x = sqrt((r + h)^2 - (r + a)^2) * 2   根據衛星高度與大氣層厚度計算最長hop距離
    max_dis_perhop = floor(sqrt((earth_radius + orbit_altitude)^2 - (earth_radius + atm_high)^2) * 2);
    satellites_per_orbit = round(n_sat / num_orbits);
    fprintf('Layer %.d\n', layer_i)
    fprintf('衛星海拔高度: %.fkm\n', orbit_altitude);
    fprintf('衛星數量: %.f顆 (per orbit: %d)\n', n_sat, satellites_per_orbit);
    fprintf('最大LISL: %.fkm\n', max_dis_perhop);
    fprintf('\n');

    % 生成衛星座標
    satellite_coords = zeros(n_sat, 3);
    for orbit = 1:num_orbits
        % 設定軌道傾角為0，即沿赤道
        % inclination = 0;
        
        for satellite = 1:satellites_per_orbit
            index = (orbit - 1) * satellites_per_orbit + satellite;
            theta = 2 * pi / satellites_per_orbit * (satellite - 1);
            phi = (2 * pi / num_orbits) * (orbit - 1);
            
            satellite_coords(index, 1) = (earth_radius + orbit_altitude) * cosd(inclination) * cos(phi) * cos(theta) - (earth_radius + orbit_altitude) * sind(inclination) * sin(phi);
            satellite_coords(index, 2) = (earth_radius + orbit_altitude) * cosd(inclination) * cos(phi) * sin(theta) + (earth_radius + orbit_altitude) * sind(inclination) * cos(phi);
            satellite_coords(index, 3) = (earth_radius + orbit_altitude) * sind(inclination) * cos(phi) * cos(theta) + (earth_radius + orbit_altitude) * cosd(inclination) * sin(phi);
        end
    end
end
