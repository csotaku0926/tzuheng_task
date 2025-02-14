% 計算並定位衛星，並轉換為3維座標資訊
function [x, y, z] = calculate_satellite_coordinates(earth_radius, orbit_altitude, RAAN, inclination, true_anomaly)
    R = earth_radius + orbit_altitude;
    inclination_rad = deg2rad(inclination);
    RAAN_rad = deg2rad(RAAN);
    true_anomaly_rad = deg2rad(true_anomaly);

    x = R * (cos(RAAN_rad) * cos(true_anomaly_rad) - sin(RAAN_rad) * sin(true_anomaly_rad) * cos(inclination_rad));
    y = R * (sin(RAAN_rad) * cos(true_anomaly_rad) + cos(RAAN_rad) * sin(true_anomaly_rad) * cos(inclination_rad));
    z = R * sin(inclination_rad) * sin(true_anomaly_rad);
end