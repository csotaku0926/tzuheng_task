% Parameter settings
earth_radius = 6378;
orbit_altitude = 550;
total_satellites = 11952;
num_orbits = 72;
inclination = 53;

% Calculate satellite positions
satellites_per_orbit = total_satellites / num_orbits;
satellite_positions = zeros(total_satellites, 3);
index = 1;

for orbit = 1:num_orbits
    RAAN = (orbit - 1) * 360 / num_orbits;
    for satellite = 1:satellites_per_orbit
        true_anomaly = (satellite - 1) * 360 / satellites_per_orbit;
        [x, y, z] = calculate_satellite_coordinates(earth_radius, orbit_altitude, RAAN, inclination, true_anomaly);
        satellite_positions(index, :) = [x, y, z];
        index = index + 1;
    end
end

function [x, y, z] = calculate_satellite_coordinates(earth_radius, orbit_altitude, RAAN, inclination, true_anomaly)
    R = earth_radius + orbit_altitude;
    inclination_rad = deg2rad(inclination);
    RAAN_rad = deg2rad(RAAN);
    true_anomaly_rad = deg2rad(true_anomaly);

    x = R * (cos(RAAN_rad) * cos(true_anomaly_rad) - sin(RAAN_rad) * sin(true_anomaly_rad) * cos(inclination_rad));
    y = R * (sin(RAAN_rad) * cos(true_anomaly_rad) + cos(RAAN_rad) * sin(true_anomaly_rad) * cos(inclination_rad));
    z = R * sin(inclination_rad) * sin(true_anomaly_rad);
end
