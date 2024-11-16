% Satellite Handover Simulation Script with Ping Measurement

function lon_offset = find_lon_offset(satellite, time_offset)
    [x, y, z, lat, lon, alt, satellite] = satellite_position(satellite, time_offset, true);
    lon_offset = rad2deg(lon);
end

function satellite = set_time_to_latitude(satellite)
    if satellite.latitude == 0
        satellite.time_to_latitude = 0;
        return;
    end

    R_earth = 6371;
    mu = 398600.4418;

    lat_target_rad = deg2rad(satellite.latitude);

    altitude = satellite.altitude;
    inclination = deg2rad(satellite.inclination);
    a = R_earth + altitude; 
    T = 2 * pi * sqrt(a^3 / mu); 
    theta_target = asin(sin(lat_target_rad) / sin(inclination));
    
    satellite.time_to_latitude = (theta_target / (2 * pi)) * T;
end

function [X, Y, Z] = geodetic_to_cartesian(lat, lon, alt)
    R_earth = 6371;
    r_orbit = R_earth + alt;
    
    X = (r_orbit) * cos(lat) * cos(lon);
    Y = (r_orbit) * cos(lat) * sin(lon);
    Z = (r_orbit) * sin(lat);
end

function [latitude, longitude, altitude] = cartesian_to_geodetic(x, y, z)
    R_earth = 6371;

    p = sqrt(x^2 + y^2);  % distance from z-axis
    latitude = atan2(z, p);
    altitude = p / cos(latitude) - R_earth;
    longitude = atan2(y, x);
end

function [x, y, z, satellite] = satellite_position_cartesian(satellite, time_offset, finding_offset)
    R_earth = 6371; % For spherical earth
    mu = 398600.4418;
    
    if isempty(satellite.time_to_latitude)
        satellite = set_time_to_latitude(satellite);
    end

    time_offset = satellite.time_to_latitude + time_offset;

    if isempty(satellite.lon_offset) && ~finding_offset
        satellite.lon_offset = find_lon_offset(satellite, 0);
    end

    if finding_offset
        longitude = 0;
    else
        longitude = deg2rad(satellite.longitude - satellite.lon_offset);
    end

    altitude = satellite.altitude;
    inclination = deg2rad(satellite.inclination);
    
    r_orbit = R_earth + altitude;
    period = 2 * pi * sqrt(r_orbit^3 / mu);
    
    mean_anomaly = 2 * pi * (time_offset / period);
    
    % Calculate orbital coordinates in the orbital plane
    x_orbit = r_orbit * cos(mean_anomaly);
    y_orbit = r_orbit * sin(mean_anomaly);
    z_orbit = 0;
    
    % Apply the orbital inclination
    x_rot = x_orbit;
    y_rot = y_orbit * cos(inclination) - z_orbit * sin(inclination);
    z_rot = y_orbit * sin(inclination) + z_orbit * cos(inclination);
    
    omega_earth = 2 * pi / 86164; % Earth’s angular velocity in radians per second
    rotation_angle = omega_earth * time_offset; % Earth's rotation angle for the given time
    R_earth_rotation = [cos(rotation_angle), sin(rotation_angle), 0; 
                        -sin(rotation_angle), cos(rotation_angle), 0;
                         0, 0, 1];
    pos_rotated = R_earth_rotation * [x_rot; y_rot; z_rot];

    x = pos_rotated(1);
    y = pos_rotated(2);
    z = pos_rotated(3);

    % Offset to desired initial longitude
    [lat, lon, alt] = cartesian_to_geodetic(x, y, z);
    lon = lon + longitude;
    [x, y, z] = geodetic_to_cartesian(lat, lon, alt);
end

function [x, y, z, lat, lon, alt, satellite] = satellite_position(satellite, time_offset, finding_offset)
    % Pass "false" to the finding_offset parameter when calling this from anywhere except the find_lon_offset function
    [x, y, z, satellite] = satellite_position_cartesian(satellite, time_offset, finding_offset);
    [lat, lon, alt] = cartesian_to_geodetic(x, y, z);
end

function [shell] = generate_even_spaced_shell(original_satellite, satellite_count, dt)
    % Note: spacing indicates the time difference in the orbital plane
    shell(satellite_count) = HierarchySatellite;
    for i = 1:satellite_count
        [x, y, z, lat, lon, alt, other_satellite] = satellite_position(original_satellite, (i - 1) * dt, false);
        other_satellite.latitude = lat;
        other_satellite.longitude = lon;
        other_satellite.altitude = alt;
        other_satellite.time_to_latitude = nan;
        other_satellite.lon_offset = nan;
        shell(i) = other_satellite;
    end
end

function write_sat_array_to_csv(arr, filename)
    cnt = size(arr, 2);
    name = strings(10, 1);
    initial_altitude = zeros(cnt, 1);
    inclination = zeros(cnt, 1);
    initial_latitude = zeros(cnt, 1);
    initial_longitude = zeros(cnt, 1);
    sat_type = strings(cnt, 1);
    for i = 1:cnt
        name(i, 1) = strcat(arr(1, 1).name, int2str(i));
        initial_altitude(i, 1) = arr(1, i).altitude;
        inclination(i, 1) = arr(1, i).inclination;
        initial_longitude(i, 1) = arr(1, i).longitude;
        initial_latitude(i, 1) = arr(1, i).latitude;
        if (initial_altitude(i, 1) < 35786); sat_type(i, 1) = "LEO"; else; sat_type(i, 1) = "GEO"; end
    end
    sat_table = table(name, initial_altitude, inclination, initial_latitude, initial_longitude, sat_type);
    writetable(sat_table, filename);
end

% Constants
mu = 398600.4418; % Earth's gravitational parameter (km^3/s^2)
R_earth = 6371; % Earth's radius in km
c = 299792.458; % Speed of light in km/s
GEO_altitude = 35786; % Altitude of geosynchronous satellites in km

% Ground Station Position (Example at Equator)
groundStation_position = [R_earth, 0, 0]; % on Earth's surface at equator

% Simulation Settings
dt = 10; % Time step in seconds
simulation_duration = 86000;
num_steps = simulation_duration / dt;

T = readtable("sample_satellite_params.csv", 'NumHeaderLines', 1);
G = findgroups(T{:, 6});
T_split = splitapply( @(varargin) varargin, T , G);
subTables = cell(size(T_split, 1));
for i = 1:size(T_split, 1)
    subTables{i} = table(T_split{i, :}, 'VariableNames', T.Properties.VariableNames);
end
first_row_label = T(1,6).Var6;
if (first_row_label == "LEO" && G(1,1) == 1)
    leo_satellite_params = table2array(subTables{1}(:, 2:5));
    geo_satellite_params = table2array(subTables{2}(:, 2:5));
elseif (first_row_label == "LEO" && G(1,1) == 2)
    geo_satellite_params = table2array(subTables{1}(:, 2:5));
    leo_satellite_params = table2array(subTables{2}(:, 2:5));
elseif (first_row_label == "GEO" && G(1,1) == 1)
    geo_satellite_params = table2array(subTables{1}(:, 2:5));
    leo_satellite_params = table2array(subTables{2}(:, 2:5));
else
    leo_satellite_params = table2array(subTables{1}(:, 2:5));
    geo_satellite_params = table2array(subTables{2}(:, 2:5));
end

leo_satellites(height(leo_satellite_params)) = HierarchySatellite;
for i = 1:height(leo_satellite_params)
    params = leo_satellite_params(i, :);
    leo_satellites(i) = HierarchySatellite(params(1), params(2), params(3), params(4));
end

geo_satellites(height(geo_satellite_params)) = HierarchySatellite;
for i = 1:height(geo_satellite_params)
    params = geo_satellite_params(i, :);
    geo_satellites(i) = HierarchySatellite(params(1), params(2), params(3), params(4));
end

leo_positions = zeros(num_steps, size(leo_satellites, 2), 6);
geo_positions = zeros(num_steps, size(geo_satellites, 2), 6);

% Simulation Loop: Calculate GEO and LEO Positions
for step = 1:num_steps
    for i = 1:size(geo_satellites, 2)
        [x, y, z, lat, lon, alt, geo_satellites(i)] = satellite_position(geo_satellites(i), (step - 1) * dt, false);
        geo_position = [x, y, z];
        geo_positions(step, i, :) = [x, y, z, lat, lon, alt];
    end

    for i = 1:size(leo_satellites, 2)
        [x, y, z, lat, lon, alt, leo_satellites(i)] = satellite_position(leo_satellites(i), (step - 1) * dt, false);
        leo_position = [x, y, z];
        leo_positions(step, i, :) = [x, y, z, lat, lon, alt];
    end
end

% Create a 3D Globe inside a uifigure
uif = uifigure;           % Create a UI figure for the globe
g = geoglobe(uif, 'NextPlot', 'add');        % Create the geoglobe within the UI figure

% Add a title to the UI figure using a uilabel
titleLabel = uilabel(uif, 'Text', 'Satellite Handover Simulation Between GEO and LEO Satellites', ...
    'Position', [10, uif.Position(4) - 30, 400, 30], 'FontSize', 14, 'FontWeight', 'bold');

colors = hsv(size(leo_positions, 2));
for i = 1:size(leo_positions, 2)
    lat = rad2deg(leo_positions(:, i, 4));
    lon = rad2deg(leo_positions(:, i, 5));
    alt = leo_positions(:, i, 6);
    geoplot3(g, lat, lon, alt, 'LineWidth', 2, 'Color', colors(i,:));
end

colors = hsv(size(geo_positions,2));
for i = 1:size(geo_positions, 2)
    lat = rad2deg(geo_positions(:, i, 4));
    lon = rad2deg(geo_positions(:, i, 5));
    alt = geo_positions(:, i, 6);
    geoplot3(g, lat, lon, alt, 'LineWidth', 2, 'Color', colors(i,:));
end


% % This is how to save the locations to a csv file
% writematrix(leo_positions, 'sample_satellite_location_output.csv')
% writematrix(geo_positions, 'sample_satellite_location_output.csv', 'WriteMode', 'append');

% % This is how to generate a dummy shell with evenly spaced satellites
% dummy = leo_satellites(i);
% dummy.name = "EastLansing";
% dummies = generate_even_spaced_shell(dummy, 100, 10);

% % This is how to write an array of satellites to a file
% write_sat_array_to_csv(dummies, "sample_generated_satellite_shell.csv");