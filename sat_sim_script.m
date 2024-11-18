% Satellite Handover Simulation Script with Ping Measurement

function lon_offset = find_lon_offset(satellite, time_offset)
    % Calculate longitude offset to make it such that a satellite will be at the longitude at the given time offset
    % Input:
    %   - satellite: HierarchySatellite object to calculate the lon_offset for
    %   - time_offset: The time offset for which to find the lon_offset
    % Output:
    %   - lon_offset: How many degrees to offset longitude by so that the satellite is at the desired satellite.longitude parameter at time_offset
    [x, y, z, lat, lon, alt, satellite] = satellite_position(satellite, time_offset, true);
    lon_offset = rad2deg(lon);
end

function [vx, vy, vz] = calculate_velocity(satellite, time)
    % Calculate velocity using central difference method
    dt = 0.1;  % Time step for velocity calculation
    [x1, y1, z1, ~] = satellite_position_cartesian(satellite, time - dt, false);
    [x2, y2, z2, ~] = satellite_position_cartesian(satellite, time + dt, false);
    
    % Calculate velocity components
    vx = (x2 - x1) / (2 * dt);
    vy = (y2 - y1) / (2 * dt);
    vz = (z2 - z1) / (2 * dt);
end

function satellite = set_time_to_latitude(satellite)
    % Set the time_to_latitude parameter in the given satellite such the satellite is at the satellite.latitude parameter at that time
    % Input:
    %   - satellite: HierarchySatellite object to calculate the time_to_latitude parameter for
    % Output:
    %   - satellite: The updated HierarchySatellite object
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
    % Convert geodetic coordinates to cartesian coordinates
    % Geodetic coordinates must be given in radians
    R_earth = 6371;
    r_orbit = R_earth + alt;
    
    X = (r_orbit) * cos(lat) * cos(lon);
    Y = (r_orbit) * cos(lat) * sin(lon);
    Z = (r_orbit) * sin(lat);
end

function [latitude, longitude, altitude] = cartesian_to_geodetic(x, y, z)
    % Convert cartesian coordiantes to geodetic coordinates
    % Geodetic coordinates are given in radians
    R_earth = 6371;

    p = sqrt(x^2 + y^2);  % distance from z-axis
    latitude = atan2(z, p);
    altitude = p / cos(latitude) - R_earth;
    longitude = atan2(y, x);
end

function [x, y, z, satellite] = satellite_position_cartesian(satellite, time_offset, finding_offset)
    % Calculate the satellite position in cartesian coordinates. 
    % If finding_offset is true, this function calculates the position without regard to the satellite.longitude or satellite.lon_offset parameters
    % Input:
    %   - satellite: Hierarchy satellite object to calculate the cartesian coordinates for
    %   - time_offset: The time_offset from the beginning of the orbit at which to calculate the position
    %   - finding_offset: Boolean to determine whether we are need to find the satellite.lon_offset parameter (this is done to avoid infinite recursion)
    % Output:
    %   - x: The cartesian x-coordinate in kilometers at the given time offset
    %   - y: The cartesian y-coordinate in kilometers at the given time offset
    %   - z: The cartesian z-coordinate in kilometers at the given time offset
    %   - satellite: The updated satellite with the time_to_latitude and lon_offset parameters set if not previously set (will only be modified on first call to this function)

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
    % Calculate the position of the given HierarchySatellite in cartesian and geodetic coordinates
    % If finding_offset is true, this function calculates the position without regard to the satellite.longitude or satellite.lon_offset parameters
    % Pass "false" to the finding_offset parameter when calling this from anywhere except the find_lon_offset function
    [x, y, z, satellite] = satellite_position_cartesian(satellite, time_offset, finding_offset);
    [lat, lon, alt] = cartesian_to_geodetic(x, y, z);
end

function [shell] = generate_even_spaced_shell(original_satellite, satellite_count, dt)
    % Generate a shell of satellites based off of the configuration of a single satellite
    % Input:
    %   - original_satellite: The satellite to base the shell off
    %   - satellite_count: The number of satellites in the shell
    %   - dt: The number of seconds between the times two adjacent satellites will pass over the same location in the orbital plane
    % Output:
    %   - shell: An array of HierarchySatellites that corresponds to the desired parameters (includes the original satellite as well)
    shell(satellite_count) = HierarchySatellite;
    for i = 1:satellite_count
        [x, y, z, lat, lon, alt, other_satellite] = satellite_position(original_satellite, (i - 1) * dt, false);
        other_satellite.latitude = rad2deg(lat);
        other_satellite.longitude = rad2deg(lon);
        other_satellite.altitude = alt;
        other_satellite.time_to_latitude = nan;
        other_satellite.lon_offset = nan;
        shell(i) = other_satellite;
    end
end

function write_sat_array_to_csv(arr, filename)
    % Write parameters for an array of HierarchySatellite objects to csv file that can be loaded for a later run of this script
    % Input:
    %   - arr: The array of HierarchySatellite objects
    %   - filename: The name of the file to save the satellite information to
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

% Initialize the current LEO index
currentLEO_index = 0; % Start with the first LEO satellite
predictedHandoverPoints = []; % To store handover points for visualization

% Constants
mu = 398600.4418; % Earth's gravitational parameter (km^3/s^2)
R_earth = 6371; % Earth's radius in km
c = 299792.458; % Speed of light in km/s
GEO_altitude = 35786; % Altitude of geosynchronous satellites in km

% Ground Station Position (Example at Equator)
[x, y, z] = geodetic_to_cartesian(deg2rad(42.737652), deg2rad(-84.483788), 0.261);
user_position = [x, y, z]; % on Earth's surface at equator

% Simulation Settings
dt = 10; % Time step in seconds
simulation_duration = 1000;
num_steps = simulation_duration / dt;

T = readtable("multiple_shells_params_doubled.csv", 'NumHeaderLines', 1);
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

% % This is how to generate multiple shells of satellites based on a few initial satellites.
% % You may need to customize some parameters if different shells need different spacings or numbers of satellites
% sat_cnt_per_shell = 17;
% generated_sats(sat_cnt_per_shell * size(leo_satellites, 1)) = HierarchySatellite;
% size(leo_satellites, 2)
% for i = 1:size(leo_satellites, 2)
%     shell = generate_even_spaced_shell(leo_satellites(i), sat_cnt_per_shell, 337);
%     generated_sats((i - 1) * sat_cnt_per_shell + 1 : i * sat_cnt_per_shell) = shell(1:sat_cnt_per_shell);
% end
% generated_sats(end + 1) = geo_satellites(1);
% write_sat_array_to_csv(generated_sats, "multiple_shells_params_doubled.csv");

% disp("written")

leo_positions = zeros(num_steps, size(leo_satellites, 2), 6);
geo_positions = zeros(num_steps, size(geo_satellites, 2), 6);
leo_velocities = zeros(num_steps, size(leo_satellites, 2), 3);

% Initialize arrays to store data
ping_history = zeros(num_steps, 1);
time_history = zeros(num_steps, 1);
leo_connection_history = zeros(num_steps, 1);

% Before the simulation loop, create the UI elements
% Create a 3D Globe inside a uifigure
% uif = uifigure;           % Create a UI figure for the globe
% g = geoglobe(uif, 'NextPlot', 'add');        % Create the geoglobe within the UI figure

% % Add a title to the UI figure using a uilabel
% titleLabel = uilabel(uif, 'Text', 'Satellite Handover Simulation Between GEO and LEO Satellites', ...
%     'Position', [10, uif.Position(4) - 30, 400, 30], 'FontSize', 14, 'FontWeight', 'bold');

% % Add a ping display label to the UI figure
% pingLabel = uilabel(uif, 'Text', 'Current Ping: N/A', ...
%     'Position', [10, uif.Position(4) - 60, 400, 30], 'FontSize', 12);

% Simulation Loop: Calculate GEO and LEO Positions
for step = 1:num_steps
    % Calculate current simulation time
    current_time = (step - 1) * dt;  % Time in seconds

    % Calculate GEO satellite positions
    for i = 1:size(geo_satellites, 2)
        [x, y, z, lat, lon, alt, geo_satellites(i)] = satellite_position(geo_satellites(i), (step - 1) * dt, false);
        geo_position = [x, y, z];
        geo_positions(step, i, :) = [x, y, z, lat, lon, alt];
    end

    % Calculate LEO satellite positions
    for i = 1:size(leo_satellites, 2)
        [x, y, z, lat, lon, alt, leo_satellites(i)] = satellite_position(leo_satellites(i), (step - 1) * dt, false);
        leo_position = [x, y, z];
        leo_positions(step, i, :) = [x, y, z, lat, lon, alt];
    end

    % Calculate LEO satellite velocities
    for i = 1:size(leo_satellites, 2)
        [vx, vy, vz] = calculate_velocity(leo_satellites(i), (step - 1) * dt);
        leo_velocities(step, i, :) = [vx, vy, vz];
    end

    % Handover process: Determine the next LEO satellite
    [currentLEO_index, nextLEO_position, predictedHandoverPoints, ping_LEO] = ...
        handoverProcess(squeeze(geo_positions(step, 1, 1:3)), squeeze(leo_positions(step, :, 1:3)), squeeze(leo_velocities(step, :, :)), user_position, currentLEO_index, c);

    % Display timestamp and ping information
    if isempty(ping_LEO)
        ping_LEO = inf;
    end
    
    % Format time as HH:MM:SS
    hours = floor(current_time / 3600);
    minutes = floor((current_time - hours * 3600) / 60);
    seconds = current_time - hours * 3600 - minutes * 60;
    timestr = sprintf('%02d:%02d:%02d', hours, minutes, floor(seconds));
    
    if ping_LEO == inf
        disp(['Time: ' timestr ' (Step ' num2str(step) ') - No LEO coverage. Ping: inf ms']);
    else
        disp(['Time: ' timestr ' (Step ' num2str(step) ') - Connected to LEO ' num2str(currentLEO_index) ' with ping: ' num2str(ping_LEO) ' ms']);
    end

    % Update ping display in UI if it exists
    if exist('pingLabel', 'var') && isvalid(pingLabel)  % Check if pingLabel exists and is valid
        if ping_LEO == inf
            pingLabel.Text = sprintf('Time: %s - No LEO coverage. Ping: inf ms', timestr);
        else
            pingLabel.Text = sprintf('Time: %s - Connected to LEO %d, Ping: %.2f ms', timestr, currentLEO_index, ping_LEO);
        end
    end

    % Store data for this timestep
    ping_history(step) = ping_LEO;
    time_history(step) = current_time;
    leo_connection_history(step) = currentLEO_index;
end

% Save and plot the data
save_and_plot_ping_data(time_history, ping_history, leo_connection_history, dt);

% colors = hsv(size(leo_positions, 2));
% for i = 1:size(leo_positions, 2)
%     lat = rad2deg(leo_positions(:, i, 4));
%     lon = rad2deg(leo_positions(:, i, 5));
%     alt = leo_positions(:, i, 6);
%     geoplot3(g, lat, lon, alt, 'LineWidth', 2, 'Color', colors(i,:));
% end

% colors = hsv(size(geo_positions,2));
% for i = 1:size(geo_positions, 2)
%     lat = rad2deg(geo_positions(:, i, 4));
%     lon = rad2deg(geo_positions(:, i, 5));
%     alt = geo_positions(:, i, 6);
%     geoplot3(g, lat, lon, alt, 'LineWidth', 2, 'Color', colors(i,:));
% end

% % This is how to save the locations to a csv file
% writematrix(leo_positions, 'sample_satellite_location_output.csv')
% writematrix(geo_positions, 'sample_satellite_location_output.csv', 'WriteMode', 'append');

% Plot the predicted handover points on the geoglobe
% if ~isempty(predictedHandoverPoints)
%     handover_lats = rad2deg(predictedHandoverPoints(:, 2));
%     handover_lons = rad2deg(predictedHandoverPoints(:, 3));
%     handover_alts = predictedHandoverPoints(:, 4);
%     geoplot3(g, handover_lats, handover_lons, handover_alts, 'Marker', 'o', 'MarkerSize', 5, 'Color', 'green');
% end

% % This is how to generate a dummy shell with evenly spaced satellites
% dummy = leo_satellites(i);
% dummy.name = "EastLansing";
% dummies = generate_even_spaced_shell(dummy, 100, 10);

% % This is how to write an array of satellites to a file
% write_sat_array_to_csv(dummies, "sample_generated_satellite_shell.csv");