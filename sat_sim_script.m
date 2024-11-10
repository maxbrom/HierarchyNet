% Satellite Handover Simulation Script with Ping Measurement

function [latitude, longitude, altitude] = cartesian_to_geodetic(x, y, z)
    % Define the WGS84 ellipsoid constants
    a = 6378.137; % semi-major axis in km
    f = 1 / 298.257223563; % flattening
    e2 = 2 * f - f^2; % eccentricity squared
    
    % Compute longitude
    longitude = atan2(y, x); % in radians
    
    % Initial guess for latitude
    p = sqrt(x^2 + y^2); % distance from z-axis
    theta = atan2(z * a, p * (1 - f)); % initial guess
    
    % Iterative process to find latitude (using Bowring's method or similar)
    max_iter = 100;
    tolerance = 1e-12;
    for i = 1:max_iter
        sin_theta = sin(theta);
        cos_theta = cos(theta);
        N = a / sqrt(1 - e2 * sin_theta^2); % radius of curvature in the prime vertical
        h = p / cos_theta - N; % height (altitude)
        
        % Update the latitude using the current guess
        theta_new = atan2(z + e2 * N * sin_theta, p);
        
        % Check for convergence
        if abs(theta_new - theta) < tolerance
            break;
        end
        
        % Update theta for next iteration
        theta = theta_new;
    end
    
    % Latitude in radians
    latitude = theta;
    
    % Compute altitude
    N = a / sqrt(1 - e2 * sin(latitude)^2); % radius of curvature in the prime vertical
    altitude = p / cos(latitude) - N; % in km
end

function [x, y, z] = satellite_position_with_rotation(satellite, time_offset)
    altitude = satellite.altitude;
    latitude = satellite.latitude;
    longitude = satellite.longitude;
    inclination = satellite.inclination;

    % Constants
    R_earth = 6371; % Radius of the Earth in km
    mu = 398600.4418; % Standard gravitational parameter for Earth (km^3/s^2)
    
    % Orbital Parameters
    r_orbit = R_earth + altitude; % Orbit radius in km
    period = 2 * pi * sqrt(r_orbit^3 / mu); % Orbital period in seconds
    
    % Calculate mean anomaly at t = 0 (assuming initial mean anomaly is 0)
    mean_anomaly = 2 * pi * (time_offset / period); % Mean anomaly at time_offset
    
    % Orbital inclination in radians
    inc = deg2rad(inclination);
    
    % Initial latitude and longitude in radians
    lat = deg2rad(latitude);
    lon = deg2rad(longitude);
    
    % Calculate the position in the orbital plane (x', y', z') (ignoring perturbations)
    true_anomaly = mean_anomaly; % For simplicity, assume mean anomaly ≈ true anomaly in a circular orbit
    
    % Calculate orbital coordinates in the orbital plane
    x_orbit = r_orbit * cos(true_anomaly);
    y_orbit = r_orbit * sin(true_anomaly);
    z_orbit = 0; % For a circular orbit in the equatorial plane
    
    % Rotate the orbital coordinates by the inclination
    x_rot = x_orbit;
    y_rot = y_orbit * cos(inc) - z_orbit * sin(inc);
    z_rot = y_orbit * sin(inc) + z_orbit * cos(inc);
    
    % Earth’s angular velocity (rad/s) and the rotation angle for the given time
    omega_earth = 2 * pi / 86164; % Earth’s angular velocity in radians per second
    rotation_angle = omega_earth * time_offset; % Earth’s rotation angle for the given time
    
    % Apply the Earth's rotation
    % Rotate around the z-axis to adjust the longitude and latitude as seen from the Earth
    % This rotation matrix takes the initial position (longitude = 0, latitude = 0)
    R_earth_rotation = [cos(rotation_angle), sin(rotation_angle), 0; 
                        -sin(rotation_angle), cos(rotation_angle), 0;
                         0, 0, 1];
    
    % Apply Earth's rotation to the satellite's position
    pos_rotated = R_earth_rotation * [x_rot; y_rot; z_rot];

    % Apply latitude and longitude offset
    % Latitude and Longitude to Cartesian coordinates (initial position offset)
    x_offset = (R_earth + altitude) * cos(lat) * cos(lon);
    y_offset = (R_earth + altitude) * cos(lat) * sin(lon);
    z_offset = (R_earth + altitude) * sin(lat);
    
    % Add the offsets to the rotated position to adjust for initial position
    x = pos_rotated(1) + x_offset;
    y = pos_rotated(2) + y_offset;
    z = pos_rotated(3) + z_offset;
end

% Constants
mu = 3.986e5;            % Earth's gravitational parameter (km^3/s^2)
earth_radius = 6371;     % Earth's radius in km
c = 299792.458;          % Speed of light in km/s

% GEO and LEO Parameters
GEO_altitude = 35786;       % GEO altitude in km
LEO_altitude = 500;         % LEO altitude in km
GEO_radius = earth_radius + GEO_altitude;
LEO_radius = earth_radius + LEO_altitude;

% Ground Station Position (Example at Equator)
groundStation_position = [earth_radius, 0, 0]; % on Earth's surface at equator

% Simulation Settings
dt = 10;                    % Time step in seconds
simulation_duration = 86400; % 1 day in seconds
num_steps = simulation_duration / dt;

% Initialize Positions and Velocities (simplified circular orbits)
GEO_trajectory = zeros(num_steps, 3);
LEO_trajectory = zeros(num_steps, 3);
handoverPoints = [];
pingTimes = []; % Array to store ping times at handover events

LEO_one_positions = zeros(num_steps, 3);
LEO_one_geodetic_positions = zeros(num_steps, 3);
LEO_one_satellite = HierarchySatellite(550, 53.2, 0, 90);

% Simulation Loop: Calculate GEO and LEO Positions
for step = 1:num_steps
    % GEO Satellite Position Update (circular orbit)
    theta_GEO = sqrt(mu / GEO_radius^3) * (step * dt); % Angle in radians
    GEO_position = [GEO_radius * cos(theta_GEO), GEO_radius * sin(theta_GEO), 0];
    GEO_trajectory(step, :) = GEO_position;
    
    % LEO Satellite Position Update (circular orbit)
    theta_LEO = sqrt(mu / LEO_radius^3) * (step * dt); % Angle in radians
    LEO_position = [LEO_radius * cos(theta_LEO), LEO_radius * sin(theta_LEO), 0];
    LEO_trajectory(step, :) = LEO_position;

    [x, y, z] = satellite_position_with_rotation(LEO_one_satellite, step * dt);
    LEO_one_positions(step, :) = [x, y, z];
    [lat, lon, alt] = cartesian_to_geodetic(x, y, z);
    LEO_one_geodetic_positions(step, :) = [lat, lon, alt];
    
    % Calculate distances to the ground station
    distance_GEO = norm(GEO_position - groundStation_position);
    distance_LEO = norm(LEO_position - groundStation_position);
    % Test push
    % Calculate round-trip ping times (in milliseconds)
    ping_GEO = 2 * (distance_GEO / c) * 1000; % GEO ping in ms
    ping_LEO = 2 * (distance_LEO / c) * 1000; % LEO ping in ms
    
    % Determine if a handover should occur
    if distance_LEO < distance_GEO
        % Handover to LEO
        handoverPoints = [handoverPoints; LEO_position];
        pingTimes = [pingTimes; ping_LEO]; % Store the LEO ping time at handover
        % disp(['Handover to LEO at time ' num2str(step * dt) ' seconds with ping: ' num2str(ping_LEO) ' ms']);
    else
        % Remain connected with GEO
        % disp(['Connection with GEO maintained at time ' num2str(step * dt) ' seconds with ping: ' num2str(ping_GEO) ' ms']);
    end
end

% Extract Latitude, Longitude, and Altitude for Plotting
GEO_lat = zeros(num_steps, 1);    % Latitude remains 0 for equatorial orbit
GEO_lon = (0:(num_steps - 1))' * 0.1;  % Simple longitude progression
GEO_alt = GEO_radius * ones(num_steps, 1); % Constant altitude

% LEO_lat = zeros(num_steps, 1);    % Latitude remains 0 for equatorial orbit
% LEO_lon = (0:(num_steps - 1))' * 0.3;  % Simple longitude progression
% LEO_alt = LEO_radius * ones(num_steps, 1); % Constant altitude

% Create a 3D Globe inside a uifigure
uif = uifigure;           % Create a UI figure for the globe
g = geoglobe(uif);        % Create the geoglobe within the UI figure

% Add a title to the UI figure using a uilabel
titleLabel = uilabel(uif, 'Text', 'Satellite Handover Simulation Between GEO and LEO Satellites', ...
    'Position', [10, uif.Position(4) - 30, 400, 30], 'FontSize', 14, 'FontWeight', 'bold');

% % Plot GEO Satellite Trajectory on Geoglobe
% geoplot3(g, GEO_lat, GEO_lon, GEO_alt, 'LineWidth', 2, 'Color', 'cyan');

% % Plot LEO Satellite Trajectory on Geoglobe
% geoplot3(g, LEO_lat, LEO_lon, LEO_alt, 'LineWidth', 2, 'Color', 'green');

LEO_lat = rad2deg(LEO_one_geodetic_positions(:, 1));
LEO_lon = rad2deg(LEO_one_geodetic_positions(:, 2));
LEO_alt = LEO_one_geodetic_positions(:, 3);

disp(LEO_one_positions)
disp([LEO_lat, LEO_lon, LEO_alt])

geoplot3(g, LEO_lat, LEO_lon, LEO_alt, 'LineWidth', 2, 'Color', 'cyan');

% Plot Handover Points on Geoglobe
if ~isempty(handoverPoints)
    handover_lats = zeros(size(handoverPoints, 1), 1);
    handover_lons = (0:size(handoverPoints, 1) - 1)' * 0.3;  % Simple progression for handover locations
    handover_alts = LEO_radius * ones(size(handoverPoints, 1), 1);
    % geoplot3(g, handover_lats, handover_lons, handover_alts, 'Marker', 'o', 'MarkerSize', 5, 'Color', 'red');
end

% Add legend using annotations
annotation(uif, 'textbox', [0.8, 0.2, 0.1, 0.1], 'String', 'GEO Trajectory', 'Color', 'cyan', 'EdgeColor', 'none');
annotation(uif, 'textbox', [0.8, 0.15, 0.1, 0.1], 'String', 'LEO Trajectory', 'Color', 'green', 'EdgeColor', 'none');
annotation(uif, 'textbox', [0.8, 0.1, 0.1, 0.1], 'String', 'Handover Points', 'Color', 'red', 'EdgeColor', 'none');
