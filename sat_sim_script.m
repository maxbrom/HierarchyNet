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

    % Constants
    R_earth = 6371;  % Radius of the Earth in km
    mu = 398600.4418; % Standard gravitational parameter for Earth (km^3/s^2)

    lat_target_rad = deg2rad(satellite.latitude);

    altitude = satellite.altitude; % Altitude in km
    inclination = deg2rad(satellite.inclination); % Inclination in radians
    a = R_earth + altitude;  % Semi-major axis (in km)
    T = 2 * pi * sqrt(a^3 / mu); % Orbital period (in seconds)
    theta_target = asin(sin(lat_target_rad) / sin(inclination));
    
    % Solve for time t when the satellite reaches theta_target
    satellite.time_to_latitude = (theta_target / (2 * pi)) * T;
end

function [X, Y, Z] = geodetic_to_cartesian(lat, lon, alt)
    % WGS84 Ellipsoid parameters
    a = 6378.137;  % semi-major axis in kilometers
    e = 0.081819190842622;  % eccentricity of the WGS84 ellipsoid
    
    % Radius of curvature in the prime vertical (N)
    N = a / sqrt(1 - e^2 * sin(lat)^2);
    
    % Cartesian coordinates (X, Y, Z) in kilometers
    X = (N + alt) * cos(lat) * cos(lon);
    Y = (N + alt) * cos(lat) * sin(lon);
    Z = ((1 - e^2) * N + alt) * sin(lat);
end

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
    
    % Iterative process to find latitude (Bowring/Newton-Raphson method)
    max_iter = 100;
    tolerance = 1e-12;
    for i = 1:max_iter
        sin_theta = sin(theta);
        N = a / sqrt(1 - e2 * sin_theta^2); % radius of curvature in the prime vertical
        
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

function [x, y, z, satellite] = satellite_position_cartesian(satellite, time_offset, finding_offset)
    % Constants
    R_earth = 6371; % Radius of the Earth in km
    mu = 398600.4418; % Standard gravitational parameter for Earth (km^3/s^2)
    
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

    % Orbital Parameters
    altitude = satellite.altitude;
    inclination = deg2rad(satellite.inclination);
    
    % Orbit radius (in km)
    r_orbit = R_earth + altitude; % Orbit radius in km
    period = 2 * pi * sqrt(r_orbit^3 / mu); % Orbital period in seconds
    
    % Calculate mean anomaly at time_offset (assume initial mean anomaly is 0)
    mean_anomaly = 2 * pi * (time_offset / period); % Mean anomaly at time_offset
    
    % Calculate the position in the orbital plane (x', y', z') (ignoring perturbations)
    true_anomaly = mean_anomaly; % For simplicity, assume mean anomaly ≈ true anomaly for circular orbit
    
    % Calculate orbital coordinates in the orbital plane
    x_orbit = r_orbit * cos(true_anomaly);
    y_orbit = r_orbit * sin(true_anomaly);
    z_orbit = 0; % For a circular orbit in the equatorial plane
    
    % Apply the orbital inclination
    x_rot = x_orbit;
    y_rot = y_orbit * cos(inclination) - z_orbit * sin(inclination);
    z_rot = y_orbit * sin(inclination) + z_orbit * cos(inclination);
    
    % Earth's angular velocity (rad/s) and the rotation angle for the given time
    omega_earth = 2 * pi / 86164; % Earth’s angular velocity in radians per second (sidereal day)
    rotation_angle = omega_earth * time_offset; % Earth's rotation angle for the given time
    
    % Rotate the satellite position by the Earth's rotation
    R_earth_rotation = [cos(rotation_angle), sin(rotation_angle), 0; 
                        -sin(rotation_angle), cos(rotation_angle), 0;
                         0, 0, 1];
    
    % Apply Earth's rotation to the satellite's position
    pos_rotated = R_earth_rotation * [x_rot; y_rot; z_rot];

    % Add the orbital position to the offset to get the final position
    x = pos_rotated(1);
    y = pos_rotated(2);
    z = pos_rotated(3);

    % Offset to desired initial longitude
    % Initial latitude has to be offset by modifying the time offset strategically
    [lat, lon, alt] = cartesian_to_geodetic(x, y, z);
    lon = lon + longitude;
    [x, y, z] = geodetic_to_cartesian(lat, lon, alt);
end

function [x, y, z, lat, lon, alt, satellite] = satellite_position(satellite, time_offset, finding_offset)
    % Pass "false" to the finding_offset parameter when calling this from anywhere except the find_lon_offset function
    [x, y, z, satellite] = satellite_position_cartesian(satellite, time_offset, finding_offset);
    [lat, lon, alt] = cartesian_to_geodetic(x, y, z);
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

GEO_one_positions = zeros(num_steps, 6);
GEO_one_satellite = HierarchySatellite(GEO_altitude, 0, 0, -90);

LEO_one_positions = zeros(num_steps, 6);
LEO_one_satellite = HierarchySatellite(550, 43, 42.737652, -84.48378); % Offset to East Lansing

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

    [x, y, z, lat, lon, alt, LEO_one_satellite] = satellite_position(LEO_one_satellite, (step - 1) * dt, false);
    LEO_one_positions(step, :) = [x, y, z, lat, lon, alt];

    [x, y, z, lat, lon, alt, GEO_one_satellite] = satellite_position(GEO_one_satellite, (step - 1) * dt, false);
    GEO_one_positions(step, :) = [x, y, z, lat, lon, alt];
    
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
% GEO_lat = zeros(num_steps, 1);    % Latitude remains 0 for equatorial orbit
% GEO_lon = (0:(num_steps - 1))' * 0.1;  % Simple longitude progression
% GEO_alt = GEO_radius * ones(num_steps, 1); % Constant altitude

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

LEO_lat = rad2deg(LEO_one_positions(:, 4));
LEO_lon = rad2deg(LEO_one_positions(:, 5));
LEO_alt = LEO_one_positions(:, 6);

GEO_lat = rad2deg(GEO_one_positions(:, 4));
GEO_lon = rad2deg(GEO_one_positions(:, 5));
GEO_alt = GEO_one_positions(:, 6);

geoplot3(g, LEO_lat, LEO_lon, LEO_alt, 'LineWidth', 2, 'Color', 'cyan');
geoplot3(g, GEO_lat, GEO_lon, GEO_alt, 'LineWidth', 2, 'Color', 'red');

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
