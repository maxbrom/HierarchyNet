% Satellite Handover Simulation Script with Ping Measurement

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
        disp(['Handover to LEO at time ' num2str(step * dt) ' seconds with ping: ' num2str(ping_LEO) ' ms']);
    else
        % Remain connected with GEO
        disp(['Connection with GEO maintained at time ' num2str(step * dt) ' seconds with ping: ' num2str(ping_GEO) ' ms']);
    end
end

% Extract Latitude, Longitude, and Altitude for Plotting
GEO_lat = zeros(num_steps, 1);    % Latitude remains 0 for equatorial orbit
GEO_lon = (0:(num_steps - 1))' * 0.1;  % Simple longitude progression
GEO_alt = GEO_radius * ones(num_steps, 1); % Constant altitude

LEO_lat = zeros(num_steps, 1);    % Latitude remains 0 for equatorial orbit
LEO_lon = (0:(num_steps - 1))' * 0.3;  % Simple longitude progression
LEO_alt = LEO_radius * ones(num_steps, 1); % Constant altitude

% Create a 3D Globe inside a uifigure
uif = uifigure;           % Create a UI figure for the globe
g = geoglobe(uif);        % Create the geoglobe within the UI figure

% Add a title to the UI figure using a uilabel
titleLabel = uilabel(uif, 'Text', 'Satellite Handover Simulation Between GEO and LEO Satellites', ...
    'Position', [10, uif.Position(4) - 30, 400, 30], 'FontSize', 14, 'FontWeight', 'bold');

% Plot GEO Satellite Trajectory on Geoglobe
geoplot3(g, GEO_lat, GEO_lon, GEO_alt, 'LineWidth', 2, 'Color', 'cyan');

% Plot LEO Satellite Trajectory on Geoglobe
geoplot3(g, LEO_lat, LEO_lon, LEO_alt, 'LineWidth', 2, 'Color', 'green');

% Plot Handover Points on Geoglobe
if ~isempty(handoverPoints)
    handover_lats = zeros(size(handoverPoints, 1), 1);
    handover_lons = (0:size(handoverPoints, 1) - 1)' * 0.3;  % Simple progression for handover locations
    handover_alts = LEO_radius * ones(size(handoverPoints, 1), 1);
    geoplot3(g, handover_lats, handover_lons, handover_alts, 'Marker', 'o', 'MarkerSize', 5, 'Color', 'red');
end

% Add legend using annotations
annotation(uif, 'textbox', [0.8, 0.2, 0.1, 0.1], 'String', 'GEO Trajectory', 'Color', 'cyan', 'EdgeColor', 'none');
annotation(uif, 'textbox', [0.8, 0.15, 0.1, 0.1], 'String', 'LEO Trajectory', 'Color', 'green', 'EdgeColor', 'none');
annotation(uif, 'textbox', [0.8, 0.1, 0.1, 0.1], 'String', 'Handover Points', 'Color', 'red', 'EdgeColor', 'none');
