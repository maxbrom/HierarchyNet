function [currentLEO_index, bestLEO_position, predictedHandoverPoints, ping_LEO] = handoverProcess(GEO_position, LEO_positions, LEO_velocities, user_position, currentLEO_index, c, use_kalman)
    % Added parameter use_kalman to switch between methods
    % use_kalman: boolean to determine whether to use Kalman filtering (true) or original method (false)
    
    if use_kalman
        [currentLEO_index, bestLEO_position, predictedHandoverPoints, ping_LEO] = kalmanHandoverProcess(GEO_position, LEO_positions, LEO_velocities, user_position, currentLEO_index, c);
    else
        % First determine the best LEO and future handover sequence
        LEO_data = collectLEOData(LEO_positions, LEO_velocities);
        G = createUserGraph(LEO_data);
        bestLEO = UserDecisionMaking(G, user_position, currentLEO_index);

        % Initialize outputs
        bestLEO_position = [];
        predictedHandoverPoints = [];
        ping_LEO = [];

        % Check if we have LEO coverage first
        if bestLEO == 0
            disp('Warning: Ground station is currently out of coverage from any LEO satellite');
            currentLEO_index = 0;
            ping_LEO = inf;
            return;
        end

        % Update the next LEO position
        bestLEO_position = LEO_positions(bestLEO, :);

        % Calculate system overhead (this is done once for planning multiple handovers)
        % but doesn't affect individual ping measurements
        leo_to_geo_delays = zeros(size(LEO_positions, 1), 1);
        for i = 1:size(LEO_positions, 1)
            distance_leo_geo = norm(LEO_positions(i,:) - GEO_position);
            propagation_delay = (distance_leo_geo / c) * 1000; % ms
            processing_delay = 5; % ms
            queuing_delay = rand() * 2; % ms
            leo_to_geo_delays(i) = propagation_delay + processing_delay + queuing_delay;
        end
        
        total_planning_overhead = max(leo_to_geo_delays) + ... % Time for all LEOs to report to GEO
                                 10 + ...                      % GEO processing time
                                 (norm(GEO_position - user_position) / c) * 1000 + ... % GEO to ground propagation
                                 8;                           % Ground station processing
        
        % Calculate actual ping to current LEO (this is what the user experiences)
        distance_to_leo = norm(bestLEO_position - user_position);
        
        % Add realistic delays for LEO communication:
        leo_processing_delay = 5; % Processing delay at LEO (ms)
        leo_propagation_delay = (distance_to_leo / c) * 1000; % Propagation delay (ms)
        leo_queuing_delay = rand() * 2; % Random queuing delay 0-2ms
        transmission_delay = 1; % Transmission delay for packet size (ms)
        
        % Calculate round-trip time for current LEO connection
        leo_round_trip = 2 * (leo_propagation_delay + leo_processing_delay + ...
                             leo_queuing_delay + transmission_delay);
        
        % Add atmospheric effects
        atmospheric_loss = rand() * 3; % Random atmospheric delay 0-3ms
        
        % Add potential interference delay
        interference_delay = 0;
        if rand() < 0.1 % 10% chance of interference
            interference_delay = rand() * 5; % 0-5ms additional delay
        end
        
        % Total ping is just the current LEO connection time
        ping_LEO = leo_round_trip + atmospheric_loss + interference_delay;

        % Check if a handover is needed
        if bestLEO ~= currentLEO_index
            disp(['Predicted handover to LEO ' num2str(bestLEO) ' with ping: ' num2str(ping_LEO) ' ms.']);
            disp(['Handover planning overhead: ' num2str(total_planning_overhead) ' ms']);
            currentLEO_index = bestLEO;
        else
            disp(['Maintaining connection to LEO ' num2str(bestLEO) ' with ping: ' num2str(ping_LEO) ' ms']);
        end
    end
end

function [currentLEO_index, bestLEO_position, predictedHandoverPoints, ping_LEO] = kalmanHandoverProcess(GEO_position, LEO_positions, LEO_velocities, user_position, currentLEO_index, c)
    % Initialize Kalman Filter parameters
    persistent kalman_states P_matrices
    if isempty(kalman_states)
        kalman_states = initializeKalmanStates(LEO_positions, LEO_velocities);
        P_matrices = initializeErrorCovariance(size(LEO_positions, 1));
    end
    
    % Process measurement updates
    for i = 1:size(LEO_positions, 1)
        [kalman_states(:,i), P_matrices{i}] = updateKalmanFilter(kalman_states(:,i), P_matrices{i}, LEO_positions(i,:), LEO_velocities(i,:));
    end
    
    % Predict future positions
    prediction_horizon = 10; % seconds
    predicted_positions = predictSatellitePositions(kalman_states, prediction_horizon);
    
    % Make handover decision based on Kalman predictions
    [bestLEO, bestLEO_position, predictedHandoverPoints] = makeKalmanHandoverDecision(predicted_positions, user_position, currentLEO_index);
    
    % Calculate ping if we have a valid connection
    if bestLEO == 0
        currentLEO_index = 0;
        ping_LEO = inf;
    else
        currentLEO_index = bestLEO;
        distance_to_leo = norm(bestLEO_position - user_position);
        
        % Add realistic delays for LEO communication:
        leo_processing_delay = 5; % Processing delay at LEO (ms)
        leo_propagation_delay = (distance_to_leo / c) * 1000; % Propagation delay (ms)
        leo_queuing_delay = rand() * 2; % Random queuing delay 0-2ms
        transmission_delay = 1; % Transmission delay for packet size (ms)
        
        % Calculate round-trip time for current LEO connection
        leo_round_trip = 2 * (leo_propagation_delay + leo_processing_delay + ...
                             leo_queuing_delay + transmission_delay);
        
        % Add atmospheric effects
        atmospheric_loss = rand() * 3; % Random atmospheric delay 0-3ms
        
        % Add potential interference delay
        interference_delay = 0;
        if rand() < 0.1 % 10% chance of interference
            interference_delay = rand() * 5; % 0-5ms additional delay
        end
        
        % Total ping is just the current LEO connection time
        ping_LEO = leo_round_trip + atmospheric_loss + interference_delay;
    end
end

function kalman_states = initializeKalmanStates(positions, velocities)
    % Initialize state vector for each satellite
    % State vector: [x, y, z, vx, vy, vz]'
    num_satellites = size(positions, 1);
    kalman_states = zeros(6, num_satellites);
    
    for i = 1:num_satellites
        kalman_states(:,i) = [positions(i,:)'; velocities(i,:)'];
    end
end


function P_matrices = initializeErrorCovariance(num_satellites)
    % Initialize error covariance matrices
    P_matrices = cell(1, num_satellites);
    for i = 1:num_satellites
        P_matrices{i} = eye(6) * 1000; % Initial uncertainty
    end
end

function [updated_state, updated_P] = updateKalmanFilter(state, P, position_measurement, velocity_measurement)
    % System matrices
    dt = 0.1; % Time step
    F = [1 0 0 dt 0 0;
         0 1 0 0 dt 0;
         0 0 1 0 0 dt;
         0 0 0 1 0 0;
         0 0 0 0 1 0;
         0 0 0 0 0 1];
    
    H = eye(6); % Measurement matrix
    
    % Process and measurement noise
    Q = eye(6) * 0.1;
    R = eye(6) * 1;
    
    % Prediction step
    predicted_state = F * state;
    predicted_P = F * P * F' + Q;
    
    % Measurement
    measurement = [position_measurement'; velocity_measurement'];
    
    % Update step
    K = predicted_P * H' / (H * predicted_P * H' + R);
    updated_state = predicted_state + K * (measurement - H * predicted_state);
    updated_P = (eye(6) - K * H) * predicted_P;
end

function predicted_positions = predictSatellitePositions(kalman_states, horizon)
    num_satellites = size(kalman_states, 2);
    num_steps = ceil(horizon / 0.1);
    predicted_positions = zeros(num_satellites, 3, num_steps);
    
    for i = 1:num_satellites
        state = kalman_states(:,i);
        for j = 1:num_steps
            t = j * 0.1;
            pos = state(1:3) + state(4:6) * t;
            predicted_positions(i,:,j) = pos';
        end
    end
end

function [bestLEO, bestLEO_position, predictedHandoverPoints] = makeKalmanHandoverDecision(predicted_positions, user_position, currentLEO)
    num_satellites = size(predicted_positions, 1);
    num_predictions = size(predicted_positions, 3);
    
    % Calculate coverage scores for each satellite over the prediction horizon
    coverage_scores = zeros(1, num_satellites);
    for i = 1:num_satellites
        score = 0;
        for j = 1:num_predictions
            pos = squeeze(predicted_positions(i,:,j));
            distance = norm(pos - user_position);
            score = score + 1/distance;
        end
        coverage_scores(i) = score;
    end
    
    % Add hysteresis for current connection
    if currentLEO > 0
        coverage_scores(currentLEO) = coverage_scores(currentLEO) * 1.1;
    end
    
    % Find best satellite
    [~, bestLEO] = max(coverage_scores);
    bestLEO_position = squeeze(predicted_positions(bestLEO,:,1))';
    
    % Store handover points for visualization
    predictedHandoverPoints = squeeze(predicted_positions(bestLEO,:,:))';
end

function LEO_data = collectLEOData(LEO_positions, LEO_velocities)
    % New function: Ground station collects additional data directly from LEOs
    LEO_data = struct('positions', LEO_positions, 'velocities', LEO_velocities);
    
    % Simulate collecting data directly from LEOs
    for i = 1:size(LEO_positions, 1)
        LEO_data.signal_strength{i} = getLEOSignalStrength(i);  % Get from LEO
    end
end

function G = createUserGraph(LEO_data)
    % Initialize empty graph
    G = digraph();
    
    % Initialize node table with LEO properties
    NodeProperties = table('Size', [size(LEO_data.positions, 1), 5], 'VariableTypes', {'cell', 'cell', 'double', 'double', 'string'}, 'VariableNames', {'Position', 'Velocity', 'SignalStrength', 'CoverageRadius', 'Name'});
    for i = 1:size(LEO_data.positions, 1)
        % Extract LEO properties
        pos = LEO_data.positions(i, :);
        vel = LEO_data.velocities(i, :);
        signal = LEO_data.signal_strength{i};
        coverage_radius = 1000; % Set appropriate coverage radius

        NodeProperties.Position{i} = pos;
        NodeProperties.Velocity{i} = vel;
        NodeProperties.SignalStrength(i) = signal;
        NodeProperties.CoverageRadius(i) = coverage_radius;
        NodeProperties.Name{i} = sprintf('LEO_%d', i);
    end
    
    % Add nodes to graph
    G = addnode(G, NodeProperties);
    
    % Create edges between overlapping LEOs
    for i = 1:height(NodeProperties)
        for j = 1:height(NodeProperties)
            if i ~= j
                if coverage_overlap_new(NodeProperties, i, j)
                    weight = calculate_handover_weight_new(NodeProperties, i, j);
                    G = addedge(G, i, j, weight);
                end
            end
        end
    end
end

function weight = calculate_handover_weight_new(NodeProperties, i, j)
    pos1 = NodeProperties.Position{i};
    pos2 = NodeProperties.Position{j};
    vel1 = NodeProperties.Velocity{i};
    vel2 = NodeProperties.Velocity{j};
    signal1 = NodeProperties.SignalStrength(i);
    signal2 = NodeProperties.SignalStrength(j);
    
    dist = norm(pos1 - pos2);
    rel_vel = norm(vel1 - vel2);
    avg_signal = (signal1 + signal2) / 2;
    
    % Updated weight calculation including signal strength
    weight = 0.4 * (1/dist) + 0.3 * (1/rel_vel) + 0.3 * avg_signal;
end

function signal = getLEOSignalStrength(leo_index)
    % Placeholder function - in reality, this would get real signal strength from LEO
    signal = rand() * 100;  % Dummy value between 0 and 100
end

% Updated helper functions for the new graph structure
function overlap = coverage_overlap_new(NodeProperties, i, j)
    pos1 = NodeProperties.Position{i};
    pos2 = NodeProperties.Position{j};
    rad1 = NodeProperties.CoverageRadius(i);
    rad2 = NodeProperties.CoverageRadius(j);
    
    dist = norm(pos1 - pos2);
    overlap = dist <= (rad1 + rad2);
end

% User decision making function
function bestLEO = UserDecisionMaking(G, user_position, currentLEO)
    % If there's no current connection (currentLEO = 0), search for initial connection
    if currentLEO == 0
        all_leos = 1:height(G.Nodes);
        bestLEO = findBestAvailableLEO(G, user_position, all_leos);
        return;
    end

    % Rest of the function for when we have a current connection
    current_leo_data = G.Nodes(currentLEO, :);
    current_position = current_leo_data.Position{1};
    current_coverage_radius = current_leo_data.CoverageRadius(1);
    
    % Calculate distance to current LEO
    current_surface_distance = calculate_surface_distance(current_position, user_position);
    
    % Check if user is outside current LEO's coverage
    if current_surface_distance > current_coverage_radius
        % User is outside coverage, search for any LEO that provides coverage
        all_leos = 1:height(G.Nodes);
        bestLEO = findBestAvailableLEO(G, user_position, all_leos);
        
        if bestLEO == 0  % No LEO provides coverage
            warning('User is currently out of coverage from any LEO satellite');
            bestLEO = 0;  % Return 0 to indicate no coverage
            return;
        end
        
        return;
    end
    
    % Rest of the existing handover logic for when user is within coverage
    handover_threshold = 0.8 * current_coverage_radius;
    
    if current_surface_distance < handover_threshold
        bestLEO = currentLEO;  % Stay with current LEO
        return;
    end

    % TODO: Must also consider which direction the coverage area is moving
    % If we have just entered the coverage area, we ideally don't recalculate everything
    
    % If we're here, user is near edge of coverage, so consider handover
    handoverCandidates = successors(G, currentLEO);
    
    % If no candidates available, stay with current LEO
    if isempty(handoverCandidates)
        bestLEO = currentLEO;
        return;
    end
    
    % Initialize best LEO search
    bestLEO = currentLEO;
    max_score = calculate_handover_score(G, currentLEO, user_position);
    
    % Evaluate each candidate
    for i = 1:length(handoverCandidates)
        candidate = handoverCandidates(i);
        candidate_data = G.Nodes(candidate, :);
        candidate_position = candidate_data.Position{1};
        
        % Calculate distance to candidate
        distance_to_candidate = norm(candidate_position - user_position);
        
        % Only consider candidates that provide good coverage
        if distance_to_candidate < 0.7 * candidate_data.CoverageRadius(1)  % Ensure we're well within new coverage
            score = calculate_handover_score(G, candidate, user_position);
            
            % Only change if the candidate is significantly better
            if score > max_score * 1.1  % 10% improvement threshold
                max_score = score;
                bestLEO = candidate;
            end
        end
    end
    
    % If we're about to leave coverage and no good candidate found, pick the best available
    if bestLEO == currentLEO && current_surface_distance > 0.95 * current_coverage_radius
        for i = 1:length(handoverCandidates)
            candidate = handoverCandidates(i);
            score = calculate_handover_score(G, candidate, user_position);
            if score > max_score
                max_score = score;
                bestLEO = candidate;
            end
        end
    end
end

function currentLEO = IdentifyCurrentLEO(G, user_position)
    % Get all nodes (LEOs) from the graph
    LEOs = G.Nodes;
    
    % Find LEO with strongest connection to user
    best_signal = -inf;
    currentLEO = 1;  % Default to first LEO
    
    for i = 1:height(LEOs)
        leo_position = LEOs.Position{i};
        signal_strength = calculate_signal_strength(leo_position, user_position);
        
        if signal_strength > best_signal
            best_signal = signal_strength;
            currentLEO = i;
        end
    end
end

function score = calculate_handover_score(G, candidate, user_position)
    % Get candidate LEO properties from graph
    leo_data = G.Nodes(candidate, :);
    
    % Calculate various factors
    distance = norm(leo_data.Position{1} - user_position);
    signal_strength = calculate_signal_strength(leo_data.Position{1}, user_position);
    velocity_factor = calculate_velocity_factor(leo_data.Position{1}, leo_data.Velocity{1}, leo_data.CoverageRadius, user_position);
    
    % Weighted scoring (adjust weights based on importance)
    w1 = 0.1;  % Distance weight
    w2 = 0.1;  % Signal strength weight
    w3 = 0.8;  % Velocity factor weight
    
    score = w1 * (-distance) + w2 * signal_strength + w3 * velocity_factor;
end

function signal_strength = calculate_signal_strength(leo_position, user_position)
    % Calculate signal strength based on distance and other factors
    distance = norm(leo_position - user_position);
    
    % Simple path loss model (can be replaced with more complex models)
    frequency = 12e9;  % Example frequency (12 GHz)
    c = 3e8;  % Speed of light
    lambda = c/frequency;
    
    % Free space path loss
    path_loss = (lambda/(4*pi*distance))^2;
    signal_strength = 10*log10(path_loss);
end

function velocity_factor = calculate_velocity_factor(leo_position, leo_velocity, coverageRadius, user_position)
    % Calculate how favorable the LEOs velocity is relative to user
    % Higher score means LEO is moving in a way that will maintain good coverage
    omega_earth = 7.2921159e-5 * [0 0 1];
    velocity_user = cross(omega_earth, user_position);
    velocity_relative = leo_velocity - velocity_user;

    d_surface = calculate_surface_distance(leo_position, user_position);

    % Step 2: Check if the user is within the coverage area
    if d_surface > coverageRadius
        disp('User is outside the coverage area.');
        velocity_factor = 0; % The user is outside the coverage area
        return;
    end

    v_rel = velocity_relative;

    % Step 4: Compute the relative velocity along the line of sight
    % Line of sight vector from the user to the satellite
    d_vec = leo_position - user_position;
    d_mag = norm(d_vec); % Magnitude of the distance vector
    v_rel_line = dot(v_rel, d_vec) / d_mag; % Component of velocity along line of sight

    % Step 5: Calculate the time the user is within the coverage area
    t_coverage = 2 * coverageRadius / abs(v_rel_line); % Time in seconds
    velocity_factor = t_coverage;  % Slower relative motion is better
end

function ExecuteHandover(bestLEO)
    try
        % Log handover attempt
        disp(['Executing handover to LEO_' num2str(bestLEO)]);
        
        % Here you would add your actual handover implementation
        % Example: initiate_handover_sequence(bestLEO);
        
    catch exception
        warning(exception.identifier, '%s', exception.message);
    end
end

% New helper function to find the best available LEO
function bestLEO = findBestAvailableLEO(G, user_position, candidates)
    bestLEO = 0;
    best_score = -inf;
    
    for i = 1:size(candidates,2)
        leo_data = G.Nodes(i, :);
        leo_position = leo_data.Position{1};
        coverage_radius = leo_data.CoverageRadius(1);

        d_surface = calculate_surface_distance(leo_position, user_position);
        if d_surface <= coverage_radius
            score = calculate_handover_score(G, i, user_position);
            if score > best_score
                best_score = score;
                bestLEO = i;
            end
        end
    end
end


function surface_distance = calculate_surface_distance(leo_position, user_position)
    % Earth's radius (mean radius in km)
    R_earth = 6371;

    % Step 1: Normalize the satellite and user positions to get their unit vectors
    unit_r_sat = leo_position / norm(leo_position);   % Unit vector from the Earth's center to the satellite
    unit_r_user = user_position / norm(user_position); % Unit vector from the Earth's center to the user

    % Step 2: Compute the horizontal (ground-level) distance between the satellite and the user
    % Project both the satellite and user onto the Earth's surface (ignore the z-coordinate)
    r_sat_surface = unit_r_sat * R_earth;  % Project the satellite to the Earth's surface
    r_user_surface = unit_r_user * R_earth; % Project the user to the Earth's surface

    % Step 3: Calculate the horizontal distance on the ground
    surface_distance = norm(r_sat_surface(1:2) - r_user_surface(1:2));  % Only consider x and y coordinates
end