function [currentLEO_index, bestLEO_position, predictedHandoverPoints, ping_LEO] = kalmanHandoverProcess(GEO_position, LEO_positions, LEO_velocities, user_position, currentLEO_index, c, network)
    % Initialize Kalman Filter parameters
    persistent kalman_states P_matrices
    if isempty(kalman_states)
        fprintf('Initializing Kalman Filter\n');
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
        distance_to_leo = norm(bestLEO_position - user_position');
        
        % Basic propagation time
        one_way_propagation = (distance_to_leo / c) * 1000; % Convert to ms
        
        % Calculate base propagation time
        base_propagation = one_way_propagation * network.fixed.atmospheric_factor;
        
        % Calculate fixed delays
        fixed_delays = network.fixed.processing_delay + ...     % Satellite processing
                      network.fixed.equipment_delay + ...       % Network equipment
                      network.fixed.ground_station_delay;       % Ground station processing
        
        % Calculate round trip time for fixed components
        round_trip_base = 2 * (base_propagation + fixed_delays);
        
        % Add variable delays (only once, not doubled)
        ping_LEO = round_trip_base + ...
                  network.variable.network_jitter + ...        % Add jitter
                  network.variable.packet_loss_delay + ...     % Potential packet loss delay
                  network.variable.final_variation;            % Final variation
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
    
    % Reduce process noise for more stable predictions
    Q = eye(6) * 0.01;  % Reduced from 0.1
    
    % Increase measurement noise for position components
    R = diag([10, 10, 10, 1, 1, 1]);  % More trust in measurements
    
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
    
    % Calculate current distances to all satellites
    current_distances = zeros(1, num_satellites);
    for i = 1:num_satellites
        current_pos = squeeze(predicted_positions(i,:,1));
        current_distances(i) = norm(current_pos - user_position);
        
        % % Debug print each satellite's position and distance
        % fprintf('Satellite %d:\n', i);
        % fprintf('  Position: [%.2f, %.2f, %.2f] km\n', current_pos(1), current_pos(2), current_pos(3));
        % fprintf('  Distance to user: %.2f km\n', current_distances(i));
    end
    
    % Simply choose the closest satellite
    [min_distance, bestLEO] = min(current_distances);
    
    % Set the best position and predicted points
    bestLEO_position = squeeze(predicted_positions(bestLEO,:,1))';
    predictedHandoverPoints = squeeze(predicted_positions(bestLEO,:,:))';
end
