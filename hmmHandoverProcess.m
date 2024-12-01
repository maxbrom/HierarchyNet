function [currentLEO_index, bestLEO_position, predictedHandoverPoints, ping_LEO] = hmmHandoverProcess(GEO_position, LEO_positions, LEO_velocities, user_position, currentLEO_index, c, network)
    % Initialize HMM parameters
    num_states = size(LEO_positions, 1);
    
    % If currentLEO_index is not set (first run), choose closest LEO
    if isempty(currentLEO_index) || currentLEO_index == 0
        [~, currentLEO_index] = min(vecnorm(LEO_positions - user_position, 2, 2));
    end
    
    % Define transition matrix with stronger preference for closer satellites
    A = calculateTransitionMatrix(LEO_positions, LEO_velocities, user_position);
    
    % Define emission probabilities (based on signal quality/distance)
    B = calculateEmissionProbabilities(LEO_positions, user_position, GEO_position);
    
    % Initial state distribution (based on current LEO)
    pi = zeros(num_states, 1);
    % Ensure currentLEO_index is valid
    currentLEO_index = max(1, min(num_states, round(currentLEO_index)));
    pi(currentLEO_index) = 1;
    
    % Use Viterbi algorithm to find most likely sequence of states
    observations = getObservationSequence(user_position, LEO_positions, GEO_position);
    best_path = viterbi(A, B, pi, observations);
    
    % Get the predicted next state
    currentLEO_index = best_path(end);
    bestLEO_position = LEO_positions(currentLEO_index, :);
    
    % Calculate predicted handover points and ping
    predictedHandoverPoints = predictHandoverPoints(best_path, LEO_positions);
    ping_LEO = calculatePing(bestLEO_position, user_position, c, network);
end

function A = calculateTransitionMatrix(LEO_positions, LEO_velocities, user_position)
    num_states = size(LEO_positions, 1);
    A = zeros(num_states);
    
    for i = 1:num_states
        for j = 1:num_states
            % Calculate probability of transition based on:
            % 1. Distance between satellites
            sat_distance = norm(LEO_positions(i,:) - LEO_positions(j,:));
            % 2. Distance to user
            user_distance_j = norm(LEO_positions(j,:) - user_position);
            % 3. Relative velocity
            relative_velocity = norm(LEO_velocities(i,:) - LEO_velocities(j,:));
            
            % Higher probability for:
            % - Closer satellites
            % - Satellites closer to user
            % - Satellites with similar velocities
            A(i,j) = 1 / (1 + sat_distance * 0.3 + user_distance_j * 0.7 + relative_velocity * 0.2);
        end
        % Normalize rows
        A(i,:) = A(i,:) / sum(A(i,:));
    end
end

function B = calculateEmissionProbabilities(LEO_positions, user_position, GEO_position)
    num_states = size(LEO_positions, 1);
    num_observations = 10; % Define number of discrete observation levels
    B = zeros(num_states, num_observations);
    
    for i = 1:num_states
        for j = 1:num_observations
            % Calculate emission probabilities based on signal quality
            signal_quality = calculateSignalQuality(LEO_positions(i,:), user_position, GEO_position);
            B(i,j) = gaussian(signal_quality, j, 1); % Use Gaussian distribution
        end
        % Normalize
        B(i,:) = B(i,:) / sum(B(i,:));
    end
end

function signal_quality = calculateSignalQuality(leo_pos, user_pos, geo_pos)
    % Calculate signal quality based on distances and angles
    leo_user_distance = norm(leo_pos - user_pos);
    geo_user_distance = norm(geo_pos - user_pos);
    
    % Normalize distances and combine factors
    signal_quality = 1 / (1 + leo_user_distance/geo_user_distance);
end

function y = gaussian(x, mu, sigma)
    y = exp(-(x-mu)^2 / (2*sigma^2)) / (sigma * sqrt(2*pi));
end

function observations = getObservationSequence(user_position, LEO_positions, GEO_position)
    % Generate observation sequence based on signal quality
    num_observations = 10; % Match this with B matrix
    num_steps = 5; % Number of time steps to predict
    
    observations = zeros(1, num_steps);
    for t = 1:num_steps
        % Calculate signal quality
        signal_quality = calculateSignalQuality(LEO_positions(1,:), user_position, GEO_position);
        
        % Discretize signal quality into observation states (1 to num_observations)
        obs_state = max(1, min(num_observations, ceil(signal_quality * num_observations)));
        observations(t) = obs_state;
    end
end

function best_path = viterbi(A, B, pi, observations)
    % Implementation of Viterbi algorithm for HMM
    % A: Transition matrix
    % B: Emission matrix
    % pi: Initial state distribution
    % observations: Sequence of observations
    
    num_states = size(A, 1);
    T = length(observations);
    
    % Initialize matrices
    delta = zeros(num_states, T);
    psi = zeros(num_states, T);
    
    % Initialization step
    delta(:,1) = pi .* B(:,observations(1));
    psi(:,1) = 0;
    
    % Recursion step
    for t = 2:T
        for j = 1:num_states
            [delta(j,t), psi(j,t)] = max(delta(:,t-1) .* A(:,j) .* B(j,observations(t)));
        end
    end
    
    % Termination step
    best_path = zeros(1, T);
    [~, best_path(T)] = max(delta(:,T));
    
    % Path backtracking
    for t = T-1:-1:1
        best_path(t) = psi(best_path(t+1), t+1);
    end
end

function handoverPoints = predictHandoverPoints(best_path, LEO_positions)
    % Find points where the path changes from one satellite to another
    % These represent predicted handover locations
    
    % Initialize empty array for handover points
    handoverPoints = [];
    
    % Loop through the path to find state changes
    for i = 1:(length(best_path)-1)
        if best_path(i) ~= best_path(i+1)
            % When state changes, store the position of the new satellite
            handoverPoints = [handoverPoints; LEO_positions(best_path(i+1), :)];
        end
    end
    
    % If no handovers predicted, return empty matrix
    if isempty(handoverPoints)
        handoverPoints = zeros(0, size(LEO_positions, 2));
    end
end

function ping = calculatePing(leo_position, user_position, c, network)
    % Calculate distance and base propagation time
    distance = norm(leo_position - user_position);
    one_way_propagation = (distance / c) * 1000; % Convert to ms

    % Calculate base propagation time
    base_propagation = one_way_propagation * network.fixed.atmospheric_factor;
    
    % Calculate fixed delays
    fixed_delays = network.fixed.processing_delay + ...     % Satellite processing
                  network.fixed.equipment_delay + ...       % Network equipment
                  network.fixed.ground_station_delay;       % Ground station processing
    
    % Calculate round trip time for fixed components
    round_trip_base = 2 * (base_propagation + fixed_delays);
    
    % Add variable delays (only once, not doubled)
    ping = round_trip_base + ...
           network.variable.network_jitter + ...        % Add jitter
           network.variable.packet_loss_delay + ...     % Potential packet loss delay
           network.variable.final_variation;            % Final variation
end 