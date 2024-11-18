function [currentLEO_index, nextLEO_position, predictedHandoverPoints, ping_LEO] = handoverProcess(GEO_position, LEO_positions, groundStation_position, currentLEO_index, c)
    % Function modified: GEO only collects position data
    % Velocities and signal strength are communicated directly from LEOs to users
    
    % Step 1: GEO collects only LEO positions
    disp('GEO satellite collected LEO position data.');

    % Step 2: GEO forwards position data to the ground station

    distance_to_ground = norm(GEO_position - groundStation_position);

    % disp(['Distance to ground station: ' num2str(distance_to_ground) ' km.']);
    transmission_time = distance_to_ground / c;
    disp(['GEO data transmission to ground station: ' num2str(transmission_time) ' seconds.']);

    % Step 3: Ground station receives additional data from LEOs and creates graph
    LEO_data = collectLEOData(LEO_positions);  % New function to collect LEO data
    G = createGroundStationGraph(LEO_data);
    bestLEO = UserDecisionMaking(G, groundStation_position, currentLEO_index);

    % Initialize outputs
    nextLEO_position = [];
    predictedHandoverPoints = [];
    ping_LEO = [];

    % Check if a handover is needed
    if bestLEO ~= currentLEO_index
        % Update the next LEO position
        nextLEO_position = LEO_positions(bestLEO, :);
        
        % Append the next LEO position to the predicted handover points
        predictedHandoverPoints = [predictedHandoverPoints; nextLEO_position];
        
        % Calculate round-trip ping time to the next LEO
        distance = norm(nextLEO_position - groundStation_position);
        ping_LEO = 2 * (distance / c) * 1000; % in ms
        
        disp(['Predicted handover to LEO ' num2str(bestLEO) ' with ping: ' num2str(ping_LEO) ' ms.']);
        
        % Update current LEO to the next LEO
        currentLEO_index = bestLEO;

    end
end

function LEO_data = collectLEOData(LEO_positions)
    % New function: Ground station collects additional data directly from LEOs
    LEO_data = struct('positions', LEO_positions);
    
    % Simulate collecting data directly from LEOs
    for i = 1:size(LEO_positions, 1)
        LEO_data.velocities{i} = calculateLEOVelocity(LEO_positions(i,:));  % Get from LEO
        LEO_data.signal_strength{i} = getLEOSignalStrength(i);  % Get from LEO
    end
end

function G = createGroundStationGraph(LEO_data)
    % Initialize empty graph
    G = digraph();
    
    % Initialize node table with LEO properties
    NodeProperties = table('Size', [size(LEO_data.positions, 1), 5], 'VariableTypes', {'cell', 'cell', 'double', 'double', 'string'}, 'VariableNames', {'Position', 'Velocity', 'SignalStrength', 'CoverageRadius', 'Name'});
    for i = 1:size(LEO_data.positions, 1)
        % Extract LEO properties
        pos = LEO_data.positions(i, :);
        vel = LEO_data.velocities{i};
        signal = LEO_data.signal_strength{i};
        coverage_radius = 1000; % Set appropriate coverage radius
        
        % Add to node table
        % disp('NodeProperties:');
        % disp(NodeProperties);

        % disp('pos:');
        % disp(pos);

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
    % Get neighboring LEOs from graph using successors
    handoverCandidates = successors(G, currentLEO);
    
    % If no candidates available, stay with current LEO
    if isempty(handoverCandidates)
        bestLEO = currentLEO;
        return;
    end
    
    % Initialize best LEO search
    bestLEO = currentLEO;  % Default to current LEO
    max_score = calculate_handover_score(G, currentLEO, user_position);  % Calculate current LEO's score
    
    % Evaluate each candidate
    for i = 1:length(handoverCandidates)
        candidate = handoverCandidates(i);
        score = calculate_handover_score(G, candidate, user_position);
        
        % Only change if the candidate is significantly better (add threshold)
        if score > max_score * 1.1  % 10% improvement threshold
            max_score = score;
            bestLEO = candidate;
        end
    end
    
    % Execute handover if a better LEO was found
    if bestLEO ~= currentLEO
        ExecuteHandover(bestLEO);
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
    velocity_factor = calculate_velocity_factor(leo_data.Velocity{1}, user_position);
    
    % Weighted scoring (adjust weights based on importance)
    w1 = 0.4;  % Distance weight
    w2 = 0.4;  % Signal strength weight
    w3 = 0.2;  % Velocity factor weight
    
    score = w1 * (1/distance) + w2 * signal_strength + w3 * velocity_factor;
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

function velocity_factor = calculate_velocity_factor(leo_velocity, user_position)
    % Calculate how favorable the LEOs velocity is relative to user
    % Higher score means LEO is moving in a way that will maintain good coverage
    
    % This is a simplified model - you might want to enhance it
    velocity_magnitude = norm(leo_velocity);
    velocity_factor = 1/velocity_magnitude;  % Slower relative motion is better
end

function velocity = calculateLEOVelocity(position)
    % calculateLEOVelocity - Calculate velocity vector for a LEO satellite
    %
    % Inputs:
    %   position - 1x3 vector [x, y, z] representing LEO position
    %
    % Outputs:
    %   velocity - 1x3 vector [vx, vy, vz] representing LEO velocity in km/s
    
    % Constants
    G = 6.67430e-11;  % Gravitational constant (m^3 kg^-1 s^-1)
    M = 5.972e24;     % Mass of Earth (kg)
    R = 6371000;      % Radius of Earth (m)
    
    % Calculate orbital parameters
    altitude = norm(position);  % Distance from Earth's center
    orbital_speed = sqrt(G * M / altitude);  % Orbital velocity magnitude
    
    % Calculate velocity vector components
    position_normalized = position / altitude;
    velocity_direction = cross([0, 0, 1], position_normalized);
    
    % Handle case when position is aligned with z-axis
    if norm(velocity_direction) < 1e-10
        velocity_direction = cross([1, 0, 0], position_normalized);
    end
    
    velocity_direction = velocity_direction / norm(velocity_direction);
    velocity = orbital_speed * velocity_direction;
    
    % Convert to km/s
    velocity = velocity / 1000;
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
