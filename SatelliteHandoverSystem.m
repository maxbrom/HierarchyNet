function G = GeoSatelliteTracking(GEO)
    % Initialize empty graph
    G = digraph();
    
    % Get LEOs in GEOs coverage area
    % Assuming GEO is a structure with coverage_area property containing LEO objects
    LEOs = GEO.coverage_area;
    
    % Initialize node table with LEO properties
    NodeProperties = table();
    for i = 1:length(LEOs)
        % Extract LEO properties
        pos = LEOs(i).position;      % [x, y, z] coordinates
        vel = LEOs(i).velocity;      % [vx, vy, vz] velocity vector
        sig = LEOs(i).signal;        % signal strength
        
        % Add to node table
        NodeProperties.Position{i} = pos;
        NodeProperties.Velocity{i} = vel;
        NodeProperties.SignalStrength(i) = sig;
        NodeProperties.Name{i} = sprintf('LEO_%d', i);
    end
    
    % Add nodes to graph
    G = addnode(G, NodeProperties);
    
    % Create edges between overlapping LEOs
    for i = 1:length(LEOs)
        for j = 1:length(LEOs)
            if i ~= j
                % Check if LEOs have coverage overlap
                if coverage_overlap(LEOs(i), LEOs(j))
                    % Calculate handover weight
                    weight = calculate_handover_weight(LEOs(i), LEOs(j));
                    % Add edge to graph
                    G = addedge(G, i, j, weight);
                end
            end
        end
    end
end

function [G_updated] = UpdateGraphData(GEO, G)
    % Get current LEO positions and data
    LEOs = GEO.coverage_area;
    
    % Get existing edges from the graph
    EdgeTable = G.Edges;
    
    % Update each edge weight
    for i = 1:height(EdgeTable)
        src = EdgeTable.EndNodes(i,1);
        dst = EdgeTable.EndNodes(i,2);
        
        LEO1 = LEOs(src);
        LEO2 = LEOs(dst);
        
        new_weight = calculate_handover_weight(LEO1, LEO2);
        
        G_updated = G;
        G_updated = rmedge(G_updated, src, dst);
        G_updated = addedge(G_updated, src, dst, new_weight);
    end
    
    % Transmit updated graph
    transmit_graph(G_updated);
end

% User decision making function
function bestLEO = UserDecisionMaking(G, user_position)
    % Identify current LEO connection
    currentLEO = IdentifyCurrentLEO(G, user_position);
    
    % Get neighboring LEOs from graph
    handoverCandidates = neighbors(G, currentLEO);
    
    % Initialize best LEO search
    bestLEO = currentLEO;  % Default to current LEO
    max_score = -inf;
    
    % Evaluate each candidate
    for i = 1:length(handoverCandidates)
        candidate = handoverCandidates(i);
        score = calculate_handover_score(G, candidate, user_position);
        
        if score > max_score
            max_score = score;
            bestLEO = candidate;
        end
    end
    
    % Execute handover if a better LEO was found
    if bestLEO ~= currentLEO
        ExecuteHandover(bestLEO);
    end
end
% Helper function to check coverage overlap
function overlap = coverage_overlap(LEO1, LEO2)
    % Calculate distance between LEOs
    dist = norm(LEO1.position - LEO2.position);
    % Compare with coverage radius
    overlap = dist <= (LEO1.coverage_radius + LEO2.coverage_radius);
end

% Helper function to calculate handover weight
function weight = calculate_handover_weight(LEO1, LEO2)
    % Consider factors like:
    % - Distance between LEOs
    % - Relative velocity
    % - Signal strength difference
    dist = norm(LEO1.position - LEO2.position);
    rel_vel = norm(LEO1.velocity - LEO2.velocity);
    sig_diff = abs(LEO1.signal - LEO2.signal);
    
    % Weighted sum of factors (weights can be adjusted)
    weight = 0.4 * (1/dist) + 0.3 * (1/rel_vel) + 0.3 * (1/sig_diff);
end 

function transmit_graph(G)
    try
        % Convert graph to transmittable format
        graph_data = struct('Nodes', G.Nodes, 'Edges', G.Edges);
        
        % Log transmission attempt
        disp('Transmitting updated graph data...');
        
        % Here you would add your actual transmission code
        % Example: send_data_via_leo_network(graph_data);
        
    catch exception
        warning(exception.identifier, '%s', exception.message);
    end
end

%helper functions for UserDecisionMaking
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