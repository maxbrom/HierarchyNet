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