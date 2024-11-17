function [currentLEO_index, nextLEO_position, predictedHandoverPoints, ping_LEO] = handoverProcess(GEO_position, LEO_positions, groundStation_position, currentLEO_index, c)
    % Function to simulate the full handover process:
    % 1. GEO collects LEO positions
    % 2. GEO passes data to the ground station
    % 3. Ground station predicts the next LEO for handover
    %
    % Inputs:
    %   GEO_position - 1x3 vector of the GEO satellite position
    %   LEO_positions - Nx3 matrix of LEO satellite positions (N is the number of LEOs)
    %   groundStation_position - 1x3 vector of the ground station position
    %   currentLEO_index - Index of the currently connected LEO satellite
    %   c - Speed of light in km/s
    %
    % Outputs:
    %   currentLEO_index - Updated index of the current LEO satellite
    %   nextLEO_position - 1x3 vector of the next LEO satellites position
    %   predictedHandoverPoints - Updated matrix of predicted handover points
    %   ping_LEO - Ping time to the next LEO satellite in milliseconds

    % Step 1: GEO collects LEO positions
    num_LEOs = size(LEO_positions, 1);
    collected_LEO_data = LEO_positions; % Assume GEO has all real-time LEO positions
    disp('GEO satellite collected LEO position data.');

    % Step 2: GEO forwards data to the ground station
    % Simulate transmission delay from GEO to ground station
    distance_to_ground = norm(GEO_position - groundStation_position);
    transmission_time = distance_to_ground / c; % Time in seconds
    disp(['GEO data transmission to ground station: ' num2str(transmission_time) ' seconds.']);

    % Step 3: Ground station predicts the next LEO handover
    % Calculate distances from all LEOs to the ground station
    distances_to_ground = zeros(num_LEOs, 1);
    for i = 1:num_LEOs
        distances_to_ground(i) = norm(collected_LEO_data(i, :) - groundStation_position);
    end

    % Find the closest LEO
    [min_distance, nextLEO_index] = min(distances_to_ground);

    % Initialize outputs
    nextLEO_position = [];
    predictedHandoverPoints = [];
    ping_LEO = [];

    % Check if a handover is needed
    if nextLEO_index ~= currentLEO_index
        % Update the next LEO position
        nextLEO_position = collected_LEO_data(nextLEO_index, :);
        
        % Append the next LEO position to the predicted handover points
        predictedHandoverPoints = [predictedHandoverPoints; nextLEO_position];
        
        % Calculate round-trip ping time to the next LEO
        ping_LEO = 2 * (min_distance / c) * 1000; % in ms
        
        disp(['Predicted handover to LEO ' num2str(nextLEO_index) ' with ping: ' num2str(ping_LEO) ' ms.']);
        
        % Update current LEO to the next LEO
        currentLEO_index = nextLEO_index;
    else
        % No handover, return current values
        nextLEO_index = currentLEO_index;
    end
end
