function [currentLEO_index, bestLEO_position, predictedHandoverPoints, ping_LEO] = keplerianHandoverProcess(GEO_position, LEO_positions, LEO_velocities, user_position, currentLEO_index, c, network)
    % Initialize outputs
    predictedHandoverPoints = [];
    
    % Validate currentLEO_index first
    if isempty(currentLEO_index) || currentLEO_index <= 0 || currentLEO_index > size(LEO_positions, 1)
        % If invalid, initialize to the closest LEO satellite
        distances = vecnorm(LEO_positions - user_position, 2, 2);
        [~, currentLEO_index] = min(distances);
    end
    
    % Now calculate distances with valid index
    LEO_to_user_distance = norm(LEO_positions(currentLEO_index,:) - user_position);
    
    % Calculate distance and base propagation time
    one_way_propagation = (LEO_to_user_distance / c) * 1000; % Convert to ms

    % Calculate total ping
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
    
    % Constants
    mu = 3.986004418e14; % Earth's gravitational parameter (m^3/s^2)
    prediction_time = 100; % Time ahead to predict (seconds)
    
    % Convert Cartesian positions/velocities to orbital elements
    [a, e, i, RAAN, omega, nu] = cart2kep(LEO_positions(currentLEO_index,:), LEO_velocities(currentLEO_index,:), mu);
    
    % Predict future positions using Kepler's equations
    future_positions = zeros(prediction_time, 3);
    for t = 1:prediction_time
        % Update mean anomaly
        n = sqrt(mu/a^3); % Mean motion
        M = n * t;
        
        % Solve Kepler's equation (simplified here, might need iteration)
        E = M; % For small eccentricities
        
        % Calculate new true anomaly
        nu_new = 2 * atan(sqrt((1+e)/(1-e)) * tan(E/2));
        
        % Convert back to Cartesian coordinates
        [r, v] = kep2cart(a, e, i, RAAN, omega, nu_new, mu);
        future_positions(t,:) = r;
    end
    
    % Find best LEO based on predicted positions
    distances = vecnorm(LEO_positions - user_position, 2, 2);
    [~, bestLEO_index] = min(distances);
    
    % Update current LEO if needed
    if bestLEO_index ~= currentLEO_index
        currentLEO_index = bestLEO_index;
        predictedHandoverPoints = future_positions(end,:);
    end
    
    bestLEO_position = LEO_positions(currentLEO_index,:);
end

% Helper functions (you'll need to implement these)
function [a, e, i, RAAN, omega, nu] = cart2kep(r, v, mu)
    % Convert Cartesian coordinates to Keplerian elements
    
    % Calculate angular momentum vector
    h = cross(r, v);
    
    % Calculate node vector
    n = cross([0 0 1], h);
    
    % Calculate eccentricity vector
    e_vec = ((norm(v)^2 - mu/norm(r))*r - dot(r,v)*v)/mu;
    
    % Semi-major axis
    a = -mu/(2*(norm(v)^2/2 - mu/norm(r)));
    
    % Eccentricity
    e = norm(e_vec);
    
    % Inclination
    i = acos(h(3)/norm(h));
    
    % Right Ascension of Ascending Node (RAAN)
    if norm(n) ~= 0
        RAAN = acos(n(1)/norm(n));
        if n(2) < 0
            RAAN = 2*pi - RAAN;
        end
    else
        RAAN = 0;
    end
    
    % Argument of periapsis
    if norm(n) ~= 0
        omega = acos(dot(n,e_vec)/(norm(n)*e));
        if e_vec(3) < 0
            omega = 2*pi - omega;
        end
    else
        omega = 0;
    end
    
    % True anomaly
    nu = acos(dot(e_vec,r)/(e*norm(r)));
    if dot(r,v) < 0
        nu = 2*pi - nu;
    end
end

function [r, v] = kep2cart(a, e, i, RAAN, omega, nu, mu)
    % Convert Keplerian elements to Cartesian coordinates
    
    % Calculate position in orbital plane
    r_mag = a * (1 - e^2) / (1 + e * cos(nu));
    r_orbital = r_mag * [cos(nu); sin(nu); 0];
    
    % Calculate velocity in orbital plane
    p = a * (1 - e^2);  % semi-latus rectum
    v_orbital = sqrt(mu/p) * [-sin(nu); e + cos(nu); 0];
    
    % Rotation matrices
    R3_RAAN = [cos(RAAN) -sin(RAAN) 0;
               sin(RAAN) cos(RAAN)  0;
               0         0          1];
           
    R1_i = [1 0         0;
            0 cos(i)    -sin(i);
            0 sin(i)    cos(i)];
        
    R3_omega = [cos(omega) -sin(omega) 0;
                sin(omega) cos(omega)  0;
                0         0           1];
            
    % Combined rotation matrix
    Q = R3_RAAN * R1_i * R3_omega;
    
    % Transform to Earth-centered inertial frame
    r = Q * r_orbital;
    v = Q * v_orbital;
    
    % Convert to row vectors to match MATLAB convention
    r = r';
    v = v';
end