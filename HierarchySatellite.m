classdef HierarchySatellite
    % Satellite class to represent a satellite's orbital parameters
    
    properties
        % Define properties for the satellite's parameters
        altitude % Altitude in km
        inclination % Orbital inclination in degrees
        longitude % Longitude in degrees
        latitude % Latitude in degrees
    end
    
    methods
        % Constructor to initialize the satellite parameters
        function obj = HierarchySatellite(altitude, inclination, longitude, latitude)
            if nargin > 0
                % Set the initial values for the satellite parameters
                obj.altitude = altitude;
                obj.inclination = inclination;
                obj.longitude = longitude;
                obj.latitude = latitude;
            end
        end
        
    end
end