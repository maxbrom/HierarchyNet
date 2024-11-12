classdef HierarchySatellite
    % Satellite class to represent a satellite's orbital parameters
    
    properties
        % Define properties for the satellite's parameters
        altitude % Altitude in km
        inclination % Orbital inclination in degrees
        longitude % Longitude in degrees
        latitude % Latitude in degrees
        time_to_latitude % Time when satellite would reach "latitude" parameter if launched from (Equator, Prime Meridian)
        lon_offset % Amount to offset the longitude after satellite location calculation so that at t=0, the satellite is at "longitude"
    end
    
    methods
        % Constructor to initialize the satellite parameters
        function obj = HierarchySatellite(altitude, inclination, latitude, longitude)
            if nargin > 0
                % Set the initial values for the satellite parameters
                obj.altitude = altitude;
                obj.inclination = inclination;
                obj.latitude = latitude;
                obj.longitude = longitude;
            end
        end
        
    end
end