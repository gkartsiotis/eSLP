function [constraints] = SwarmConstraintsWithoutTW(states) %#codegen
% In this case the TW is not utilized and we only use the wrapper's predictions
% For testing purposes; currently not used

load ObstacleConstraints.mat;

% Extract states
x    = states(1);
y    = states(2);
time = states(5);

% Real time obstacle position and radius
x0=ObsIniX+(time/Divider)*SpeedX;  % Obstacle center x axis
y0=ObsIniY+(time/Divider)*SpeedY;  % Obstacle center y axis
R=ObsIniR+(time/Divider)*SpeedR;   % Obstacle dynamic radius

% Check if obstacle is withing detection area and keep ObsSafRad distance
ObstacleRealTime=log( ( (x-x0) / (R+ObsSafRad) ) ^ 50 + ( (y-y0) / (R+ObsSafRad) ) ^ 50);
constraints=[ObstacleRealTime];

end