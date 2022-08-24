 function [constraints] = SwarmConstraintsWithTW(states,controls)
% In this case we can use only the TW, or combine the TW with the wrapper's predictions
TWActivated=1;         % Keep a record that TW was activated
save TWActivated.mat;
load ObstacleConstraints.mat;

% Extract states
x=states(1);
y=states(2);
vx=states(3);
vy=states(4);
ax=controls(1);
ay=controls(2);
time=states(5);

% To use the wrapper's prediction in combination with the TW uncomment the following else only the predictions from the TW will be produced
% Currently not used
% % Real time obstacle position and radius
% x0=ObsIniX+(time/Divider)*SpeedX;  % Obstacle center x axis
% y0=ObsIniY+(time/Divider)*SpeedY;  % Obstacle center y axis
% R=ObsIniR+(time/Divider)*SpeedR;   % Obstacle dynamic radius
% % Obstacle current iteration
% ObstacleRealTime=log( ( (x-x0) / (R+ObsSafRad) ) ^ 50 + ( (y-y0) / (R+ObsSafRad) ) ^ 50);
% constraints=[ObstacleRealTime];

% Time window predictions (C(t_k))
[m,n]=size(LeaObsDetCon);  
constraints=[ ];
for i=1:m
    Obstacle=piecewise( time<LeaObsDetCon(i,1),time,time>LeaObsDetCon(i,2),time,log( ( (x-LeaObsDetCon(i,4))^2  / LeaObsDetCon(i,6)^2 )  + ( (y-LeaObsDetCon(i,5))^2  / LeaObsDetCon(i,7)^2 ) ) );
    constraints=[constraints;Obstacle];
end


end