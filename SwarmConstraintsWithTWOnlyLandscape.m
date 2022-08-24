 function [constraints] = SwarmConstraintsWithTWOnlyLandscape(states,controls)
 % In this case we can use only the TW, or combine the TW with the wrapper's predictions
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

% Puddle model
ImpedenceFactor=time/100;
MaxAcc=1-ImpedenceFactor;

[m,n]=size(LeaObsDetCon2);  
constraints=[ ];
for i=1:m
    CAX1=piecewise( time<LeaObsDetCon2(i,1),time,time>LeaObsDetCon2(i,2),time,log( ( (x-LeaObsDetCon2(i,4))^2  / LeaObsDetCon2(i,6)^2 )  + ( (y-LeaObsDetCon2(i,5))^2  / LeaObsDetCon2(i,7)^2 ) )<=0, MaxAcc-ax,time );
    CAX2=piecewise( time<LeaObsDetCon2(i,1),time,time>LeaObsDetCon2(i,2),time,log( ( (x-LeaObsDetCon2(i,4))^2  / LeaObsDetCon2(i,6)^2 )  + ( (y-LeaObsDetCon2(i,5))^2  / LeaObsDetCon2(i,7)^2 ) )<=0, ax+MaxAcc,time );
    CAY1=piecewise( time<LeaObsDetCon2(i,1),time,time>LeaObsDetCon2(i,2),time,log( ( (x-LeaObsDetCon2(i,4))^2  / LeaObsDetCon2(i,6)^2 )  + ( (y-LeaObsDetCon2(i,5))^2  / LeaObsDetCon2(i,7)^2 ) )<=0, MaxAcc-ay,time );
    CAY2=piecewise( time<LeaObsDetCon2(i,1),time,time>LeaObsDetCon2(i,2),time,log( ( (x-LeaObsDetCon2(i,4))^2  / LeaObsDetCon2(i,6)^2 )  + ( (y-LeaObsDetCon2(i,5))^2  / LeaObsDetCon2(i,7)^2 ) )<=0, ay+MaxAcc,time );
    constraints=[constraints;CAX1;CAX2;CAY1;CAY2];
end

end