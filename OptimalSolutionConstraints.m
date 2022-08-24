function [constraints] = OptimalSolutionConstraints(states,controls)
load ObstacleOptimalSolutionConstraints.mat;

% Extract states
x=states(1);
y=states(2);
vx=states(3);
vy=states(4);
ax=controls(1);
ay=controls(2);
time=states(5);

% Object model
x0=ObsIniX+(time/Divider)*SpeedX;  % Obstacle center x axis
y0=ObsIniY+(time/Divider)*SpeedY;  % Obstacle center y axis
R=ObsIniR+(time/Divider)*SpeedR;   % Obstacle dynamic radius
Obstacle=log( ( (x-x0) / (R+ObsSafRad) ) ^ 50 + ( (y-y0) / (R+ObsSafRad) ) ^ 50);

% Puddle model
ImpedenceFactor=time/100;
R1=CeR+(time/100);

MaxSpe=3-ImpedenceFactor;
CVX1=piecewise(log( (x-CeX)^2+(y-CeY)^2 ) <= 2 * log(R1) ,MaxSpe-vx,100);  % > 0 
CVX2=piecewise(log( (x-CeX)^2+(y-CeY)^2 ) <= 2 * log(R1) ,vx+MaxSpe,100);  % > 0
CVY1=piecewise(log( (x-CeX)^2+(y-CeY)^2 ) <= 2 * log(R1) ,MaxSpe-vy,100);  % > 0 
CVY2=piecewise(log( (x-CeX)^2+(y-CeY)^2 ) <= 2 * log(R1) ,vy+MaxSpe,100);  % > 0

MaxAcc=1-ImpedenceFactor;
CAX1=piecewise(log( (x-CeX)^2+(y-CeY)^2 ) <= 2 * log(R1) ,MaxAcc-ax,100);  % > 0 
CAX2=piecewise(log( (x-CeX)^2+(y-CeY)^2 ) <= 2 * log(R1) ,ax+MaxAcc,100);  % > 0
CAY1=piecewise(log( (x-CeX)^2+(y-CeY)^2 ) <= 2 * log(R1) ,MaxAcc-ay,100);  % > 0 
CAY2=piecewise(log( (x-CeX)^2+(y-CeY)^2 ) <= 2 * log(R1) ,ay+MaxAcc,100);  % > 0

% Output
% constraints=[Obstacle;CVX1;CVX2;CVY1;CVY2;CAX1;CAX2;CAY1;CAY2];
constraints=[Obstacle;1;1;1;1;CAX1;CAX2;CAY1;CAY2];

end