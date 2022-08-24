function [constraints] = SubOptimalCostFuction(outputs, states, controls)
load ObstacleConstraintsSubOptimal.mat;
x=states(1);
y=states(2);
vx=states(3);
vy=states(4);
ax=controls(1);
ay=controls(2);
time=states(5);
R1=CeR+(time/100);

% Weights for time,states and controls
Rt=1;                          % Time weight
wx=0.01;                       % Position x axis weight
wy=0.01;                       % Position y axis weight
wvx=1;                         % Velocity x axis weight
wvy=1;                         % Velocity y axis weight
Rs=[wx 0 0 0 0;0 wy 0 0 0;0 0 wvx 0 0;0 0 0 wvy 0;0 0 0 0 0];   % State weight vector including zeros to remove the arithmetical effects of the dummy variable (time)
wax=1;                         % Acceleration x axis weight
way=1;                         % Acceleration y axis weight
Rc=[wax 0;0 way];              % Controls weight vector

% Cost function definition
CostFunction=piecewise(log( (x-CeX)^2+(y-CeY)^2 ) <= 2 * log(R1) , Rt*1+(states')*Rs*states+2* ( (controls')*Rc*controls ),Rt*1+(states')*Rs*states+(controls')*Rc*controls);

% Output
constraints = [CostFunction];
end