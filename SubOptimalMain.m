clear;clc;close all;                         % Clear workspace
% ------------------------------------------------------------------------------------------------------------------------------------------------------------------
% Algorithm parameters
% ------------------------------------------------------------------------------------------------------------------------------------------------------------------
delta=14.794;                                % Agent separation 
sampling=7.21;                               % Sampling period
LeaDecRad=sqrt(2)*3;                         % Detection radius for the area centered at each agent
MaxTime=150;                                 % Maximum experiment time
PlotRealTimeProgression=1   ;                % Plot suboptimal agent vs obstacle real time figure
ProximityDistance=0.01;                      % Proximity maximum distance between leader follower
% ------------------------------------------------------------------------------------------------------------------------------------------------------------------
% End of: Algorithm parameters
% ------------------------------------------------------------------------------------------------------------------------------------------------------------------

% ------------------------------------------------------------------------------------------------------------------------------------------------------------------
% Agent x_0 initialization
% ------------------------------------------------------------------------------------------------------------------------------------------------------------------
% 1. Speed and acceleration limits
VelocitySetting=3;                           % Uniform selection for both axis velocity (remove and set manually if different speed settings are needed)
AccelerationSetting=1;                       % Uniform selection for both axis accleration (remove and set manually if different accleration settings are needed)
LowerVelocityLimitX=-VelocitySetting;        % Lower limit for the x-axis speed (state)
UpperVelocityLimitX=VelocitySetting;         % Upper limit for the x-axis speed (state)
LowerVelocityLimitY=-VelocitySetting;        % Lower limit for the y-axis speed (state)
UpperVelocityLimitY=VelocitySetting;         % Upper limit for the y-axis speed (state)
LowerControlLimitX=-AccelerationSetting;     % Lower limit for the x-axis acceleration (control)
UpperControlLimitX=AccelerationSetting;      % Upper limit for the x-axis acceleration (control)
LowerControlLimitY=-AccelerationSetting;     % Lower limit for the y-axis acceleration (control)
UpperControlLimitY=AccelerationSetting;      % Upper limit for the y-axis acceleration (control)

% 2. Time discretization 
TimeLowerLimit=0;                                      % Final time is free but values are needed for discretization; lower limit, upper limit, step and 
TimeLimitStep=MaxTime;                                 % an initial guess (midpoint) with MaxTime available in total
TimeUpperLimit=TimeLowerLimit+TimeLimitStep;    
TimeInitialGuess=(TimeLowerLimit+TimeUpperLimit)/2;    
TimeStep=0.01;
ObsFactor=3;
ExtraPoints=100;

% 3. Scaling
PositionXScaling=0.1;
PositionYScaling=0.01;
VelocityScaling=1;
AccelerationScaling=1;
TimeScaling=1;

% 4. Dynamics are discretized on a time grid    % Number of discretization points on the time grid used for dynamics
DynamicsDiscretizationStep=1;
DynamicsDiscretizationUpperLimit=91;         

% 5. Phase initial time                         % Each problem in Falcon has at least one phase; we solve one problem with one phase for x_0 (suboptimal agent)
PhaseInitialTime=0;                             % and multiple problems each one having at one phase for x_k, k>=1 (optimal agents)

% 6. Solver tolerance
FeasabilityTolerance=1e-5;                      % Feasibility tolerance for the IPOPT solver
OptimalityTolerance=1e-5;                       % Optimality tolerance for the IPOPT solver

% 7. Initial boundaries                              
xi=5;                                           % Agent initial position x-axis
yi=0;                                           % Agent initial position y-axis
vxi=0;                                          % Agent iInitial speed x-axis
vyi=0;                                          % Agent initial speed y-axis

% 8. Final boundaries
xfa=2;xfb=10;                                   % Target area position x-axis [xfa,xfb]
yfa=300;yfb=350;                                % Target area position y-axis [yfa,yfb]
xv=[xfa xfb];yv=[yfa yfb];                      % Target area in vector format
vxfa=-VelocitySetting;vxfb=VelocitySetting;     % Target area speed x-axis [vxfa,vxfb]
vyfa=-VelocitySetting;vyfb=VelocitySetting;     % Target area speed y-axis [vyfa,vyfb]      
                                                
% 9. Obstacle
ObsIniX=-40;                                    % Obstacle initial x-position 55
ObsIniY=145;                                    % Obstacle initial y-position
ObsIniR=4.5;                                    % Obstacle initial radius
ObsSafRad=0.5;                                  % Safety distance from obstacle
SpeedX=0.9;                                     % Obstacle speed x-axis multiplier
SpeedY=0;                                       % Obstacle speed y-axis multiplier
SpeedR=0;                                       % Obstacle dynamic radius growth multiplier
Divider=1;                                      % Every Divider time units the obstacle moves for SpeedX SpeedY and it's radius grows by SpeedR
CeX=5.5;
CeY=145;
CeR=4.5;

% 10. Iterations related
TotalResults={ };                               % Used to store results from all agents
AgentCounterRow=1;                              % Each agent corresponds to one row in TotalResults
AgentCounterColumn=1;                           % Each iteration per agent is stored in different column

% 11. Other variables to be transferred
RoadLowerLimitX=-150;                          
RoadUpperLimitX=150;
RoadScalingX=0.01;
RoadLowerLimitY=0;                              
RoadUpperLimitY=350;
RoadScalingY=0.01;
PlotXLow=-100;PlotXUpp=100;
PlotYLow=0;PlotYUpp=350;

save AlgorithmParametersX0.mat;                 % Save parameters to be loaded when the next agent starts - Save A
% ------------------------------------------------------------------------------------------------------------------------------------------------------------------
% End of: Agent x_0 initialization
% ------------------------------------------------------------------------------------------------------------------------------------------------------------------

% ------------------------------------------------------------------------------------------------------------------------------------------------------------------
% Agent x_0 Main
% ------------------------------------------------------------------------------------------------------------------------------------------------------------------
% A. Define state space and controls
% State vector: Position axis - x (x), Position axis - y (y), Velocity axis - x (vx), Velocity axis - y (vy) and dummy state (time)
x_vec = [...
    falcon.State('x',5,150,PositionXScaling);...
    falcon.State('y',0,350,PositionYScaling);...
    falcon.State('vx',LowerVelocityLimitX,UpperVelocityLimitX,VelocityScaling);...
    falcon.State('vy',LowerVelocityLimitY,UpperVelocityLimitY,VelocityScaling);...
    falcon.State('time',TimeLowerLimit,TimeUpperLimit,1)];

% Control vector: Acceleration axis - x (ax), Acceleration axis - y (ay)
u_vec = [...
    falcon.Control('ax',LowerControlLimitX,UpperControlLimitX,AccelerationScaling);...      
    falcon.Control('ay',LowerControlLimitY,UpperControlLimitY,AccelerationScaling)];

% B. Final time                                                            % Create a Falcon parameter and set it as final time of the phase results in solving a free final time problem
tf = falcon.Parameter('FinalTime', TimeInitialGuess,TimeLowerLimit,TimeUpperLimit,TimeStep);  

% C. Dynamics discretization                                               % The time grid used for the dynamics discretization
tau = linspace(0,DynamicsDiscretizationStep,DynamicsDiscretizationUpperLimit);

% D. Problem: Create a new Instance
problem = falcon.Problem('SubOptimalAgent');

% E. Problem: Add a new phase                                              % For x_0 we have one problem with one phase and for x_k k>=1 we solve multiple problems each one having one phase
phase = problem.addNewPhase(@SubOptimalProblemDefinition, x_vec, tau, PhaseInitialTime, tf);   

% F. Phase: Add control grid                                               % The control vector is discretized on time grid tau created at C (dynamics discretization time grid)
phase.addNewControlGrid(u_vec, tau);

% G. Phase: Add model output
phase.Model.setModelOutputs([falcon.Output('x_out'); falcon.Output('y_out');falcon.Output('vx_out');falcon.Output('vy_out');falcon.Output('time_out');falcon.Output('ax_out');falcon.Output('ay_out')]);

% H. Phase: Add Lagrange cost function      % Suboptimal agent cost function
OptimizeCostFunction=1;                     % If value is 0 then we optimize using only time
if OptimizeCostFunction==0                  % else we use the cost function
   problem.addNewParameterCost(tf);  
else
   Cost=falcon.Cost('Cost');
   phase.addNewLagrangeCost(@SubOptimalCostFuction,Cost,tau);
end                 

% I. Phase: Add boundary conditions
phase.setInitialBoundaries([xi;yi;vxi;vyi;TimeLowerLimit]);                                               % Set initial boundaries
phase.setFinalBoundaries([x_vec(1);x_vec(2);x_vec(3);x_vec(4)],[xfa;350;vxfa;vyfa],[xfb;350;vxfb;vyfb]);  % Set final boundaries [xfa,xfb],[yfa,yfb] for states x,y and [vxfa,vxfb],[vyfa,vyfb] for vx,vy

% J. Phase: Add constraints (obstacles)
save ObstacleConstraintsSubOptimal.mat ObsIniX ObsIniY ObsIniR SpeedX SpeedY SpeedR ObsSafRad Divider CeX CeY CeR;
pconMdl = falcon.PathConstraintBuilder('AntsPCon',[],x_vec,u_vec,[],@SubOptimalConstraints);
pathconstraints = [falcon.Constraint('Obstacle',0,inf);...
                   falcon.Constraint('CVX1',0,inf);falcon.Constraint('CVX2',0,inf);falcon.Constraint('CVY1',0,inf);falcon.Constraint('CVY2',0,inf);...
                   falcon.Constraint('CAX1',0,inf);falcon.Constraint('CAX2',0,inf);falcon.Constraint('CAY1',0,inf);falcon.Constraint('CAY2',0,inf)];
phase.addNewPathConstraint(@AntsPCon, pathconstraints, tau);
pconMdl.Build();

% K. Problem: Prepare for solving
problem.Bake();

% L. Problem: Solve 
solver = falcon.solver.ipopt(problem);                    % Solve with IPOPT
solver.Options.MajorIterLimit = 3000;                     % Maximum number of iterations
solver.Options.MajorFeasTol = FeasabilityTolerance;       % Feasibility tolerance
solver.Options.MajorOptTol = OptimalityTolerance;         % Optimality tolerance
solver.Solve();                                           % Solve command

% 1. Leader's trajectory and detection area for each point
Leader(1,:)=[TimeLowerLimit xi yi];                                                                          % Initial time and position
Leader=[Leader;phase.RealTime(1,2:end)' phase.StateGrid.Values(1,2:end)' phase.StateGrid.Values(2,2:end)'];  % Trajectory and real time grid for leader
[m,n]=size(Leader);
Leader(1,4)=Leader(1,2)-(LeaDecRad*(Leader(2,1)-Leader(1,1)));                                
Leader(1,5)=Leader(1,2)+(LeaDecRad*(Leader(2,1)-Leader(1,1)));
Leader(1,6)=Leader(1,3)-(LeaDecRad*(Leader(2,1)-Leader(1,1)));
Leader(1,7)=Leader(1,3)+(LeaDecRad*(Leader(2,1)-Leader(1,1)));
for i=2:m                                                                 
     Leader(i,4)=Leader(i-1,2)-(LeaDecRad*(Leader(2,1)-Leader(1,1)));
     Leader(i,5)=Leader(i-1,2)+(LeaDecRad*(Leader(2,1)-Leader(1,1)));
     Leader(i,6)=Leader(i-1,3)-(LeaDecRad*(Leader(2,1)-Leader(1,1)));
     Leader(i,7)=Leader(i-1,3)+(LeaDecRad*(Leader(2,1)-Leader(1,1)));
     x(1,1)=Leader(i,2)-(LeaDecRad*(Leader(2,1)-Leader(1,1)));
     x(1,2)=Leader(i,2)+(LeaDecRad*(Leader(2,1)-Leader(1,1)));
     x(2,1)=Leader(i,3)-(LeaDecRad*(Leader(2,1)-Leader(1,1)));
     x(2,2)=Leader(i,3)+(LeaDecRad*(Leader(2,1)-Leader(1,1)));
     Leader(i,4)=min(x(1,1),Leader(i,4));
     Leader(i,5)=max(x(1,2),Leader(i,5));
     Leader(i,6)=min(x(2,1),Leader(i,6));
     Leader(i,7)=max(x(2,2),Leader(i,7));
end
clear m n i;

[m,n]=size(Leader);
LeaderTemp=[ ];
k=0;
for i=1:m-1
    Points=linspace(Leader(i,1),Leader(i+1,1),ObsFactor);
    for j=1:ObsFactor
        LeaderTemp(j+k,1)=Points(j);
        TempX=interp1(Leader(:,1),Leader(:,2),Points(j));
        TempY=interp1(Leader(:,1),Leader(:,3),Points(j));
        TempX1=interp1(Leader(:,1),Leader(:,4),Points(j));
        TempX2=interp1(Leader(:,1),Leader(:,5),Points(j));
        TempY1=interp1(Leader(:,1),Leader(:,6),Points(j));
        TempY2=interp1(Leader(:,1),Leader(:,7),Points(j));
        LeaderTemp(j+k,2)=TempX;LeaderTemp(j+k,3)=TempY;
        LeaderTemp(j+k,4)=TempX1;LeaderTemp(j+k,5)=TempX2;
        LeaderTemp(j+k,6)=TempY1;LeaderTemp(j+k,7)=TempY2;
    end
    k=k+ObsFactor;
end
LeaderTemp=unique(LeaderTemp,'rows');
clear m n k Points size i j Leader TempX TempY TempX1 TempX2 TempY1 TempY2;
Leader=LeaderTemp;

TempStep=Leader(end,1)-Leader(end-1,1);
[m,n]=size(Leader);
for i=1:ExtraPoints
    Leader(end+1,1)=Leader(end,1)+TempStep;
    Leader(end,2:n)=Leader(end-1,2:n);
end
clear m n i TempStep LeaderTemp;

save LeaderX0.mat Leader;                                                  % Save leader's trajectory - Save B

SubOptimalCosts(1,:)=[TimeLowerLimit phase.LagrangeCostFunctions.OutputGrid.Values(1,1)];
SubOptimalCosts=[SubOptimalCosts;phase.RealTime(1,2:end)' phase.LagrangeCostFunctions.OutputGrid.Values(2:end)'];
save SubOptimalCosts.mat SubOptimalCosts;
clear temp1 temp4;

% 2. Obstacle trajectory and time grid step
Obstacle=[ ];
[m,n]=size(Leader);
for i=1:m
    Obstacle(i,1)=Leader(i,1);
    Obstacle(i,2)=ObsIniX+(Leader(i,1)/Divider)*SpeedX;  % Obstacle center x axis
    Obstacle(i,3)=ObsIniY+(Leader(i,1)/Divider)*SpeedY;  % Obstacle center y axis
    Obstacle(i,4)=ObsIniR+(Leader(i,1)/Divider)*SpeedR;  % Obstacle dynamic radius
    
    Obstacle2(i,1)=Leader(i,1);
    Obstacle2(i,2)=CeX;                                  % Puddle center x axis
    Obstacle2(i,3)=CeY;                                  % Puddle center y axis
    Obstacle2(i,4)=CeR+(Leader(i,1)/100);                  % Obstacle dynamic radius
end
Obstacle(:,5)=Obstacle(:,2)-(Obstacle(:,4)+ObsSafRad);   % Detection area around each leader's position is a rectangle of LeaDecRad radius so we
Obstacle(:,6)=Obstacle(:,2)+(Obstacle(:,4)+ObsSafRad);   % take the four courners and produce a LeaDecRad*LeaDecRad bounding box
Obstacle(:,7)=Obstacle(:,3)-(Obstacle(:,4)+ObsSafRad);
Obstacle(:,8)=Obstacle(:,3)+(Obstacle(:,4)+ObsSafRad);
save ObstacleX0.mat Obstacle;                            % Save obstacle related information - Save C

Obstacle2(:,5)=Obstacle2(:,2)-Obstacle2(:,4);   % Detection area around each leader's position is a rectangle of LeaDecRad radius so we
Obstacle2(:,6)=Obstacle2(:,2)+Obstacle2(:,4);   % take the four courners and produce a LeaDecRad*LeaDecRad bounding box
Obstacle2(:,7)=Obstacle2(:,3)-Obstacle2(:,4);
Obstacle2(:,8)=Obstacle2(:,3)+Obstacle2(:,4);
save ObstacleX0P.mat Obstacle2;                          % Save obstacle related information - Save C
clear m n i;

% 3. Initial time window construction
% 3.1. Leader positions and detections for the first Ä time units
PointTemp=find(Leader(:,1)<=delta);                                        % Leader's position after ? time units
PointA=PointTemp(end);                                  
[m,n]=size(Leader);
if (Leader(PointA,1)==delta) || (m==PointA)                                % If Ä matches the value in the discretization grid then we don't need to interpolate
   xf=Leader(PointA,2);yf=Leader(PointA,3);                                % Leader's position at Ä to be used as final boundary for the next agent
   LeaderDet=Leader(1:PointA,:);                                           % Keep leader's positions for first ? time units
   Lx=min(Leader(1:PointA,4));Ux=max(Leader(1:PointA,5));                  % xmin xmax for all x-axis detections in the initial window [0,?]
   Ly=min(Leader(1:PointA,6));Uy=max(Leader(1:PointA,7));                  % ymin ymax for all x-axis detections in the initial window [0,?]
else                                                                                                % If interpolation is needed
   PointB=PointA+1;                                                                                 % Latter interpolation point
   xf=interp1([Leader(PointA,1) Leader(PointB,1)],[Leader(PointA,2) Leader(PointB,2)],delta);       % Interpolation for position-x
   yf=interp1([Leader(PointA,1) Leader(PointB,1)],[Leader(PointA,3) Leader(PointB,3)],delta);       % Interpolation for position-y
   LowerX=xf-LeaDecRad;UpperX=xf+LeaDecRad;                                % Detection area for interpolated position-x
   LowerY=yf-LeaDecRad;UpperY=yf+LeaDecRad;                                % Detection area for interpolated position-x
   LeaderDet=[Leader(1:PointA,:);delta xf yf LowerX UpperX LowerY UpperY]; % Keep interpolated leader's position for [0,Ä] including the interpolation point
   Lx=min(min(Leader(1:PointA,4)),LowerX);                                 % xmin for all detections in the initial window [0,Ä] including the interpolation point
   Ux=max(max(Leader(1:PointA,5),UpperX));                                 % xmax for all detections in the initial window [0,Ä] including the interpolation point
   Ly=min(min(Leader(1:PointA,6)),LowerY);                                 % ymin for all detections in the initial window [0,Ä] including the interpolation point
   Uy=max(max(Leader(1:PointA,7)),UpperY);                                 % ymin for all detections in the initial window [0,Ä] including the interpolation point
end

% 3.2. Time window variable
TW(AgentCounterRow,:)=[1 0 delta xi yi vxi vyi xf yf xf yf Lx Ux Ly Uy delta 0]; % Initial time window
save TimeWindowX0.mat TW;                                                  % Save initial time window information - Save D
clear PointTemp PointA m n xf yf Lx Ux Ly Uy PointB LowerX UpperX LowerY UpperY;

% 3.3. Obstacle position and detections for the first Ä time units
PointTemp=find(Obstacle(:,1)<=delta);                                      % Find obstacle coordinates up to Ä
PointA=PointTemp(end);      
[m,n]=size(Obstacle);
if (Obstacle(PointA,1)==delta) || (m==PointA)                              % If Ä matches the value in the discretization grid or if delta is less than the time needed for x_0 to reach goal then no interpolation is needed
   ObstacleDet=Obstacle(1:PointA,:);                     
   ObstacleDet2=Obstacle2(1:PointA,:);
else
   x0=ObsIniX+(delta/Divider)*SpeedX;  % Obstacle center x axis at delta
   y0=ObsIniY+(delta/Divider)*SpeedY;  % Obstacle center y axis at delta
   R=ObsIniR+(delta/Divider)*SpeedR;   % Obstacle dynamic radius at delta
   ObstacleDet=[Obstacle(1:PointA,:);delta x0 y0 R x0-(R+ObsSafRad) x0+(R+ObsSafRad) y0-(R+ObsSafRad) y0+(R+ObsSafRad)]; % Add interpolated points  

   x0=CeX;                               % Puddle center x axis at delta
   y0=CeY;                               % Puddle center y axis at delta
   R=CeR+delta/100;                      % Obstacle dynamic radius at delta
   ObstacleDet2=[Obstacle2(1:PointA,:);delta x0 y0 R x0-R x0+R y0-R y0+R]; % Add interpolated points
end
clear PointTemp PointA m n x0 y0 R;

% 3.3. Intersections for the detections around each leader's position with the obstacle
[m,n]=size(ObstacleDet);
for i=1:m
     xmin=ObstacleDet(i,5);xmax=ObstacleDet(i,6);ymin=ObstacleDet(i,7);ymax=ObstacleDet(i,8);  % Bounding box from the four corners of each obstacle's position 
     LeaObsIntPoly(i,1)=polyshape([xmin xmin xmax xmax],[ymin ymax ymax ymin]);                % Polyshape corrsponding to the obstacle's bounding box
     xmin=Leader(i,4);xmax=Leader(i,5);ymin=Leader(i,6);ymax=Leader(i,7);                      % Bounding box from the four corners of each leader's position 
     LeaObsIntPoly(i,2)=polyshape([xmin xmin xmax xmax],[ymin ymax ymax ymin]);                % Polyshape corrsponding to the leader's bounding box
     LeaObsIntPoly(i,3)=intersect(LeaObsIntPoly(i,1),LeaObsIntPoly(i,2));                      % Bounding boxes for intersections (0 vertices if empty)
end
save IntersectionPolyshapeX0.mat LeaObsIntPoly; % Save E
clear m n i xmin xmax ymin ymax xmin xmax ymin ymax;

[m,n]=size(ObstacleDet);
for i=1:m
     xmin2=ObstacleDet2(i,5);xmax2=ObstacleDet2(i,6);ymin2=ObstacleDet2(i,7);ymax2=ObstacleDet2(i,8);  % Bounding box from the four corners of each puddles's position 
     LeaObsIntPoly2(i,1)=polyshape([xmin2 xmin2 xmax2 xmax2],[ymin2 ymax2 ymax2 ymin2]);               % Polyshape corrsponding to the puddle's bounding box
     xmin2=Leader(i,4);xmax2=Leader(i,5);ymin2=Leader(i,6);ymax2=Leader(i,7);                          % Bounding box from the four corners of each leader's position 
     LeaObsIntPoly2(i,2)=polyshape([xmin2 xmin2 xmax2 xmax2],[ymin2 ymax2 ymax2 ymin2]);               % Polyshape corrsponding to the leader's bounding box
     LeaObsIntPoly2(i,3)=intersect(LeaObsIntPoly2(i,1),LeaObsIntPoly2(i,2));                           % Bounding boxes for intersections (0 vertices if empty)
end
save IntersectionPolyshapeX0P.mat LeaObsIntPoly2; % Save E
clear m n i xmin2 xmax2 ymin2 ymax2 xmin2 xmax2 ymin2 ymax2;

% 3.4. Get non empty intersections and create minimum bounding ellipse for each one
[m,n]=size(LeaObsIntPoly);                                                                     
LeaObsDetTW=[ ];
IntersectionFlag=false;
j=1;   
for i=1:m
    if ~isempty(LeaObsIntPoly(i,3).Vertices)                               % Check if there was an intersection
        [XLimBou,YLimBou] = boundingbox(LeaObsIntPoly(i,3));               % Create bounding box for the intersection coordinates
        xmin=XLimBou(1);xmax=XLimBou(2);ymin=YLimBou(1);ymax=YLimBou(2);   % Prepare intersection coordinates for minimum bounding ellipse function
        P=[[xmin;ymin] [xmin;ymax] [xmax;ymax] [xmax;ymin]];
        [A,C]=MinVolEllipse(P,0.00001);                                    % Get a,b and coordinates for the center of the minimum bounding ellipse
        [a,b,C,X]=Ellipse_plot(A,C);
        LeaObsDetTW(j,1:7)=LeaderDet(i,:);                                  % Save Leader t,x,y and bounding box for each leader's position     
        LeaObsDetTW(j,8:15)=ObstacleDet(i,:);                               % Save obstacle t,x,y,r and bounding box for each obstacle's position
        LeaObsDetTW(j,16:19)=[XLimBou(1) XLimBou(2) YLimBou(1) YLimBou(2)]; % Save bounding box of each polygon intersection
        LeaObsDetTW(j,20:23)=[C(1) C(2) a b];                               % Save center-x,center-y,a and b for the minimum bounding ellipse of each intersection
        IntersectionFlag=true;
        j=j+1;
    end
end
clear m n IntersectionFlag j i XLimBou YLimBou xmin xmax ymin ymax P A C a b X LeaderDet ObstacleDet;

[m,n]=size(LeaObsIntPoly2);                                                                     
LeaObsDetTW2=[ ];
IntersectionFlag=false;
j=1;   
for i=1:m
    if ~isempty(LeaObsIntPoly2(i,3).Vertices)                                    % Check if there was an intersection
        [XLimBou2,YLimBou2] = boundingbox(LeaObsIntPoly2(i,3));                  % Create bounding box for the intersection coordinates
        xmin2=XLimBou(1);xmax2=XLimBou(2);ymin2=YLimBou(1);ymax2=YLimBou(2);     % Prepare intersection coordinates for minimum bounding ellipse function
        P2=[[xmin2;ymin2] [xmin2;ymax2] [xmax2;ymax2] [xmax2;ymin2]];
        [A2,C2]=MinVolEllipse(P,0.00001);                                        % Get a,b and coordinates for the center of the minimum bounding ellipse
        [a2,b2,C2,X2]=Ellipse_plot(A2,C2);
        LeaObsDetTW2(j,1:7)=LeaderDet2(i,:);                                     % Save Leader t,x,y and bounding box for each leader's position     
        LeaObsDetTW2(j,8:15)=ObstacleDet2(i,:);                                  % Save obstacle t,x,y,r and bounding box for each obstacle's position
        LeaObsDetTW2(j,16:19)=[XLimBou2(1) XLimBou2(2) YLimBou2(1) YLimBou2(2)]; % Save bounding box of each polygon intersection
        LeaObsDetTW2(j,20:23)=[C2(1) C2(2) a2 b2];                               % Save center-x,center-y,a and b for the minimum bounding ellipse of each intersection
        IntersectionFlag=true;
        j=j+1;
    end
end
clear m n IntersectionFlag j i XLimBou2 YLimBou2 xmin2 xmax2 ymin2 ymax2 P2 A2 C2 a2 b2 X2 LeaderDet2 ObstacleDet2;

% 3.5. Get minimum bounding ellipse for each pair of intersections
[m,n]=size(LeaObsDetTW);
LeaObsDetCon = [ ];
for i=1:m-1
     xmin=LeaObsDetTW(i,16);xmax=LeaObsDetTW(i,17);ymin=LeaObsDetTW(i,18);ymax=LeaObsDetTW(i,19);                    % Get first intersection
     PolyLowerTime=polyshape([xmin xmin xmax xmax],[ymin ymax ymax ymin]);                                           % Polyshape for first intersection
     xmin=LeaObsDetTW(i+1,16);xmax=LeaObsDetTW(i+1,17);ymin=LeaObsDetTW(i+1,18);ymax=LeaObsDetTW(i+1,19);            % Get second intersection
     PolyUpperTime=polyshape([xmin xmin xmax xmax],[ymin ymax ymax ymin]);                                           % Polyshape for second intersection
     PolyUnion(i,1)=union(PolyLowerTime,PolyUpperTime);                                                              % Union of the two intersections
     [XLimBou,YLimBou]=boundingbox(PolyUnion(i,1));                                                                  % Bounding box for the union
     xmin=XLimBou(1);xmax=XLimBou(2);ymin=YLimBou(1);ymax=YLimBou(2);                                                % Prepare intersection coordinates for minimum bounding ellipse function
     P=[[xmin;ymin] [xmin;ymax] [xmax;ymax] [xmax;ymin]];
     [A,C]=MinVolEllipse(P,0.00001);                                                                                 % Get a,b and coordinates for the center of the minimum bounding ellipse
     [a,b,C,X]=Ellipse_plot(A,C);
     LeaObsDetCon(i,:)=[LeaObsDetTW(i,1) LeaObsDetTW(i+1,1) (LeaObsDetTW(i,1)+LeaObsDetTW(i+1,1))/2 C(1) C(2) a b];  % Save center-x,center-y,a and b for the minimum bounding ellipse of each intersection pair         
end
clear m n i xmin xmax ymin ymax PolyLowerTime PolyUpperTime PolyUnion XLimBou YLimBou P A C a b C X;

[m,n]=size(LeaObsDetTW2);
LeaObsDetCon2 = [ ];
for i=1:m-1
     xmin2=LeaObsDetTW2(i,16);xmax2=LeaObsDetTW2(i,17);ymin2=LeaObsDetTW2(i,18);ymax2=LeaObsDetTW2(i,19);                    % Get first intersection
     PolyLowerTime2=polyshape([xmin2 xmin2 xmax2 xmax2],[ymin2 ymax2 ymax2 ymin2]);                                          % Polyshape for first intersection
     xmin2=LeaObsDetTW2(i+1,16);xmax2=LeaObsDetTW2(i+1,17);ymin2=LeaObsDetTW2(i+1,18);ymax2=LeaObsDetTW2(i+1,19);            % Get second intersection
     PolyUpperTime2=polyshape([xmin2 xmin2 xmax2 xmax2],[ymin2 ymax2 ymax2 ymin2]);                                          % Polyshape for second intersection
     PolyUnion2(i,1)=union(PolyLowerTime2,PolyUpperTime2);                                                                   % Union of the two intersections
     [XLimBou2,YLimBou2]=boundingbox(PolyUnion2(i,1));                                                                       % Bounding box for the union
     xmin2=XLimBou2(1);xmax2=XLimBou2(2);ymin2=YLimBou2(1);ymax2=YLimBou2(2);                                                % Prepare intersection coordinates for minimum bounding ellipse function
     P2=[[xmin2;ymin2] [xmin2;ymax2] [xmax2;ymax2] [xmax2;ymin2]];
     [A2,C2]=MinVolEllipse(P2,0.00001);                                                                                      % Get a,b and coordinates for the center of the minimum bounding ellipse
     [a2,b2,C2,X2]=Ellipse_plot(A2,C2);                            
     LeaObsDetCon2(i,:)=[LeaObsDetTW2(i,1) LeaObsDetTW2(i+1,1) (LeaObsDetTW2(i,1)+LeaObsDetTW2(i+1,1))/2 C2(1) C2(2) a2 b2];  % Save center-x,center-y,a and b for the minimum bounding ellipse of each intersection pair         
end
clear m n i xmin2 xmax2 ymin2 ymax2 PolyLowerTime2 PolyUpperTime2 PolyUnion2 XLimBou2 YLimBou2 P2 A2 C2 a2 b2 C2 X2;

save DetectionWindowX0.mat LeaObsDetTW LeaObsDetCon;                       % Save the time window detections and intersections realted information - Save F
save DetectionWindowX0P.mat LeaObsDetTW2 LeaObsDetCon2;                    % Save the time window detections and intersections realted information - Save F

% 4. Collect results
temp1=phase.RealTime(1,1:end)';
temp2=phase.StateGrid.Values(1:4,1:end)';
temp3=phase.ControlGrids.Values(:,1:end)';
Leader2=[temp1 temp2 temp3];   % Trajectory and real time grid for leader
TotalResults{AgentCounterRow,AgentCounterColumn}=Leader2;            % Store suboptimal agent real time grid, position-x and position-y
save TotalResultsX0.mat TotalResults;  % Save G
clear temp1 temp2 Leader2;

% 5. Plots | 5.1. Sub optimal agent trajectory
FigureX0A=figure('Name','Sub optimal agent trajectory');title('Sub optimal agent trajectory');
xlim([PlotXLow PlotXUpp]);ylim([PlotYLow PlotYUpp]);xlabel('x');ylabel('y');   % Figure settings for axis
hold on;
scatter(xi,yi,'MarkerFaceColor','y');                                      % Sub optimal agent initial position
scatter(Leader(1,2),Leader(1,3),'MarkerFaceColor','y');                    % Plot initial position
CoordinateX=num2str(Leader(1,2));                                          % Initial position x - axis string format
CoordinateY=num2str(Leader(1,3));                                          % Initial position y - axis string format
StartCoordinates=strcat(' S(',CoordinateX,',',CoordinateY,')');            % Initial position text format
text(Leader(1,2),Leader(1,3),StartCoordinates,'Color','k');                % Plot (text) initial position coordinates
scatter(Leader(end,2),Leader(end,3),'MarkerFaceColor','y');                % Plot final position
CoordinateX=num2str(Leader(end,2));                                        % Final position x - axis string format
CoordinateY=num2str(Leader(end,3));                                        % Final position y - axis string format
EndCoordinates=strcat(' G(',CoordinateX,',',CoordinateY,')');              % Final position text format
text(Leader(end,2),Leader(end,3),EndCoordinates,'Color','k');              % Plot (text) final position coordinates
plot(Leader(:,2),Leader(:,3),'Color','red');                               % Plot suboptimal agent trajectory
savefig(FigureX0A,'FigureX0A.fig');

% 5. Plots | 5.2. Leader trajectory,detection areas and initial time window
FigureX0B=figure('Name','Sub optimal agent trajectory,detection areas and initial time window');title('Sub optimal agent trajectory,detection areas and initial time window');
xlim([PlotXLow PlotXUpp]);ylim([PlotYLow PlotYUpp]);xlabel('x');ylabel('y');   % Figure settings for axis
hold on;
rectangle('Position',[xfa yfa (xfb-xfa) (yfb-yfa)],'EdgeColor','k');           % Plot problem target area 
plot(Leader(:,2),Leader(:,3),'r');                                             % Plot suboptimal agent trajectory from start to goal
scatter(xi,yi,'MarkerFaceColor','y');                                          % Plot initial point (common for all agents)   
scatter(Leader(end,2),Leader(end,3),'MarkerFaceColor','y');                    % Plot final point
scatter(TW(1,8),TW(1,9),'MarkerFaceColor','r');                                % Plot agent position at delta (intepolated or not)
rectangle('Position',[xi-LeaDecRad yi-LeaDecRad 2*LeaDecRad 2*LeaDecRad],'EdgeColor','r'); % Plot initial detection area
PointTemp=find(Leader(:,1)<=delta);                                                        % Find the leader's position after ? time units
PointA=PointTemp(end);                                                                     % Former point  
for i=2:PointA                                                                             % Plot the detection area around each leader's position
     rectangle('Position',[Leader(i,4) Leader(i,6) 2*LeaDecRad 2*LeaDecRad],'EdgeColor','k');
end
rectangle('Position',[TW(1,8)-LeaDecRad TW(1,9)-LeaDecRad 2*LeaDecRad 2*LeaDecRad],'EdgeColor','r');                          % Plot detection area at delta (intepolated or not)
rectangle('Position',[TW(1,12) TW(1,14) TW(1,13)-TW(1,12) TW(1,15)-TW(1,14)],'EdgeColor','r' ,'LineWidth',1,'LineStyle','--') % Plot time window in 2D
scatter(Leader(1,2),Leader(1,3),'MarkerFaceColor','y');                    % Plot initial point
CoordinateX=num2str(Leader(1,2));                                          % Initial point x - axis string format
CoordinateY=num2str(Leader(1,3));                                          % Initial point y - axis string format
StartCoordinates=strcat(' S(',CoordinateX,',',CoordinateY,')');            % Final point text format
text(Leader(1,2),Leader(1,3),StartCoordinates,'Color','k');                % Plot initial point coordinates
scatter(Leader(end,2),Leader(end,3),'MarkerFaceColor','y');                % Plot final point
CoordinateX=num2str(Leader(end,2));                                        % Final point x - axis string format
CoordinateY=num2str(Leader(end,3));                                        % Final point y - axis string format
EndCoordinates=strcat(' G(',CoordinateX,',',CoordinateY,')');              % Final point text format
text(Leader(end,2),Leader(end,3),EndCoordinates,'Color','k');              % Plot initial point coordinates
savefig(FigureX0B,'FigureX0B.fig');
clear PointTemp PointA i CoordinateX CoordinateY EndingCoordinates;

% 5. Plots | 5.3. Leader trajectory,detection areas and initial time window - 3D
FigureX0C=figure('Name','Sub optimal agent trajectory,detection areas and initial time window');
xlim([PlotXLow PlotXUpp]);ylim([PlotYLow PlotYUpp]);zlim([0 Leader(end,1)]);xlabel('x');ylabel('y');zlabel('t');    % Figure settings for suboptimal agent 3D
plot3(Leader(:,2),Leader(:,3),Leader(:,1),'Color','red');                                                           % Plot suboptimal agent trajectory from start to goal - 3D
hold on;
scatter3(xi,yi,TW(1,2),'MarkerFaceColor','y');                             % Plot initial point - 3D
xmin=xfa;xmax=xfb;ymin=yfa;ymax=yfb;                                       % Plot problem target area - 3D
X=[xmin xmin xmax xmax xmin];Y=[ymin ymax ymax ymin ymin];                 % Add extra xmin ymin so to plot the last line
T=[Leader(end,1) Leader(end,1) Leader(end,1) Leader(end,1) Leader(end,1)];
plot3(X,Y,T,'Color','k');
scatter3(Leader(end,2),Leader(end,3),Leader(end,1),'MarkerFaceColor','y')  % Plot suboptimal agent final point - 3D
scatter3(TW(1,8),TW(1,9),TW(1,3),'MarkerFaceColor','r');                   % Plot suboptimal agent position at delta (intepolated or not)
clear xmin xmax ymin ymax X Y T ;

xmin=xi-LeaDecRad;xmax=xi+LeaDecRad;ymin=yi-LeaDecRad;ymax=yi+LeaDecRad;   % Plot initial detection area - 3D
X=[xmin xmin xmax xmax xmin];Y=[ymin ymax ymax ymin ymin];
T=[TW(1,2) TW(1,2) TW(1,2) TW(1,2) TW(1,2)];
plot3(X,Y,T,'Color','r');

PointTemp=find(Leader(:,1)<=delta);             % Find the leader's position after ? time units
PointA=PointTemp(end);                          % Former point 
for i=2:PointA                                  % Plot detection areas in 3D to be included in the box, i.e. time window
     xmin=Leader(i,4);xmax=Leader(i,5);
     ymin=Leader(i,6);ymax=Leader(i,7);
     X=[xmin xmin xmax xmax xmin];Y=[ymin ymax ymax ymin ymin];
     T=[Leader(i,1) Leader(i,1) Leader(i,1) Leader(i,1) Leader(i,1)];
     plot3(X,Y,T,'Color','k');
end
xmin=TW(1,8)-LeaDecRad;xmax=TW(1,8)+LeaDecRad;   % Plot detection area at delta (intepolated or not)
ymin=TW(1,9)-LeaDecRad;ymax=TW(1,9)+LeaDecRad;
X=[xmin xmin xmax xmax xmin];Y=[ymin ymax ymax ymin ymin];
T=[delta delta delta delta delta];
plot3(X,Y,T,'Color','r');

xmin=TW(1,12);xmax=TW(1,13);                                                % Get time window coordinates
ymin=TW(1,14);ymax=TW(1,15);
X=[xmin xmin xmax xmax xmin];Y=[ymin ymax ymax ymin ymin];                  % Plot time window
T=[TW(1,2) TW(1,2) TW(1,2) TW(1,2) TW(1,2)];                                % Plot time window bottom
plot3(X,Y,T,'Color','r','LineStyle','--');                                                                                        
T=[delta delta delta delta delta];                                          % Plot time window top                                           
plot3(X,Y,T,'Color','r','LineStyle','--');                                                                                      
line([xmin xmin],[ymin ymin],[TW(1,2) delta],'Color','r','LineStyle','--'); % Plot time window sides
line([xmin xmin],[ymax ymax],[TW(1,2) delta],'Color','r','LineStyle','--'); 
line([xmax xmax],[ymax ymax],[TW(1,2) delta],'Color','r','LineStyle','--');
line([xmax xmax],[ymin ymin],[TW(1,2) delta],'Color','r','LineStyle','--');

CoordinateX=num2str(Leader(1,2));                                          % Initial point x - axis string format
CoordinateY=num2str(Leader(1,3));                                          % Initial point y - axis string format          
CoordinateT=num2str(Leader(1,1));                                          % Initial point z - axis string format
StartCoordinates=strcat(' S(',CoordinateX,',',CoordinateY,',',CoordinateT,')');
text(Leader(1,2),Leader(1,3),Leader(1,1),StartCoordinates,'Color','k');        % Plot follower final position coordinates
CoordinateX=num2str(Leader(end,2));                                            % Initial point z - axis string format
CoordinateY=num2str(Leader(end,3));                                            % Initial point x - axis string format
CoordinateT=num2str(Leader(end,1));                                            % Initial point y - axis string format          
EndCoordinates=strcat(' G(',CoordinateX,',',CoordinateY,',',CoordinateT,')');
text(Leader(end,2),Leader(end,3),Leader(end,1),EndCoordinates,'Color','k');   % Plot follower final position coordinates
title('Suboptimal agent trajectory,detection areas and initial time window');
savefig(FigureX0C,'FigureX0C.fig');
clear PointTemp PointA i xmin xmax ymin ymax  X Y T CoordinateX CoordinateY CoordinateT StartCoordinates EndCoordinates;

% 5. Plots | 5.4. Sub optimal agent and obstacle realtime progression
if PlotRealTimeProgression
   FigureX0D=figure('Name','Sub optimal agent and obstacle real time progression');title('Sub optimal agent and obstacle real time progression');
   xlim([PlotXLow PlotXUpp]);ylim([PlotYLow PlotYUpp]);xlabel('x');ylabel('y');   % Figure settings for axis
   hold on;
   scatter(xi,yi,'MarkerFaceColor','y');                                   % Agent initial position
   theta = linspace(0,2*pi,100);
   [m,n]=size(Obstacle);
   scatter(Leader(1,2),Leader(1,3),'MarkerFaceColor','y');                 % Plot initial point
   CoordinateX=num2str(Leader(1,2));                                       % Initial point x - axis string format
   CoordinateY=num2str(Leader(1,3));                                       % Initial point y - axis string format
   StartCoordinates=strcat(' S(',CoordinateX,',',CoordinateY,')');         % Final point text format
   text(Leader(1,2),Leader(1,3),StartCoordinates,'Color','k');             % Plot initial point coordinates
   scatter(Leader(end,2),Leader(end,3),'MarkerFaceColor','y');             % Plot final point
   CoordinateX=num2str(Leader(end,2));                                     % Final point x - axis string format
   CoordinateY=num2str(Leader(end,3));                                     % Final point y - axis string format
   EndCoordinates=strcat(' G(',CoordinateX,',',CoordinateY,')');           % Final point text format
   text(Leader(end,2),Leader(end,3),EndCoordinates,'Color','k');           % Plot initial point coordinates
   v=VideoWriter('D_SubOptimal_Progression.avi');
   open(v);
   for i=1:m
        h1=rectangle('Position',[Obstacle(i,5) Obstacle(i,7) Obstacle(i,6)-Obstacle(i,5) Obstacle(i,8)-Obstacle(i,7)]);
        h2=scatter(Leader(i,2),Leader(i,3),'MarkerFaceColor','r');
        F(i)=getframe(gcf);
        writeVideo(v,F(i));
        if i~=m
           delete(h1);
           delete(h2);
        end
    end
close(v);
end
clear theta m n i x0 y0 R F h1 h2 h3 CoordinateX CoordinateY StartCoordinates EndCoordinates;

% 5. Plots | 5.5. Leader detection areas and intersections real time progression
if PlotRealTimeProgression
  FigureX0E=figure('Name','Sub optimal agent detection areas and intersections real time progression');title('Sub optimal agent detection areas and intersections real time progression');
  xlim([PlotXLow PlotXUpp]);ylim([PlotYLow PlotYUpp]);xlabel('x');ylabel('y');           % Figure settings for axis
  hold on;
  rectangle('Position',[xfa yfa (xfb-xfa) (yfb-yfa)],'EdgeColor','k');                   % Plot problem target area 
  [m,n]=size(LeaObsIntPoly);
  [k,l]=size(Obstacle);
  theta = linspace(0,2*pi,100);
  v=VideoWriter('X0_TimeWindowCreationRealTime.avi');
  v.FrameRate=5;
  open(v);
  for i=1:m
       x0=Obstacle(i,2);y0=Obstacle(i,3);R0=Obstacle(i,4);                              % Get obstacle position and radius
       x1=Leader(i,2);y1=Leader(i,3);R1=LeaDecRad;                                      % Get leader position (the radius is always LeaDecRad)
       h1=plot(LeaObsIntPoly(i,1),'FaceColor','r','FaceAlpha',1);                       % Plot obstacle bounding box
       h2=plot(x0 + R0*cos(theta),y0 + R0*sin(theta),'-k');                             % Plot obstacle circle without safety radius                            
       h3=scatter(x0,y0,'filled','MarkerFaceColor','k');                                % Plot obstacle center
       h4=plot(LeaObsIntPoly(i,2),'FaceColor','b','FaceAlpha',1);                       % Plot leader bounding box
       h5=scatter(Leader(i,2),Leader(i,3),'filled','MarkerFaceColor','k');              % Plot leader center
       h6=plot(x1 + R1*cos(theta),y1 + R1*sin(theta),'-k');                             % Plot leader detection circle
       h7=plot(x0 + (R0+ObsSafRad)*cos(theta),y0 + (R0+ObsSafRad)*sin(theta),'--k');    % Plot obstacle with safety radius
       h8=plot(LeaObsIntPoly(i,3),'FaceColor','g');                                     % Plot intersections
       F(i)=getframe(gcf);
       writeVideo(v,F(i));
       if i~=k
          delete(h2);delete(h6);delete(h7);
       end
  end
scatter(Leader(1,2),Leader(1,3),'MarkerFaceColor','y');                    % Plot initial point
CoordinateX=num2str(Leader(1,2));                                          % Initial point x - axis string format
CoordinateY=num2str(Leader(1,3));                                          % Initial point y - axis string format
StartCoordinates=strcat(' S(',CoordinateX,',',CoordinateY,')');            % Final point text format
text(Leader(1,2),Leader(1,3),StartCoordinates,'Color','k');                % Plot initial point coordinates
scatter(Leader(end,2),Leader(end,3),'MarkerFaceColor','y');                % Plot final point
CoordinateX=num2str(Leader(end,2));                                        % Final point x - axis string format
CoordinateY=num2str(Leader(end,3));                                        % Final point y - axis string format
EndCoordinates=strcat(' G(',CoordinateX,',',CoordinateY,')');              % Final point text format
text(Leader(end,2),Leader(end,3),EndCoordinates,'Color','k');              % Plot initial point coordinates
close(v);
clear m n k l theta i h1 h2 h3 h4 h5 h6 h7 h8 LeaObsIntPoly R0 R1 F CoordinateX CoordinateY EndingCoordinates; 
end

SubOptimalSolutionTime=solver.output.cpu;
save SubOptimalSolutionTime SubOptimalSolutionTime;
% ------------------------------------------------------------------------------------------------------------------------------------------------------------------
% End of Agent x_0 Main
% ------------------------------------------------------------------------------------------------------------------------------------------------------------------

% Clear Mex files and ObstacleConstraints.mat
rmdir C:\FALCON.m.v1.24.2002191427\falcon\SetupA_Final\fm_constraints s
rmdir C:\FALCON.m.v1.24.2002191427\falcon\SetupA_Final\fm_models s
delete *.mexw64
delete ObstacleConstraintsSubOptimal.mat

% End of file