clear;clc;close all;                          % Clear workspace
% ------------------------------------------------------------------------------------------------------------------------------------------------------------------
% Load data from leader
% ------------------------------------------------------------------------------------------------------------------------------------------------------------------
load AlgorithmParametersX5.mat;               % Load save A
load LeaderX5.mat;                            % Load save B
load ObstacleX5.mat;                          % Load save C
load TimeWindowX5.mat;                        % Load save D 
load IntersectionPolyshapeX5.mat              % Load save E
load DetectionWindowX5.mat;                   % Load save F
load IntersectionPolyshapeX5P.mat             % Load save E (Puddle)
load ObstacleX5P.mat;                         % Load save C (Puddle)
load DetectionWindowX5P.mat;                  % Load save F (Puddle)
load TotalResultsX5.mat;                      % Load save G
load TotalCost.mat;                           % Load save H
load SolverStats.mat                          % Load save I
% ------------------------------------------------------------------------------------------------------------------------------------------------------------------
% End of Load data from leader
% ------------------------------------------------------------------------------------------------------------------------------------------------------------------

% ------------------------------------------------------------------------------------------------------------------------------------------------------------------
% Agent x_6 initialization
% ------------------------------------------------------------------------------------------------------------------------------------------------------------------
MaxTime=60;                                  % Maximum time per problem
PlotRealTimeProgression=0;                   % Plot current agent realtime progression (next agent)
OptimizeInDelta=0;                           % Restrict time to [s,s+Ä], i.e. the time window frame
OptimizeInTW=1;                              % Restrict movement only within the bounding grid of the time window
OptimizeCostFunction=1;                      % 0 for optimizing time only else optimize time, energy and control 
ArrivalVelocity=3;                           % Set to 0 or 1 or else use the settings provided

% 1. Speed and acceleration (control) limits for x_k k>=1
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

if ArrivalVelocity==0
   vxfa=0;vxfb=0;
   vyfa=0;vyfb=0;
elseif ArrivalVelocity==1
   vxfa=-1;vxfb=1;                                             
   vyfa=-1;vyfb=1;
else
   vxfa=-VelocitySetting;vxfb=VelocitySetting;
   vyfa=-VelocitySetting;vyfb=VelocitySetting;
end

% 2. Time discretization and Tolernce levels
TimeLowerLimit=0;                                                          % The initial time window corresponds to [0,Ä]
if OptimizeInDelta                                                         % Restrict optimization time to [s,s+Ä], i.e. for the time window frame
   TimeLimitStep=delta;   
else
   TimeLimitStep=MaxTime;
end
TimeUpperLimit=TimeLowerLimit+TimeLimitStep;
TimeInitialGuess=(TimeLowerLimit+TimeUpperLimit)/2; 
FeasabilityTolerance=1e-5;                                                  % Feasibility tolerance for the IPOPT solver
OptimalityTolerance=1e-5;                                                   % Optimality tolerance for the IPOPT solver

% 3. States and controls scaling 
MinStateX=0;MaxStateX=50;
MinStateY=0;MaxStateY=300;

% 3-4-5-6: Dynamics time grid discretization,initial phase time and tolerance levels
% We use the same with the suboptimal agent
% 7. Initial boundaries
% All agents depart from the same point stored in the time window variable (TW)
% 8. Final boundaries
% The first final boundary is the leader's location in the first time window stored in variable (TW)

% 10, Algorithm related
PhaseCounter=1;                         % Used to find entries in the time window
AgentCounterRow=AgentCounterRow+1;      % Each agent corresponds to one row in TotalResults
AgentCounterColumn=1;                   % Each column corresponds to an iteration for agent in row AgentCounterRow
EndFlag=false;                          % Signals the end of iterations

% 11. Plot limits                                              
PlotXLow=-50;PlotXUpp=150;
PlotYLow=0;PlotYUpp=300;

% 12. Record figure to video
v=VideoWriter('X6_Progression.avi');
v.FrameRate=1;
open(v);

save AlgorithmParametersX6.mat;        % Save setting so to be used by all x_k,k>=1 - Save A
% ------------------------------------------------------------------------------------------------------------------------------------------------------------------
% End of Agent x_6 initialization
% ------------------------------------------------------------------------------------------------------------------------------------------------------------------

% ------------------------------------------------------------------------------------------------------------------------------------------------------------------
% Plot initial time window Leader(X5) - Follower(X6) 
% ------------------------------------------------------------------------------------------------------------------------------------------------------------------
FigureX6A=figure('Name','Leader: Agent X5 | Follower: Agent X6');
plot3(Leader(:,2),Leader(:,3),Leader(:,1),'Color','red');                                                                 
title('Leader: Agent X5 | Follower: Agent X6');
axis([PlotXLow PlotXUpp PlotYLow PlotYUpp 0 Leader(end,1)]);
xlabel('x');ylabel('y');zlabel('t');
hold on;

% Plot initial position and coordinates (common for all agents)
scatter3(xi,yi,0,'MarkerFaceColor','y');
CoordinateX=num2str(xi);CoordinateY=num2str(yi);CoordinateT=num2str(0);
LeaderAtStartCoordinates=strcat(' S(',CoordinateX,',',CoordinateY,',',CoordinateT,')');
text(xi,yi,0,LeaderAtStartCoordinates,'Color','k');
clear CoordinateX CoordinateY CoordinateT LeaderAtStartCoordinates;

% Plot leader at delta position and coordinates
scatter3(TW(PhaseCounter,8),TW(PhaseCounter,9),TW(PhaseCounter,3),'MarkerFaceColor','r');
CoordinateX=num2str(TW(PhaseCounter,8));CoordinateY=num2str(TW(PhaseCounter,9));CoordinateT=num2str(TW(PhaseCounter,3));
LeaderAtDeltaCoordinates=strcat(' L(',CoordinateX,',',CoordinateY,',',CoordinateT,')');
h1=text(TW(PhaseCounter,8),TW(PhaseCounter,9),TW(PhaseCounter,3),LeaderAtDeltaCoordinates,'Color','r');
clear CoordinateX CoordinateY CoordinateT LeaderAtDeltaCoordinates;

% Plot leader final position and coordinates
scatter3(Leader(end,2),Leader(end,3),Leader(end,1),'MarkerFaceColor','y');

% Plot target area
xmin=xfa;xmax=xfb;ymin=yfa;ymax=yfb;
X=[xmin xmin xmax xmax xmin];Y=[ymin ymax ymax ymin ymin];
T=[Leader(end,1) Leader(end,1) Leader(end,1) Leader(end,1) Leader(end,1)];
plot3(X,Y,T,'Color','k');
clear xmin xmax ymin ymax X Y T;

% Plot follower initial position and coordinates
scatter3(TW(PhaseCounter,4),TW(PhaseCounter,5),TW(PhaseCounter,2),'MarkerFaceColor','b');
CoordinateX=num2str(TW(PhaseCounter,4));CoordinateY=num2str(TW(PhaseCounter,5));CoordinateT=num2str(TW(PhaseCounter,2));
FollowerAtStartCoordinates=strcat(' F(',CoordinateX,',',CoordinateY,',',CoordinateT,')');
h2=text(TW(PhaseCounter,4),TW(PhaseCounter,5),TW(PhaseCounter,2),FollowerAtStartCoordinates,'Color','b');
clear CoordinateX CoordinateY CoordinateT FollowerAtStartCoordinates;

% Plot intersections bounding boxes
[m,n]=size(LeaObsDetTW);
if m~=0
   for i=1:m
        xmin=LeaObsDetTW(i,16);xmax=LeaObsDetTW(i,17);ymin=LeaObsDetTW(i,18);ymax=LeaObsDetTW(i,19);      % Get intersection 
        X=[xmin xmin xmax xmax xmin];Y=[ymin ymax ymax ymin ymin];                                                                      
        T=[LeaObsDetTW(i,1) LeaObsDetTW(i,1) LeaObsDetTW(i,1) LeaObsDetTW(i,1) LeaObsDetTW(i,1)];         % Plot intersection
        PlotsIntersections{i,1}=plot3(X,Y,T,'Color','r','LineStyle','-');                                 % Cell to store intersection plots so to dynamically redraw
   end
else
        PlotsIntersections={ };
end
clear m n i xmin xmax ymin ymax X Y T;

% Plot per pair intersections bounding boxes
[m,n]=size(LeaObsDetTW);
if m==0                                                                    % If there were no intersections
   PlotsPerPairIntersections={ };  
elseif m==1                                                                % If there is only one intersection
   xmin=LeaObsDetTW(1,16);xmax=LeaObsDetTW(1,17);ymin=LeaObsDetTW(1,18);ymax=LeaObsDetTW(1,19);  % Get intersection   
   Poly=polyshape([xmin xmin xmax xmax],[ymin ymax ymax ymin]);                                  % Polyshape for the intersection
   [XLimBou,YLimBou] = boundingbox(Poly);                                                        % Bounding box for the intersection
   xmin=XLimBou(1);xmax=XLimBou(2);ymin=YLimBou(1);ymax=YLimBou(2);        % Prepare intersection coordinates for minimum bounding ellipse function
   P=[[xmin;ymin] [xmin;ymax] [xmax;ymax] [xmax;ymin]];
   [A,C]=MinVolEllipse(P,0.00001);                                         % Get a,b and coordinates for the center of the minimum bounding ellipse
   [a,b,C,X]=Ellipse_plot(A,C);
   [k l]=size(X);
   temp=repmat(LeaObsDetTW(1,1),1,l);   
   PlotsPerPairIntersections{1,1}=plot3(X(1,:),X(2,:),temp,'y','LineWidth',1.5);    % Plot midpoint intersections
else                                                                                % If there are more than one intersections
  for i=1:m-1
       xmin=LeaObsDetTW(i,16);xmax=LeaObsDetTW(i,17);ymin=LeaObsDetTW(i,18);ymax=LeaObsDetTW(i,19);           % Get first intersection
       PolyLowerTime=polyshape([xmin xmin xmax xmax],[ymin ymax ymax ymin]);                                  % Polyshape for first intersection
       xmin=LeaObsDetTW(i+1,16);xmax=LeaObsDetTW(i+1,17);ymin=LeaObsDetTW(i+1,18);ymax=LeaObsDetTW(i+1,19);   % Get second intersection
       PolyUpperTime=polyshape([xmin xmin xmax xmax],[ymin ymax ymax ymin]);                                  % Polyshape for second intersection
       PolyUnion(i,1)=union(PolyLowerTime,PolyUpperTime);                                                     % Union of the two intersections
       [XLimBou,YLimBou] = boundingbox(PolyUnion(i,1));                                                       % Bounding box for the union
       xmin=XLimBou(1);xmax=XLimBou(2);ymin=YLimBou(1);ymax=YLimBou(2);                                       % Prepare intersection coordinates for minimum bounding ellipse function
       P=[[xmin;ymin] [xmin;ymax] [xmax;ymax] [xmax;ymin]];
       [A,C]=MinVolEllipse(P,0.00001);                                                                        % Get a,b and coordinates for the center of the minimum bounding ellipse
       [a,b,C,X]=Ellipse_plot(A,C);
       [k l]=size(X);
       temp=repmat((LeaObsDetTW(i,1)+LeaObsDetTW(i+1,1))/2,1,l);   
       PlotsPerPairIntersections{i,1}=plot3(X(1,:),X(2,:),temp,'y','LineWidth',1.5);                          % Plot midpoint intersections
  end
end
clear m n xmin xmax ymin ymax Poly PolyLowerTime PolyUpperTime PolyUnion XLimBou YLimBou P A C a b C X k l temp ;
clear LeaObsDetTW LeaObsIntPoly;
% ------------------------------------------------------------------------------------------------------------------------------------------------------------------
% End of Plot leader and get handles
% ------------------------------------------------------------------------------------------------------------------------------------------------------------------

% ------------------------------------------------------------------------------------------------------------------------------------------------------------------
% x_6 Main
% ------------------------------------------------------------------------------------------------------------------------------------------------------------------
while (1)
% A. Define state space and controls 
% State vector: Position axis - x (x), Position axis - y (y), Velocity axis - x (vx), Velocity axis - y (vy) and dummy state (time)
if OptimizeInTW==1
    if TW(PhaseCounter,12)<RoadLowerLimitX
        x_vec= [...
        falcon.State('x',RoadLowerLimitX,TW(PhaseCounter,13),RoadScalingX);...
        falcon.State('y',TW(PhaseCounter,14),TW(PhaseCounter,15),RoadScalingY);...
        falcon.State('vx',LowerVelocityLimitX,UpperVelocityLimitX,VelocityScaling);...
        falcon.State('vy',LowerVelocityLimitY,UpperVelocityLimitY,VelocityScaling);...
        falcon.State('time',TimeLowerLimit,TimeUpperLimit,TimeScaling)];
    elseif TW(PhaseCounter,13)>RoadUpperLimitX
        x_vec= [...
        falcon.State('x',TW(PhaseCounter,12),RoadUpperLimitX,RoadScalingX);...
        falcon.State('y',TW(PhaseCounter,14),TW(PhaseCounter,15),RoadScalingY);...
        falcon.State('vx',LowerVelocityLimitX,UpperVelocityLimitX,VelocityScaling);...
        falcon.State('vy',LowerVelocityLimitY,UpperVelocityLimitY,VelocityScaling);...
        falcon.State('time',TimeLowerLimit,TimeUpperLimit,TimeScaling)];
    else
        x_vec= [...
        falcon.State('x',TW(PhaseCounter,12),TW(PhaseCounter,13),RoadScalingX);...
        falcon.State('y',TW(PhaseCounter,14),TW(PhaseCounter,15),RoadScalingY);...
        falcon.State('vx',LowerVelocityLimitX,UpperVelocityLimitX,VelocityScaling);...
        falcon.State('vy',LowerVelocityLimitY,UpperVelocityLimitY,VelocityScaling);...
        falcon.State('time',TimeLowerLimit,TimeUpperLimit,TimeScaling)];
    end
else
x_vec = [...
    falcon.State('x',MinStateX,MaxStateX,0.1);...
    falcon.State('y',MinStateY,MaxStateY,0.01);...
    falcon.State('vx',LowerVelocityLimitX,UpperVelocityLimitX,VelocityScaling);...
    falcon.State('vy',LowerVelocityLimitY,UpperVelocityLimitY,VelocityScaling);...
    falcon.State('time',TimeLowerLimit,TimeUpperLimit,TimeScaling)];
end

 % Control vector: Acceleration axis - x (ax), Acceleration axis - y (ay)
u_vec = [...                                                                                              
    falcon.Control('ax',LowerControlLimitX,UpperControlLimitX,AccelerationScaling);...      
    falcon.Control('ay',LowerControlLimitY,UpperControlLimitY,AccelerationScaling)];

% B. Final time
tf = falcon.Parameter('FinalTime', TimeInitialGuess, TimeLowerLimit, TimeUpperLimit,TimeStep); 

% C. Discretization
tau = linspace(0,DynamicsDiscretizationStep,DynamicsDiscretizationUpperLimit);

% D. Problem: Create a new Instance
problem = falcon.Problem('Ants');

% E. Problem: Add a new phase
phase = problem.addNewPhase(@SwarmProblemDefinition, x_vec, tau, PhaseInitialTime, tf);

% F. Phase: Add control grid
phase.addNewControlGrid(u_vec, tau);

% G. Phase: Add model output
phase.Model.setModelOutputs([falcon.Output('x_out'); falcon.Output('y_out');falcon.Output('vx_out');falcon.Output('vy_out');falcon.Output('time_out');falcon.Output('ax_out');falcon.Output('ay_out')]);

% H. Phase: Add Lagrange cost function
if OptimizeCostFunction==0
   problem.addNewParameterCost(tf);  
else
   Cost=falcon.Cost('Cost');
   phase.addNewLagrangeCost(@SwarmCostFuction,Cost,tau);
end
  
% Pass on needed obstacle information via saving .mat file and loading it later on in the path constraint function
save ObstacleConstraints.mat ObsIniX ObsIniY ObsIniR SpeedX SpeedY SpeedR ObsSafRad Divider LeaDecRad LeaObsDetCon LeaObsDetCon2 CeX CeY CeR;

% Add predictions as state space constraints
[m,n]=size(LeaObsDetCon);
[m1,n1]=size(LeaObsDetCon2);
if OptimizeInTW>0 
   ConChecker=1;   % If OptimizeInTW=1 then predictions are based either by using only the TW or combining the TW with the wrapper
else
   ConChecker=0;   % If OptimizeInTW=0 then predictions are produced only by the the wrapper     
end

if ConChecker==0
% ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
% First case: If only the wrapper's predictions are used (In this implementation OptimizeInTW=1 so ConChecker=1 and this is never activated)
% ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
% J. Phase: Add constraints (obstacles)
pconMdl = falcon.PathConstraintBuilder('AntsPCon',[],x_vec,u_vec,[],@SwarmConstraintsWithoutTW);     % Create a model to add the constraints
pathconstraints=[falcon.Constraint('ObstacleRealTime',0,inf)];                                    % Predictions produced by the wrapper
phase.addNewPathConstraint(@AntsPCon, pathconstraints, tau);                                      % Add all the constraints to the phase
pconMdl.Build();
% ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
% End of First case
% ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
else 
% ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
% Second case: If predictions are baseed solely on the TW or by combinining it with the wrapper
% ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
% J. Phase: Add constraints (obstacles)
if ~isempty(LeaObsDetCon) && isempty(LeaObsDetCon2)
    Go=1;
elseif isempty(LeaObsDetCon) && ~isempty(LeaObsDetCon2)
    Go=2;
elseif ~isempty(LeaObsDetCon) && ~isempty(LeaObsDetCon2)
    Go=3;
else
    Go=0;
end

% pathconstraints=[falcon.Constraint('ObstacleRealTime',0,inf)];
% Uncomment if the combination of TW and planner is to be used, In this implementation this is always commented so only the TW is used
% Currently not used
if Go==1
   pconMdl = falcon.PathConstraintBuilder('AntsPCon',[],x_vec,u_vec,[],@SwarmConstraintsWithTW);   
   pathconstraints=[ ];
   for i=1:m                                                                                                 
       pathconstraints=[pathconstraints;falcon.Constraint(strcat('ObstacleTimeWindow_',int2str(i)),0,inf)];
   end
   phase.addNewPathConstraint(@AntsPCon, pathconstraints, tau);                                    
   pconMdl.Build();        
elseif Go==2
   pconMdl = falcon.PathConstraintBuilder('AntsPCon',[],x_vec,u_vec,[],@SwarmConstraintsWithTWOnlyLandscape);
   pathconstraints=[ ];
   for i=1:4*m1                                                                                                 
       pathconstraints=[pathconstraints;falcon.Constraint(strcat('ObstacleTimeWindow_',int2str(i)),0,inf)];
   end
   phase.addNewPathConstraint(@AntsPCon, pathconstraints, tau);                                    
   pconMdl.Build();              
elseif Go==3
   pconMdl = falcon.PathConstraintBuilder('AntsPCon',[],x_vec,u_vec,[],@SwarmConstraintsLandScapeWithTW);
   pathconstraints=[ ];
   for i=1:m+4*m1                                                                                               
       pathconstraints=[pathconstraints;falcon.Constraint(strcat('ObstacleTimeWindow_',int2str(i)),0,inf)];
   end
   phase.addNewPathConstraint(@AntsPCon, pathconstraints, tau);                               
   pconMdl.Build();         
end
clear Go;    

end
% ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
% End of Second case
% ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
clear m n m1 n1 i ConChecker;

% I. Phase Add boundary conditions
phase.setInitialBoundaries([TW(PhaseCounter,4);TW(PhaseCounter,5);TW(PhaseCounter,6);TW(PhaseCounter,7);TimeLowerLimit]);
xfa=TW(PhaseCounter,8);xfb=TW(PhaseCounter,10);           % Leader's position x or area if it's in C_p
yfa=TW(PhaseCounter,9);yfb=TW(PhaseCounter,11);           % Leader's position y or area if it's in C_p
phase.setFinalBoundaries([x_vec(1);x_vec(2);x_vec(3);x_vec(4)],[xfa;yfa;vxfa;vyfa],[xfb;yfb;vxfb;vyfb]); % The final control is always in [LowerControlLimitX,UpperControlLimitX] and [LowerControlLimitY,UpperControlLimitY]

% K. Problem: Prepare for solving
problem.Bake();

% L. Problem: Solve 
solver = falcon.solver.ipopt(problem);                      % IPOPT solver
solver.Options.MajorIterLimit = 3000;                       % Maximum number of iterations
solver.Options.MajorFeasTol = FeasabilityTolerance;         % Feasibility tolerance
solver.Options.MajorOptTol = OptimalityTolerance;           % Optimality tolerance
solver.Solve();                                                                        % Solve command
SolverStats{AgentCounterRow-1,AgentCounterColumn}=[solver.output.status solver.output.iter solver.output.cpu]; % Save solver output

% Final iteration plots and collect results
if EndFlag   
   plot3(phase.StateGrid.Values(1,:),phase.StateGrid.Values(2,:),phase.RealTime,'b');                                      % Plot follower's trajectory to C_P - 3D
   delete(h2);
   scatter3(phase.StateGrid.Values(1,end),phase.StateGrid.Values(2,end),phase.RealTime(end),'MarkerFaceColor','b');        % Plot follower final point
   CoordinateX=num2str(phase.StateGrid.Values(1,end));CoordinateY=num2str(phase.StateGrid.Values(2,end));                  % Follower x,y and t to string format
   CoordinateT=num2str(phase.RealTime(end)); 
   EndingCoordinates=strcat(' F(',CoordinateX,',',CoordinateY,',',CoordinateT,')');
   text(phase.StateGrid.Values(1,end),phase.StateGrid.Values(2,end),phase.RealTime(end),EndingCoordinates,'Color','b','HorizontalAlignment','right');   % Plot follower final position coordinates
   t=phase.RealTime;                                                       % Store time in TotalResults
   sx=phase.StateGrid.Values(1,:);                                         % Store trajectory in TotalResults
   sy=phase.StateGrid.Values(2,:);
   svx=phase.StateGrid.Values(3,:);
   svy=phase.StateGrid.Values(4,:);
   cax=phase.ControlGrids.Values(1,:);
   cay=phase.ControlGrids.Values(2,:);
   TJ=phase.LagrangeCostFunctions.OutputGrid.Values; % Store Lagrange cost functions in TotalResults
   TotalResults{AgentCounterRow,AgentCounterColumn}=[t' sx' sy' svx' svy' cax' cay' TJ']; % Final TotalResults format
   [m,n]=size(TotalResults);
   j=1;
   for i=1:n
        if ~isempty(TotalResults{AgentCounterRow,i})
           PartialCosts(1,j)=sum(TotalResults{AgentCounterRow,i}(:,end));
           j=j+1;
        end
   end
   PartialCosts(1,end+1)=sum(PartialCosts(1,:));
   TotalCost{AgentCounterRow-1,1}=PartialCosts(1,1:end-1);
   TotalCost{AgentCounterRow-1,2}=PartialCosts(1,end);
   save TotalCost.mat TotalCost;                      %  Store total costs - Save H
   AgentPlot=getframe(gcf);
   writeVideo(v,AgentPlot);
   clear j i h1 h2 PlotsA PlotsB PlotsC PlotsD sx sy svx svy cax cay TJ m n i PartialCosts;
   break;
end

% Next iteration | A.1. Next problem time window frame depends on whether the sampling point was s or tf (whatever comes first)
TW(PhaseCounter,17)=phase.FinalTime.Value;                                 % Save the final time from the optimization in the current time window
TW(PhaseCounter+1,1)=PhaseCounter+1;                                       % Each line in the TW variable corresponds to a new time window for the follower
FlagTf=false;                                                              % Signals whether tf was smaller than sampling or not
if (TW(PhaseCounter,2)+sampling)<=phase.FinalTime.Value                    % The sampling period may be less or equal than the arrival time. In this case, the agent will construct 
   TW(PhaseCounter+1,2)=TW(PhaseCounter,2)+sampling;                       % the next window at the next sampling
else                                                                       % If the final time is less than the sampling the next time window will be constructed at the final time
   TW(PhaseCounter+1,2)=phase.FinalTime.Value;                             % rather than at the next sampling  
   FlagTf=true;                                                            % Signals that tf was smaller than sampling
end
TW(PhaseCounter+1,3)=TW(PhaseCounter+1,2)+delta;                           % The upper bound of the next time window equals lower bound (s or tf) + delta time units
TW(PhaseCounter+1,16)=TW(PhaseCounter+1,3);                                % Store time window ending point (in time) again to use for the last iteration, i.e. when the leader has reached it's target

% Next iteration | A.2. Next problem initial boundary                                                                                               
if FlagTf                                                                  % If final time<sampling point then the next initial boundary for position-x, position-y,
   xi=phase.StateGrid.Values(1,end);yi=phase.StateGrid.Values(2,end);      % speed-x and speed-y are stored at the last column of StateGrid.Values since the 
   vxi=phase.StateGrid.Values(3,end);vyi=phase.StateGrid.Values(4,end);    % follower concludes it's movement
   t=phase.RealTime(1,1:end-1);                                            % Store time in TotalResults (Next iteration's initial point is the current's final so we don't save the last to avoid duplicates)
   sx=phase.StateGrid.Values(1,1:end-1);                                   % Store trajectory in TotalResults
   sy=phase.StateGrid.Values(2,1:end-1);
   svx=phase.StateGrid.Values(3,1:end-1);
   svy=phase.StateGrid.Values(4,1:end-1);
   cax= phase.ControlGrids.Values(1,1:end-1);
   cay= phase.ControlGrids.Values(2,1:end-1);
   TJ=phase.LagrangeCostFunctions.OutputGrid.Values(1,1:end-1);                           % Store Lagrange cost functions in TotalResults
   TotalResults{AgentCounterRow,AgentCounterColumn}=[t' sx' sy' svx' svy' cax' cay' TJ']; % Final TotalResults format
else                                                                           % If sampling<=final time then we need to check whether the sampling point is on the real
   [i,j]=find(phase.RealTime==TW(PhaseCounter+1,2));                           % time grid, i.e. no interpolation needed, or if we have to find the two closest points and interpolate
   if isempty(j)                                                               % If we need to interpolate, i.e. value wasn't found on the time grid
      PointAtemp=find(phase.RealTime<TW(PhaseCounter+1,2));                    % Find former point
      PointA=PointAtemp(end);                                                                                       
      PointB=PointA+1;                                                         % Find latter point
      TempRealTime=[phase.RealTime(1,PointA) phase.RealTime(1,PointB)];        % Find interpolation values
      xi=interp1(TempRealTime,[phase.StateGrid.Values(1,PointA) phase.StateGrid.Values(1,PointB)],TW(PhaseCounter+1,2));
      yi=interp1(TempRealTime,[phase.StateGrid.Values(2,PointA) phase.StateGrid.Values(2,PointB)],TW(PhaseCounter+1,2));
      vxi=interp1(TempRealTime,[phase.StateGrid.Values(3,PointA) phase.StateGrid.Values(3,PointB)],TW(PhaseCounter+1,2));
      vyi=interp1(TempRealTime,[phase.StateGrid.Values(4,PointA) phase.StateGrid.Values(4,PointB)],TW(PhaseCounter+1,2));
      t=phase.RealTime(1,1:PointA);                                        % Store time in TotalResults
      sx=phase.StateGrid.Values(1,1:PointA);                               % Store trajectory in TotalResults
      sy=phase.StateGrid.Values(2,1:PointA);
      svx=phase.StateGrid.Values(3,1:PointA);
      svy=phase.StateGrid.Values(4,1:PointA);
      cax=phase.ControlGrids.Values(1,1:PointA);
      cay=phase.ControlGrids.Values(2,1:PointA);
      TJ=phase.LagrangeCostFunctions.OutputGrid.Values(1,1:PointA); % Store Lagrange cost functions in TotalResults
      TotalResults{AgentCounterRow,AgentCounterColumn}=[t' sx' sy' svx' svy' cax' cay' TJ']; % Final TotalResults format
   else                                                                                      % If we don't need to interpolate
      xi=phase.StateGrid.Values(1,j);yi=phase.StateGrid.Values(2,j);
      vxi=phase.StateGrid.Values(3,j);vyi=phase.StateGrid.Values(4,j);  
      t=phase.RealTime(1,1:j-1);                                           % Store time in TotalResults
      sx=phase.StateGrid.Values(1,1:j-1);                                  % Store trajectory in TotalResults
      sy=phase.StateGrid.Values(2,1:j-1); 
      svx=phase.StateGrid.Values(3,1:j-1);
      svy=phase.StateGrid.Values(4,1:j-1);
      cax=phase.ControlGrids.Values(1,1:j-1);
      cay=phase.ControlGrids.Values(2,1:j-1);
      TJ=phase.LagrangeCostFunctions.OutputGrid.Values(1,1:j-1);                             % Store Lagrange cost functions in TotalResults
      TotalResults{AgentCounterRow,AgentCounterColumn}=[t' sx' sy' svx' svy' cax' cay' TJ']; % Final TotalResults format
   end
end
TW(PhaseCounter+1,4:7)=[xi yi vxi vyi];                                              % Store position-x,position-y,speed-x and speed-y to be used as initial boundaries for the next iteration
clear FlagTf xi yi vxi vyi i j PointAtemp PointA PointB TempRealTime t sx sy svx svy cax cay TJ;

% Next iteration | A.3. Next problem final boundary
PointTemp=find(Leader(:,1)<=TW(PhaseCounter+1,3));                                    % Find the leader's position at sampling (s or tf)
PointA=PointTemp(end);
[m,n]=size(Leader);
if (Leader(PointA,1)==TW(PhaseCounter+1,3)) || (m==PointA)                            % If tf or s matches the value in the discretization grid or if it's less than the time needed for x_0
   xl=Leader(PointA,2);                                                               % to reach goal then no interpolation is needed
   yl=Leader(PointA,3);
else                                                                                  % If tf or s doesn't match the value in the discretization grid and x_0 hasn't reached the goal then interpolation is needed
   PointB=PointA+1;                                                                   % Latter point
   TempRealTime=[Leader(PointA,1) Leader(PointB,1)];                                  % Time for interpolation values
   xl=interp1(TempRealTime,[Leader(PointA,2) Leader(PointB,2)],TW(PhaseCounter+1,3)); % Interpolation for position-x
   yl=interp1(TempRealTime,[Leader(PointA,3) Leader(PointB,3)],TW(PhaseCounter+1,3)); % Interpolation for position-y 
end                                                 
TW(PhaseCounter+1,8:11)=[xl yl xl yl];                                                % Save leader's position  to be used as final boundary for the next iteration
clear PointTemp PointA m n xl yl PointB TempRealTime;

% Next iteration | A.4. Ending Conditions : Check if the follower has "catched" the leader or leader has reached C_P
ProximityCheck=sqrt( (TW(PhaseCounter+1,4)-TW(PhaseCounter+1,8))^2 + (TW(PhaseCounter+1,5)-TW(PhaseCounter+1,9))^2 );   % Leader follower distance
xl=TW(PhaseCounter+1,8);yl=TW(PhaseCounter+1,9);                                                                        % Leader final position has already been evaluated and stored in the time window variable (TW)
if (inpolygon(xl,yl,xv,yv)) || (ProximityCheck<=ProximityDistance)                                                      % Check if follower has reached C_P or caught up with leader
%    TW(PhaseCounter+1,8:11)=[Leader(end,2) Leader(end,3) Leader(end,2) Leader(end,3)];                                 % C_P is the final boundary for the next iteration
   TW(PhaseCounter+1,8:11)=[xv(1) yv(1) xv(2) yv(2)];                                                                   % C_P is the final boundary for the next iteration
   TW(PhaseCounter+1,16)=Leader(end,1);                                                                                 % In this case the final time for the leader is not  the end of the time window but when it reached goal
   EndFlag=true;                                                                                                        % Signal the next iteration to be the last
end
clear ProximityCheck xl yl;

% Next iteration | A.5. Next time window
LeaderDet=[ ];
% A.5.1. Leader's next initial position and detection area
PointAtemp=find(TW(PhaseCounter+1,2)<=Leader(:,1));                        % Find obstacle positions for the next time window
PointA=PointAtemp(1);
[m,n]=size(Leader);
if (Leader(PointA,1)==delta) || (m==PointA)                                % If Ä matches the value in the discretization grid or if delta is less than the time needed for x_0
   xl=Leader(PointA,2);                                                    % to reach goal then no interpolation is needed
   yl=Leader(PointA,3);
else                                                                       % If Ä doesn't match the value in the discretization grid and x_0 hasn't reached the goal then interpolation is needed
   PointB=PointA-1;                                                                                                % Latter interpolation point
   xl=interp1([Leader(PointA,1) Leader(PointB,1)],[Leader(PointA,2) Leader(PointB,2)],TW(PhaseCounter+1,2));       % Interpolation for position-x
   yl=interp1([Leader(PointA,1) Leader(PointB,1)],[Leader(PointA,3) Leader(PointB,3)],TW(PhaseCounter+1,2));       % Interpolation for position-y
end
LowerX=xl-LeaDecRad;UpperX=xl+LeaDecRad;                                   % Detection area for interpolated position-x
LowerY=yl-LeaDecRad;UpperY=yl+LeaDecRad;                                   % Detection area for interpolated position-x
LeaderDet(1,:)=[TW(PhaseCounter+1,2) xl yl LowerX UpperX LowerY UpperY];
clear PointAtemp PointA m n xl yl PointB LowerX UpperX LowerY UpperY;

% A.5.2. Leader positions and detection areas in time window
temp1=find(TW(PhaseCounter+1,2)<=Leader(:,1));   % Find obstacle positions for the next time window
temp2=find(Leader(:,1)<=TW(PhaseCounter+1,3));
PointA=temp1(1);PointB=temp2(end);
LeaderDet=[LeaderDet; Leader(PointA:PointB,:)];
clear temp1 temp2 PointA PointB;

% A.5.3. Leader final position and detection area
xl=TW(PhaseCounter+1,8);yl=TW(PhaseCounter+1,9);
LowerX=xl-LeaDecRad;UpperX=xl+LeaDecRad;                                   % Detection area for interpolated position-x
LowerY=yl-LeaDecRad;UpperY=yl+LeaDecRad;                                   % Detection area for interpolated position-x
LeaderDet(end+1,:)=[TW(PhaseCounter+1,3) xl yl LowerX UpperX LowerY UpperY];
clear xl yl LowerX UpperX LowerY UpperY;

% A.5.4. Follower current position and detection area
xl=TW(PhaseCounter+1,4);yl=TW(PhaseCounter+1,5);                           % We already have evaluated the follower's current position
LowerX=xl-LeaDecRad;UpperX=xl+LeaDecRad;                                   % Detection area for interpolated position-x
LowerY=yl-LeaDecRad;UpperY=yl+LeaDecRad;                                   % Detection area for interpolated position-x

% A.5.5. Time window
XminNTW=min(min(LeaderDet(:,4)),LowerX);
XmaxNTW=max(max(LeaderDet(:,5)),UpperX);
YminNTW=min(min(LeaderDet(:,6)),LowerY);
YmaxNTW=max(max(LeaderDet(:,7)),UpperY);
TW(PhaseCounter+1,12:15)=[XminNTW XmaxNTW YminNTW YmaxNTW]; 
clear xl yl LowerX UpperX LowerY UpperY XminNTW XmaxNTW YminNTW YmaxNTW;

% Next iteration | B. Final time
TimeLowerLimit=TW(PhaseCounter+1,2) ;                                      % The new lower limit is the sampling point (tf or s)
if OptimizeInDelta                                                         % If the follower is set to optimize [s,s+Ä]
   TimeLimitStep=delta;
else
   TimeLimitStep=MaxTime;                                                  % If the follower is set to optimize in MaxTime frame
end
TimeUpperLimit=TimeLowerLimit+TimeLimitStep;
TimeInitialGuess=(TimeLowerLimit+TimeUpperLimit)/2;    

% Next iteration | C. Discretization
% Doesn't need any changes; all tau are the same for all problems solved

% Next iteration | D. Problem: Create a new Instance 
% Doesn't need any changes; all variables are cleared at the end, so we use the same problem name

% Next iteration | E. Problem: Add a new phase
PhaseInitialTime=TW(PhaseCounter+1,2);                         % The new problem will also have one phase which starts at the sampling point (tf or s)

% Next iteration | F. Phase: Add control grid
% Control limits are the same for all iterations

% Next iteration | G. Phase: Add model output
% Outputs are the same for all iterations

% Next iteration | I. Phase: Add boundary conditions
% I.A. The initial boundaries were evaluated and saved in the time window variable (TW)
% I.B. The final boundaries were evaluated and saved in the time window variable (TW)

% Next iteration | O.1. Obstacle positions for the next time window
ObstacleDet=[ ];
[m,n]=size(LeaderDet);
for i=1:m
     ObstacleDet(i,:)=[LeaderDet(i,1) ObsIniX+(LeaderDet(i,1)/Divider)*SpeedX ObsIniY+(LeaderDet(i,1)/Divider)*SpeedY ObsIniR+(LeaderDet(i,1)/Divider)*SpeedR];
     ObstacleDet2(i,:)=[LeaderDet(i,1) CeX CeY CeR+LeaderDet(i,1)/100];
end
ObstacleDet(:,5)=ObstacleDet(:,2)-(ObstacleDet(:,4)+ObsSafRad);
ObstacleDet(:,6)=ObstacleDet(:,2)+(ObstacleDet(:,4)+ObsSafRad);
ObstacleDet(:,7)=ObstacleDet(:,3)-(ObstacleDet(:,4)+ObsSafRad);
ObstacleDet(:,8)=ObstacleDet(:,3)+(ObstacleDet(:,4)+ObsSafRad);

ObstacleDet2(:,5)=ObstacleDet2(:,2)-ObstacleDet2(:,4);
ObstacleDet2(:,6)=ObstacleDet2(:,2)+ObstacleDet2(:,4);
ObstacleDet2(:,7)=ObstacleDet2(:,3)-ObstacleDet2(:,4);
ObstacleDet2(:,8)=ObstacleDet2(:,3)+ObstacleDet2(:,4);
clear m n i;

% Next iteration | O.2. Intersections for the detections around each leader's position with the obstacle
[m,n]=size(LeaderDet);
for i=1:m
     xmin=ObstacleDet(i,5);xmax=ObstacleDet(i,6);ymin=ObstacleDet(i,7);ymax=ObstacleDet(i,8);  % Bounding box from the four corners of each obstacle position 
     LeaObsIntPoly(i,1)=polyshape([xmin xmin xmax xmax],[ymin ymax ymax ymin]);                % Polyshape corrsponding to the obstacle's bounding box
     xmin=LeaderDet(i,4);xmax=LeaderDet(i,5);ymin=LeaderDet(i,6);ymax=LeaderDet(i,7);          % Bounding box from the four corners of each leader's position 
     LeaObsIntPoly(i,2)=polyshape([xmin xmin xmax xmax],[ymin ymax ymax ymin]);                % Polyshape corrsponding to the leader's bounding box     
     LeaObsIntPoly(i,3)=intersect(LeaObsIntPoly(i,1),LeaObsIntPoly(i,2));                      % Bounding boxes for intersections (0 vertices if empty)
end
clear m n i xmin xmax ymin ymax;

[m,n]=size(LeaderDet);
for i=1:m
     xmin2=ObstacleDet2(i,5);xmax2=ObstacleDet2(i,6);ymin2=ObstacleDet2(i,7);ymax2=ObstacleDet2(i,8);  % Bounding box from the four corners of each puddle's position 
     LeaObsIntPoly2(i,1)=polyshape([xmin2 xmin2 xmax2 xmax2],[ymin2 ymax2 ymax2 ymin2]);               % Polyshape corrsponding to the puddle's bounding box
     xmin=LeaderDet(i,4);xmax=LeaderDet(i,5);ymin=LeaderDet(i,6);ymax=LeaderDet(i,7);                  % Bounding box from the four corners of each leader's position 
     LeaObsIntPoly2(i,2)=polyshape([xmin xmin xmax xmax],[ymin ymax ymax ymin]);                       % Polyshape corrsponding to the leader's bounding box     
     LeaObsIntPoly2(i,3)=intersect(LeaObsIntPoly2(i,1),LeaObsIntPoly2(i,2));                           % Bounding boxes for intersections (0 vertices if empty)
end
clear m n i xmin2 xmax2 ymin2 ymax2;

% Next iteration | O.3. Get non zero intersections and create minimum bounding ellipse for each intersection
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
        LeaObsDetTW(j,1:7)=LeaderDet(i,:);                                 % Save Leader t,x,y and bounding box for each leader's position     
        LeaObsDetTW(j,8:15)=ObstacleDet(i,:);                              % Save obstacle t,x,y,r and bounding box for each obstacle's position
        LeaObsDetTW(j,16:19)=[XLimBou(1) XLimBou(2) YLimBou(1) YLimBou(2)];    % Save bounding box of each polygon intersection
        LeaObsDetTW(j,20:23)=[C(1) C(2) a b];                                  % Save center-x,center-y,a and b for the minimum bounding ellipse of each intersection
        IntersectionFlag=true;
        j=j+1;
    end
end

[m,n]=size(LeaObsIntPoly2);                                                                     
LeaObsDetTW2=[ ];
IntersectionFlag=false;
j=1;   
for i=1:m
    if ~isempty(LeaObsIntPoly2(i,3).Vertices)                                       % Check if there was an intersection
        [XLimBou2,YLimBou2] = boundingbox(LeaObsIntPoly2(i,3));                     % Create bounding box for the intersection coordinates
        xmin2=XLimBou2(1);xmax2=XLimBou2(2);ymin2=YLimBou2(1);ymax2=YLimBou2(2);    % Prepare intersection coordinates for minimum bounding ellipse function
        P2=[[xmin2;ymin2] [xmin2;ymax2] [xmax2;ymax2] [xmax2;ymin2]];
        [A2,C2]=MinVolEllipse(P2,0.00001);                                          % Get a,b and coordinates for the center of the minimum bounding ellipse
        [a2,b2,C2,X2]=Ellipse_plot(A2,C2);
        LeaObsDetTW2(j,1:7)=LeaderDet(i,:);                                         % Save Leader t,x,y and bounding box for each leader's position     
        LeaObsDetTW2(j,8:15)=ObstacleDet2(i,:);                                     % Save obstacle t,x,y,r and bounding box for each obstacle's position
        LeaObsDetTW2(j,16:19)=[XLimBou2(1) XLimBou2(2) YLimBou2(1) YLimBou2(2)];    % Save bounding box of each polygon intersection
        LeaObsDetTW2(j,20:23)=[C2(1) C2(2) a2 b2];                                  % Save center-x,center-y,a and b for the minimum bounding ellipse of each intersection
        IntersectionFlag=true;
        j=j+1;
    end
end
clear m n IntersectionFlag j i XLimBou YLimBou xmin xmax ymin ymax P A C a b X LeaderDet ObstacleDet LeaObsIntPoly;
clear m n IntersectionFlag j i XLimBou2 YLimBou2 xmin2 xmax2 ymin2 ymax2 P2 A2 C2 a2 b2 X2 LeaderDet2 ObstacleDet2 LeaObsIntPoly2;

% Next iteration | O.4. Get minimum bounding ellipse for each pair of intersections
[m,n]=size(LeaObsDetTW);
LeaObsDetCon = [ ];
if m==1  % If there is only one intersection
   xmin=LeaObsDetTW(1,16);xmax=LeaObsDetTW(1,17);ymin=LeaObsDetTW(1,18);ymax=LeaObsDetTW(1,19);  % Get intersection   
   Poly=polyshape([xmin xmin xmax xmax],[ymin ymax ymax ymin]);            % Polyshape for the intersection
   [XLimBou,YLimBou] = boundingbox(Poly);                                  % Bounding box for the intersection
   xmin=XLimBou(1);xmax=XLimBou(2);ymin=YLimBou(1);ymax=YLimBou(2);        % Prepare intersection coordinates for minimum bounding ellipse function
   P=[[xmin;ymin] [xmin;ymax] [xmax;ymax] [xmax;ymin]];
   [A,C]=MinVolEllipse(P,0.00001);                                         % Get a,b and coordinates for the center of the minimum bounding ellipse
   [a,b,C,X]=Ellipse_plot(A,C);
   LeaObsDetCon(1,:)=[LeaObsDetTW(1,1) LeaObsDetTW(1,1) (LeaObsDetTW(1,1)+LeaObsDetTW(1,1))/2 C(1) C(2) a b]; % Save center-x,center-y,a and b for the minimum bounding ellipse of each intersection pair         
else        % If there are more than one intersections
  for i=1:m-1
       xmin=LeaObsDetTW(i,16);xmax=LeaObsDetTW(i,17);ymin=LeaObsDetTW(i,18);ymax=LeaObsDetTW(i,19);           % Get first intersection
       PolyLowerTime=polyshape([xmin xmin xmax xmax],[ymin ymax ymax ymin]);                                  % Polyshape for first intersection
       xmin=LeaObsDetTW(i+1,16);xmax=LeaObsDetTW(i+1,17);ymin=LeaObsDetTW(i+1,18);ymax=LeaObsDetTW(i+1,19);   % Get second intersection
       PolyUpperTime=polyshape([xmin xmin xmax xmax],[ymin ymax ymax ymin]);                                  % Polyshape for second intersection
       PolyUnion(i,1)=union(PolyLowerTime,PolyUpperTime);                                                     % Union of the two intersections
       [XLimBou,YLimBou] = boundingbox(PolyUnion(i,1));                                                       % Bounding box for the union
       xmin=XLimBou(1);xmax=XLimBou(2);ymin=YLimBou(1);ymax=YLimBou(2);                                       % Prepare intersection coordinates for minimum bounding ellipse function
       P=[[xmin;ymin] [xmin;ymax] [xmax;ymax] [xmax;ymin]];
       [A,C]=MinVolEllipse(P,0.00001);                                                                        % Get a,b and coordinates for the center of the minimum bounding ellipse
       [a,b,C,X]=Ellipse_plot(A,C);
       LeaObsDetCon(i,:)=[LeaObsDetTW(i,1) LeaObsDetTW(i+1,1) (LeaObsDetTW(i,1)+LeaObsDetTW(i+1,1))/2 C(1) C(2) a b]; % Save center-x,center-y,a and b for the minimum bounding ellipse of each intersection pair         
  end
end
clear m n i xmin xmax ymin ymax PolyLowerTime PolyUpperTime PolyUnion XLimBou YLimBou P A C a b C X;

[m,n]=size(LeaObsDetTW2);
LeaObsDetCon2 = [ ];
if m==1  % If there is only one intersection
   xmin2=LeaObsDetTW2(1,16);xmax2=LeaObsDetTW2(1,17);ymin2=LeaObsDetTW2(1,18);ymax2=LeaObsDetTW2(1,19);  % Get intersection   
   Poly2=polyshape([xmin2 xmin2 xmax2 xmax2],[ymin2 ymax2 ymax2 ymin2]);            % Polyshape for the intersection
   [XLimBou2,YLimBou2] = boundingbox(Poly2);                                       % Bounding box for the intersection
   xmin2=XLimBou2(1);xmax2=XLimBou2(2);ymin2=YLimBou2(1);ymax2=YLimBou2(2);        % Prepare intersection coordinates for minimum bounding ellipse function
   P2=[[xmin2;ymin2] [xmin2;ymax2] [xmax2;ymax2] [xmax2;ymin2]];
   [A2,C2]=MinVolEllipse(P2,0.00001);                                              % Get a,b and coordinates for the center of the minimum bounding ellipse
   [a2,b2,C2,X2]=Ellipse_plot(A2,C2);
   LeaObsDetCon2(1,:)=[LeaObsDetTW2(1,1) LeaObsDetTW2(1,1) (LeaObsDetTW2(1,1)+LeaObsDetTW2(1,1))/2 C2(1) C2(2) a2 b2]; % Save center-x,center-y,a and b for the minimum bounding ellipse of each intersection pair         
else        % If there are more than one intersections
  for i=1:m-1
       xmin2=LeaObsDetTW2(i,16);xmax2=LeaObsDetTW2(i,17);ymin2=LeaObsDetTW2(i,18);ymax2=LeaObsDetTW2(i,19);           % Get first intersection
       PolyLowerTime2=polyshape([xmin2 xmin2 xmax2 xmax2],[ymin2 ymax2 ymax2 ymin2]);                                  % Polyshape for first intersection
       xmin2=LeaObsDetTW2(i+1,16);xmax2=LeaObsDetTW2(i+1,17);ymin2=LeaObsDetTW2(i+1,18);ymax2=LeaObsDetTW2(i+1,19);   % Get second intersection
       PolyUpperTime2=polyshape([xmin2 xmin2 xmax2 xmax2],[ymin2 ymax2 ymax2 ymin2]);                                  % Polyshape for second intersection
       PolyUnion2(i,1)=union(PolyLowerTime2,PolyUpperTime2);                                                     % Union of the two intersections
       [XLimBou2,YLimBou2] = boundingbox(PolyUnion2(i,1));                                                       % Bounding box for the union
       xmin2=XLimBou2(1);xmax2=XLimBou2(2);ymin2=YLimBou2(1);ymax2=YLimBou2(2);                                       % Prepare intersection coordinates for minimum bounding ellipse function
       P2=[[xmin2;ymin2] [xmin2;ymax2] [xmax2;ymax2] [xmax2;ymin2]];
       [A2,C2]=MinVolEllipse(P2,0.00001);                                                                        % Get a,b and coordinates for the center of the minimum bounding ellipse
       [a2,b2,C2,X2]=Ellipse_plot(A2,C2);
       LeaObsDetCon2(i,:)=[LeaObsDetTW2(i,1) LeaObsDetTW2(i+1,1) (LeaObsDetTW2(i,1)+LeaObsDetTW2(i+1,1))/2 C2(1) C2(2) a2 b2]; % Save center-x,center-y,a and b for the minimum bounding ellipse of each intersection pair         
  end
end
clear m n i xmin2 xmax2 ymin2 ymax2 PolyLowerTime2 PolyUpperTime2 PolyUnion2 XLimBou2 YLimBou2 P2 A2 C2 a2 b2 C2 X2;

% Plots
% Plot leader at delta position and coordinates
delete(h1);                                                                                                               % Delete previous leader at delta position
scatter3(TW(PhaseCounter+1,8),TW(PhaseCounter+1,9),TW(PhaseCounter+1,16),'MarkerFaceColor','r');                          % Plot leader at delta position
CoordinateX=num2str(TW(PhaseCounter+1,8));CoordinateY=num2str(TW(PhaseCounter+1,9));CoordinateT=num2str(TW(PhaseCounter+1,16));
LeaderAtDeltaCoordinates=strcat(' L(',CoordinateX,',',CoordinateY,',',CoordinateT,')');
h1=text(TW(PhaseCounter+1,8),TW(PhaseCounter+1,9),TW(PhaseCounter+1,16),LeaderAtDeltaCoordinates,'Color','r');  % Plot leader at delta text coordinates
clear CoordinateX CoordinateY CoordinateT LeaderAtDeltaCoordinates;

% Plot follower at delta position and coordinates
delete(h2);                                                                                                             % Delete previous follower position            
scatter3(TW(PhaseCounter+1,4),TW(PhaseCounter+1,5),TW(PhaseCounter+1,2),'MarkerFaceColor','b');                         % Plot follower position
CoordinateX=num2str(TW(PhaseCounter+1,4));CoordinateY=num2str(TW(PhaseCounter+1,5));CoordinateT=num2str(TW(PhaseCounter+1,2));
LeaderAtDeltaCoordinates=strcat(' F(',CoordinateX,',',CoordinateY,',',CoordinateT,')');
h2=text(TW(PhaseCounter+1,4),TW(PhaseCounter+1,5),TW(PhaseCounter+1,2),LeaderAtDeltaCoordinates,'Color','b'); % Plot follower position coordinates
clear CoordinateX CoordinateY CoordinateT LeaderAtDeltaCoordinates;

% Plot follower trajectory
plot3(phase.StateGrid.Values(1,:),phase.StateGrid.Values(2,:),phase.RealTime,'b');                                          % Plot follower's trajectory for all points regardless s<>=tf - 3D
scatter3(phase.StateGrid.Values(1,end),phase.StateGrid.Values(2,end),phase.RealTime(1,end),'square','MarkerFaceColor','b'); % If s<tf then the next initial point is different than the arrival point of the evaluated trajectory      

% Plots B. Obstacle position for follower
% Clean up previous plots
if PhaseCounter>1                                             
   [m,n]=size(PlotsObstacleCenter);
   for i=1:m
        delete(PlotsObstacleCenter{i,1}); 
        delete(PlotsObstacleWithoutSafRad{i,1}); 
        delete(PlotsObstacleWithSafRad{i,1}); 
   end
end
clear m n i;

% Plot obstacle positions and radius                          
j=1;
for time=TW(PhaseCounter,2):TW(PhaseCounter+1,2)
    x0=ObsIniX+(time/Divider)*SpeedX;
    y0=ObsIniY+(time/Divider)*SpeedY;
    R=ObsIniR+(time/Divider)*SpeedR;
    PlotsObstacleCenter{j,1}=scatter3(x0,y0,time);
    xmin=x0-R;xmax=x0+R;
    ymin=y0-R;ymax=y0+R;
    X=[xmin xmin xmax xmax];Y=[ymin ymax ymax ymin];           
    P = [X(1) Y(1) time;X(2) Y(2) time;X(3) Y(3) time;X(4) Y(4) time]; 
    PlotsObstacleWithoutSafRad{j,1}=patch(P(:,1),P(:,2),P(:,3),'w');
    xmin=x0-(R+ObsSafRad);xmax=x0+(R+ObsSafRad);
    ymin=y0-(R+ObsSafRad);ymax=y0+(R+ObsSafRad);
    X=[xmin xmin xmax xmax xmin];Y=[ymin ymax ymax ymin ymin];
    P = [X(1) Y(1) time;X(2) Y(2) time;X(3) Y(3) time;X(4) Y(4) time]; 
    PlotsObstacleWithSafRad{j,1}=patch(P(:,1),P(:,2),P(:,3),'w');
    j=j+1;
end
clear m n theta j time x0 y0 R X Y X1 Y1 k l T;

% Plots C. Bounding boxes intersections for the next time window
[m,n]=size(PlotsIntersections);
for i=1:m
     delete(PlotsIntersections{i,1}); 
end
clear m n;

[m,n]=size(LeaObsDetTW);
for i=1:m
     xmin=LeaObsDetTW(i,16);xmax=LeaObsDetTW(i,17);ymin=LeaObsDetTW(i,18);ymax=LeaObsDetTW(i,19);      % Get intersection 
     X=[xmin xmin xmax xmax xmin];Y=[ymin ymax ymax ymin ymin];                                                                      
     T=[LeaObsDetTW(i,1) LeaObsDetTW(i,1) LeaObsDetTW(i,1) LeaObsDetTW(i,1) LeaObsDetTW(i,1)];         % Plot intersection
     PlotsIntersections{i,1}=plot3(X,Y,T,'Color','r','LineStyle','-');        
end
clear m n i xmin xmax ymin ymax X Y T;

% Plots D. Minimum bounding ellipse per pair of intersections for the next time window
[m,n]=size(PlotsPerPairIntersections);
for i=1:m
     delete(PlotsPerPairIntersections{i,1}); 
end
clear m n;

[m,n]=size(LeaObsDetTW);
for i=1:m-1
     xmin=LeaObsDetTW(i,16);xmax=LeaObsDetTW(i,17);ymin=LeaObsDetTW(i,18);ymax=LeaObsDetTW(i,19);          % Get first intersection                               
     PolyLowerTime=polyshape([xmin xmin xmax xmax],[ymin ymax ymax ymin]);                                 % First intersection polyshape
     xmin=LeaObsDetTW(i+1,16);xmax=LeaObsDetTW(i+1,17);ymin=LeaObsDetTW(i+1,18);ymax=LeaObsDetTW(i+1,19);  % Get second intersection 
     PolyUpperTime=polyshape([xmin xmin xmax xmax],[ymin ymax ymax ymin]);                                 % Second intesection polyshape
     PolyUnion(i,1)=union(PolyLowerTime,PolyUpperTime);                                                    % Union of intersections
     [XLimBou,YLimBou] = boundingbox(PolyUnion(i,1));                                                      % Bounding box of the union
     xmin=XLimBou(1);xmax=XLimBou(2);ymin=YLimBou(1);ymax=YLimBou(2);                                      % Prepare bounding box (union) for minimum bounding ellipse function
     P=[[xmin;ymin] [xmin;ymax] [xmax;ymax] [xmax;ymin]];
     [A,C]=MinVolEllipse(P,0.00001);                                                                       % Get a,b and coordinates for the center of the minimum bounding ellipse
     [a,b,C,X]=Ellipse_plot(A,C);
     [k l]=size(X);
     temp=repmat((LeaObsDetTW(i,1)+LeaObsDetTW(i+1,1))/2,1,l);   
     PlotsPerPairIntersections{i,1}=plot3(X(1,:),X(2,:),temp,'y','LineWidth',1.5);                         % Plot midpoint intersections
end
title('Leader(X5) - Follower(X6)');
clear m n i xmin xmax ymin ymax X Y T PolyLowerTime PolyUpperTime PolyUnion XLimBou YLimBou P A X a b C X k l temp;

PhaseCounter=PhaseCounter+1;                                               % Increase iteration counter
AgentCounterColumn=AgentCounterColumn+1;                                   % Store the results of the next iteration in the next column of the same line in TotalResults

% Record figure to video
AgentPlot=getframe(gcf);
writeVideo(v,AgentPlot);

% Clean up problem and constraint folder
clear problem;
rmdir C:\FALCON.m.v1.24.2002191427\falcon\SetupA_Final\fm_constraints s
end

% Save video and figure
close(v);
savefig(FigureX6A,'FigureX6A.fig');

% Save solver stats
save SolverStats.mat SolverStats;
% ------------------------------------------------------------------------------------------------------------------------------------------------------------------
% End of x_6 Main
% ------------------------------------------------------------------------------------------------------------------------------------------------------------------

% ------------------------------------------------------------------------------------------------------------------------------------------------------------------
% Prepare data for next agent
% ------------------------------------------------------------------------------------------------------------------------------------------------------------------
% P.1. New leader coordinates
Leader=[ ] ;
[m,n]=size(TotalResults);                                                  % The trajectory for each problem was saved in the AgentCounterColumn of the TotalResults 
for i=1:n                                                                  % cell (in the corresponding to the agent AgentCounterRow)
     if ~isempty(TotalResults{AgentCounterRow,i})
        Leader=[Leader; TotalResults{AgentCounterRow,i}(:,1:3)];
     end
end
clear m n i;

Leader=unique(Leader,'rows');                                              % Remove any duplicate entries
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

save LeaderX6.mat Leader;                                                  % Save leader's trajectory - Save B

% P.2. New obstacle coordinates
Obstacle=[ ];
Obstacle2=[ ];
[m,n]=size(Leader);
for i=1:m
     Obstacle(i,:)=[Leader(i,1) ObsIniX+(Leader(i,1)/Divider)*SpeedX ObsIniY+(Leader(i,1)/Divider)*SpeedY ObsIniR+(Leader(i,1)/Divider)*SpeedR];
     Obstacle2(i,:)=[Leader(i,1) CeX CeY CeR+Leader(i,1)/100];
end
Obstacle(:,5)=Obstacle(:,2)-(Obstacle(:,4)+ObsSafRad);
Obstacle(:,6)=Obstacle(:,2)+(Obstacle(:,4)+ObsSafRad);  
Obstacle(:,7)=Obstacle(:,3)-(Obstacle(:,4)+ObsSafRad);
Obstacle(:,8)=Obstacle(:,3)+(Obstacle(:,4)+ObsSafRad);
save ObstacleX6.mat Obstacle;                    % Save obstacle positions and radius - Save C

Obstacle2(:,5)=Obstacle2(:,2)-Obstacle2(:,4);
Obstacle2(:,6)=Obstacle2(:,2)+Obstacle2(:,4);  
Obstacle2(:,7)=Obstacle2(:,3)-Obstacle2(:,4);
Obstacle2(:,8)=Obstacle2(:,3)+Obstacle2(:,4);
save ObstacleX6P.mat Obstacle2;                    % Save obstacle positions and radius - Save C
clear m n i;

% P.3. New initial time window construction
% P.3.1. Leader positions and detections for the first Ä time units
PointTemp=find(Leader(:,1)<=delta);                                        % Leader's position after Ä time units
PointA=PointTemp(end);                                  
[m,n]=size(Leader);
if (Leader(PointA,1)==delta) || (m==PointA)                                % If Ä matches the value in the discretization grid then we don't need to interpolate
   xf=Leader(PointA,2);yf=Leader(PointA,3);                                % Leader's position at Ä to be used as final boundary for the next agent
   LeaderDet=Leader(1:PointA,:);                                           % Keep leader's positions for first Ä time units
   Lx=min(Leader(1:PointA,4));Ux=max(Leader(1:PointA,5));                  % xmin xmax for all x-axis detections in the initial window [0,Ä]
   Ly=min(Leader(1:PointA,6));Uy=max(Leader(1:PointA,7));                  % ymin ymax for all x-axis detections in the initial window [0,Ä]
else                                                                       % If interpolation is needed
   PointB=PointA+1;                                                        % Latter interpolation point
   xf=interp1([Leader(PointA,1) Leader(PointB,1)],[Leader(PointA,2) Leader(PointB,2)],delta);       % Interpolation for position-x
   yf=interp1([Leader(PointA,1) Leader(PointB,1)],[Leader(PointA,3) Leader(PointB,3)],delta);       % Interpolation for position-y
   LowerX=xf-LeaDecRad;UpperX=xf+LeaDecRad;  % Detection area for interpolated position-x
   LowerY=yf-LeaDecRad;UpperY=yf+LeaDecRad;  % Detection area for interpolated position-x
   LeaderDet=[Leader(1:PointA,:);delta xf yf LowerX UpperX LowerY UpperY]; % Keep interpolated leader's position for [0,Ä] including the interpolation point
   Lx=min(min(Leader(1:PointA,4)),LowerX);                                 % xmin for all detections in the initial window [0,Ä] including the interpolation point
   Ux=max(max(Leader(1:PointA,5),UpperX));                                 % xmax for all detections in the initial window [0,Ä] including the interpolation point
   Ly=min(min(Leader(1:PointA,6)),LowerY);                                 % ymin for all detections in the initial window [0,Ä] including the interpolation point
   Uy=max(max(Leader(1:PointA,7)),UpperY);                                 % ymin for all detections in the initial window [0,Ä] including the interpolation point
end

% P.3.2. Time window variable
xi=TW(1,4);yi=TW(1,5);                                                     % All agents depart from the same point with same speed (zero)
vxi=TW(1,6);vyi=TW(1,7);                                   
TW=[ ];
TW(1,:)=[1 0 delta xi yi vxi vyi xf yf xf yf Lx Ux Ly Uy delta 0]; % Initial time window
save TimeWindowX6.mat TW;                                % Save the new initial time window information - Save D
clear PointTemp PointA m n xf yf Lx Ux Ly Uy PointB LowerX UpperX LowerY UpperY;

% 3.3. Obstacle position and detections for the first Ä time units
PointTemp=find(Obstacle(:,1)<=delta);                  % Find obstacle coordinates up to Ä
PointA=PointTemp(end);      
[m,n]=size(Obstacle);
if (Obstacle(PointA,1)==delta) || (m==PointA)          % If Ä matches the value in the discretization grid or if delta is less than the time needed for x_0 to reach goal then no interpolation is needed
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

% P.3.4. Intersections for the detections around each leader's position with the obstacle
[m,n]=size(LeaderDet);
for i=1:m
     xmin=ObstacleDet(i,5);xmax=ObstacleDet(i,6);ymin=ObstacleDet(i,7);ymax=ObstacleDet(i,8);  % Bounding box from the four corners of each obstacle position 
     LeaObsIntPoly(i,1)=polyshape([xmin xmin xmax xmax],[ymin ymax ymax ymin]);                % Polyshape corrsponding to the obstacle's bounding box
     xmin=LeaderDet(i,4);xmax=LeaderDet(i,5);ymin=LeaderDet(i,6);ymax=LeaderDet(i,7);          % Bounding box from the four corners of each leader's position 
     LeaObsIntPoly(i,2)=polyshape([xmin xmin xmax xmax],[ymin ymax ymax ymin]);                % Polyshape corrsponding to the leader's bounding box
     LeaObsIntPoly(i,3)=intersect(LeaObsIntPoly(i,1),LeaObsIntPoly(i,2));                      % Bounding boxes for intersections (0 vertices if empty)
end
save IntersectionPolyshapeX6.mat LeaObsIntPoly; % Save E
clear m n i xmin xmax ymin ymax;

[m,n]=size(ObstacleDet);
for i=1:m
     xmin2=ObstacleDet2(i,5);xmax2=ObstacleDet2(i,6);ymin2=ObstacleDet2(i,7);ymax2=ObstacleDet2(i,8);  % Bounding box from the four corners of each puddles's position 
     LeaObsIntPoly2(i,1)=polyshape([xmin2 xmin2 xmax2 xmax2],[ymin2 ymax2 ymax2 ymin2]);               % Polyshape corrsponding to the puddle's bounding box
     xmin2=Leader(i,4);xmax2=Leader(i,5);ymin2=Leader(i,6);ymax2=Leader(i,7);                          % Bounding box from the four corners of each leader's position 
     LeaObsIntPoly2(i,2)=polyshape([xmin2 xmin2 xmax2 xmax2],[ymin2 ymax2 ymax2 ymin2]);               % Polyshape corrsponding to the leader's bounding box
     LeaObsIntPoly2(i,3)=intersect(LeaObsIntPoly2(i,1),LeaObsIntPoly2(i,2));                           % Bounding boxes for intersections (0 vertices if empty)
end
save IntersectionPolyshapeX6P.mat LeaObsIntPoly2; % Save E
clear m n i xmin xmax ymin ymax;

% P.3.5. Get non zero intersections and create minimum bounding ellipse for each intersection
[m,n]=size(LeaObsIntPoly);                                                                     
LeaObsDetTW=[ ];
IntersectionFlag=false;
j=1;   
for i=1:m
    if ~isempty(LeaObsIntPoly(i,3).Vertices)                                 % Check if there was an intersection
        [XLimBou,YLimBou] = boundingbox(LeaObsIntPoly(i,3));                 % Create bounding box for the intersection coordinates
        xmin=XLimBou(1);xmax=XLimBou(2);ymin=YLimBou(1);ymax=YLimBou(2);     % Prepare intersection coordinates for minimum bounding ellipse function
        P=[[xmin;ymin] [xmin;ymax] [xmax;ymax] [xmax;ymin]];
        [A,C]=MinVolEllipse(P,0.00001);                                      % Get a,b and coordinates for the center of the minimum bounding ellipse
        [a,b,C,X]=Ellipse_plot(A,C);
        LeaObsDetTW(j,1:7)=LeaderDet(i,:);                                   % Save Leader t,x,y and bounding box for each leader's position     
        LeaObsDetTW(j,8:15)=ObstacleDet(i,:);                                % Save obstacle t,x,y,r and bounding box for each obstacle's position
        LeaObsDetTW(j,16:19)=[XLimBou(1) XLimBou(2) YLimBou(1) YLimBou(2)];  % Save bounding box of each polygon intersection
        LeaObsDetTW(j,20:23)=[C(1) C(2) a b];                                % Save center-x,center-y,a and b for the minimum bounding ellipse of each intersection
        IntersectionFlag=true;
        j=j+1;
    end
end

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
        LeaObsDetTW2(j,1:7)=LeaderDet(i,:);                                      % Save Leader t,x,y and bounding box for each leader's position     
        LeaObsDetTW2(j,8:15)=ObstacleDet2(i,:);                                  % Save obstacle t,x,y,r and bounding box for each obstacle's position
        LeaObsDetTW2(j,16:19)=[XLimBou2(1) XLimBou2(2) YLimBou2(1) YLimBou2(2)]; % Save bounding box of each polygon intersection
        LeaObsDetTW2(j,20:23)=[C2(1) C2(2) a2 b2];                               % Save center-x,center-y,a and b for the minimum bounding ellipse of each intersection
        IntersectionFlag=true;
        j=j+1;
    end
end
clear m n IntersectionFlag j i XLimBou YLimBou xmin xmax ymin ymax P A C a b X LeaderDet ObstacleDet LeaObsIntPoly;
clear m n IntersectionFlag j i XLimBou2 YLimBou2 xmin2 xmax2 ymin2 ymax2 P2 A2 C2 a2 b2 X2 LeaderDet2 ObstacleDet2;

% P.3.6. Get minimum bounding ellipse for each pair of intersections
[m,n]=size(LeaObsDetTW);
LeaObsDetCon = [ ];
for i=1:m-1
     xmin=LeaObsDetTW(i,16);xmax=LeaObsDetTW(i,17);ymin=LeaObsDetTW(i,18);ymax=LeaObsDetTW(i,19);           % Get first intersection
     PolyLowerTime=polyshape([xmin xmin xmax xmax],[ymin ymax ymax ymin]);                                  % Polyshape for first intersection
     xmin=LeaObsDetTW(i+1,16);xmax=LeaObsDetTW(i+1,17);ymin=LeaObsDetTW(i+1,18);ymax=LeaObsDetTW(i+1,19);   % Get second intersection
     PolyUpperTime=polyshape([xmin xmin xmax xmax],[ymin ymax ymax ymin]);                                  % Polyshape for second intersection
     PolyUnion(i,1)=union(PolyLowerTime,PolyUpperTime);                                                     % Union of the two intersections
     [XLimBou,YLimBou] = boundingbox(PolyUnion(i,1));                                                       % Bounding box for the union
     xmin=XLimBou(1);xmax=XLimBou(2);ymin=YLimBou(1);ymax=YLimBou(2);                                       % Prepare intersection coordinates for minimum bounding ellipse function
     P=[[xmin;ymin] [xmin;ymax] [xmax;ymax] [xmax;ymin]];
     [A,C]=MinVolEllipse(P,0.00001);                                                                        % Get a,b and coordinates for the center of the minimum bounding ellipse
     [a,b,C,X]=Ellipse_plot(A,C);
     LeaObsDetCon(i,:)=[LeaObsDetTW(i,1) LeaObsDetTW(i+1,1) (LeaObsDetTW(i,1)+LeaObsDetTW(i+1,1))/2 C(1) C(2) a b]; % Save center-x,center-y,a and b for the minimum bounding ellipse of each intersection pair         
end
clear m n i xmin xmax ymin ymax PolyLowerTime PolyUpperTime PolyUnion XLimBou YLimBou P A C a b C X;
save DetectionWindowX6.mat LeaObsDetTW LeaObsDetCon;  % Save the time window detectuions and intersections realted information - Save F

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
save DetectionWindowX6P.mat LeaObsDetTW2 LeaObsDetCon2;  % Save the time window detectuions and intersections realted information - Save F

% 4. Collect results
save TotalResultsX6.mat TotalResults;         % Save G
% ------------------------------------------------------------------------------------------------------------------------------------------------------------------
% End of Prepare data for next agent
% ------------------------------------------------------------------------------------------------------------------------------------------------------------------

% ------------------------------------------------------------------------------------------------------------------------------------------------------------------
% Verification plot and cleaning up
% ------------------------------------------------------------------------------------------------------------------------------------------------------------------
% Plots E. Follower and obstacle realtime progression (verification)
if PlotRealTimeProgression==1
   FigureX6B=figure('Name','Leader:  X5 | Follower: Agent X6 | Obstacle realtime progression - 2D');title('Leader: Agent X5 | Follower: Agent X6 | Obstacle realtime progression - 2D');
   xlim([-50 PlotXUpp]);ylim([PlotYLow PlotYUpp]);xlabel('x');ylabel('y');  
   hold on;
   scatter(Leader(1,2),Leader(1,3),'MarkerFaceColor','y');                 
   rectangle('Position',[xv(1) yv(1) xv(2)-xv(1) yv(2)-yv(1)],'EdgeColor','k');
   theta = linspace(0,2*pi,100);
   [m,n]=size(Obstacle);
   v=VideoWriter('X6_RealTimeObstacle.avi');
   open(v);
   for i=1:m
        x0=Obstacle(i,2);y0=Obstacle(i,3);R=Obstacle(i,4);
        h1=scatter(Leader(i,2),Leader(i,3),'MarkerFaceColor','r');
        h2=plot(x0 + R*cos(theta),y0 + R*sin(theta),'-k');
        h3=plot(x0 + (R+ObsSafRad)*cos(theta),y0 + (R+ObsSafRad)*sin(theta),'-k','LineStyle','--');
        F(i)=getframe(gcf);
        writeVideo(v,F(i));
        if i~=m
           delete(h1);
           delete(h2);
           delete(h3);
        end
  end
  close(v);
end
clear theta m n i x0 y0 R h1 h2 h3 F;

% Clear Mex files and ObstacleConstraints.mat
rmdir C:\FALCON.m.v1.24.2002191427\falcon\SetupA_Final\fm_constraints s
rmdir C:\FALCON.m.v1.24.2002191427\falcon\SetupA_Final\fm_models s
delete *.mexw64
delete ObstacleConstraints.mat
% ------------------------------------------------------------------------------------------------------------------------------------------------------------------
% End of Verification plot and cleaning up
% ------------------------------------------------------------------------------------------------------------------------------------------------------------------

% End of file