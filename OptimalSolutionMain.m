clear;clc;close all;                                             % Clear workspace 3.81
% ------------------------------------------------------------------------------------------------------------------------------------------------------------------
%  Optimal solution agent algorithm parameters
% ------------------------------------------------------------------------------------------------------------------------------------------------------------------
load LeaderX0.mat;
load AlgorithmParametersX0.mat;                      % Load save A
TimeLowerLimit=0;                                    % Final time is free but values are needed for discretization; lower limit, upper limit, step
TimeUpperLimit=300;    
TimeInitialGuess=(TimeLowerLimit+TimeUpperLimit)/2;    
PlotRealTimeProgression=1;                           % Plot suboptimal agent vs obstacle real time figure
% ------------------------------------------------------------------------------------------------------------------------------------------------------------------
% End of  Optimal solution agent algorithm parameters
% ------------------------------------------------------------------------------------------------------------------------------------------------------------------
 
% ------------------------------------------------------------------------------------------------------------------------------------------------------------------
%  Optimal Agent initialization
% ------------------------------------------------------------------------------------------------------------------------------------------------------------------
% Points must match that of the swarm's last agent
SamplingMultiplier=15;
PointPerSubproblem=DynamicsDiscretizationUpperLimit;
DynamicsDiscretizationUpperLimit=SamplingMultiplier*PointPerSubproblem;
% ------------------------------------------------------------------------------------------------------------------------------------------------------------------
% End of Optimal Agent initialization
% ------------------------------------------------------------------------------------------------------------------------------------------------------------------

% ------------------------------------------------------------------------------------------------------------------------------------------------------------------
%  Optimal Agent
% ------------------------------------------------------------------------------------------------------------------------------------------------------------------
% A. Define state space and controls
% State vector: Position axis - x (x), Position axis - y (y), Velocity axis - x (vx), Velocity axis - y (vy) and dummy state (time)
x_vec = [...
    falcon.State('x',RoadLowerLimitX,RoadUpperLimitX,RoadScalingX);...
    falcon.State('y',RoadLowerLimitY,RoadUpperLimitY,RoadScalingY);...
    falcon.State('vx',LowerVelocityLimitX,UpperVelocityLimitX,VelocityScaling);...
    falcon.State('vy',LowerVelocityLimitY,UpperVelocityLimitY,VelocityScaling);...
    falcon.State('time',TimeLowerLimit,TimeUpperLimit,1)];

% Control vector: Acceleration axis - x (ax), Acceleration axis - y (ay)
u_vec = [...
    falcon.Control('ax',LowerControlLimitX,UpperControlLimitX,AccelerationScaling);...      
    falcon.Control('ay',LowerControlLimitY,UpperControlLimitY,AccelerationScaling)];

% B. Final time                                          % Create a Falcon parameter, and by setting it as the final time of the phase Falcon solves a free final time problem
tf = falcon.Parameter('FinalTime', TimeInitialGuess,TimeLowerLimit,TimeUpperLimit,TimeStep);  

% C. Dynamics discretization                             % Time discretization grid used for the dynamics 
tau = linspace(0,DynamicsDiscretizationStep,DynamicsDiscretizationUpperLimit);

% D. Problem: Create a new Instance
problem = falcon.Problem('OptimalSolution');

% E. Problem: Add a new phase                            % Optimal agents solves one problem
phase = problem.addNewPhase(@OptimalSolutionProblemDefinition, x_vec, tau, PhaseInitialTime, tf);

% F. Phase: Add control grid                           % The control vector is discretized on time grid tau (dynamics discretization time grid)
phase.addNewControlGrid(u_vec, tau);

% G. Phase: Add model output
phase.Model.setModelOutputs([falcon.Output('x_out'); falcon.Output('y_out');falcon.Output('vx_out');falcon.Output('vy_out');falcon.Output('time_out');falcon.Output('ax_out');falcon.Output('ay_out')]);

% H. Phase: Add Lagrange cost function
OptimizeCostFunction=1;                     % If value is 0 then we optimize using only time
if OptimizeCostFunction==0                  % else we use the cost function
   problem.addNewParameterCost(tf);  
else
   Cost=falcon.Cost('Cost');
   phase.addNewLagrangeCost(@OptimalSolutionCost,Cost,tau);    
end      

% I. Phase: Add boundary conditions
phase.setInitialBoundaries([xi;yi;vxi;vyi;TimeLowerLimit]);
phase.setFinalBoundaries([x_vec(1);x_vec(2);x_vec(3);x_vec(4)],[xfa;yfa;vxfa;vyfa],[xfb;yfb;vxfb;vyfb]);  % Set final boundaries [xfa,xfb],[yfa,yfb] for states x,y and [vxfa,vxfb],[vyfa,vyfb] for vx,vy

% J. Phase: Add constraints (obstacles)
CeX=5;CeY=150;
save ObstacleOptimalSolutionConstraints.mat ObsIniX ObsIniY ObsIniR SpeedX SpeedY SpeedR ObsSafRad Divider LeaDecRad CeX CeY CeR;
pconMdl = falcon.PathConstraintBuilder('AntsPCon',[],x_vec,u_vec,[],@OptimalSolutionConstraints);
pathconstraints = [falcon.Constraint('Obstacle',0,inf);...
                   falcon.Constraint('CVX1',0,inf);falcon.Constraint('CVX2',0,inf);falcon.Constraint('CVY1',0,inf);falcon.Constraint('CVY2',0,inf);...
                   falcon.Constraint('CAX1',0,inf);falcon.Constraint('CAX2',0,inf);falcon.Constraint('CAY1',0,inf);falcon.Constraint('CAY2',0,inf)];
phase.addNewPathConstraint(@AntsPCon, pathconstraints, tau);
phase.addNewPathConstraint(@AntsPCon, pathconstraints, tau);                                    
pconMdl.Build();

% K. Problem: Prepare for solving
problem.Bake();

% L. Problem: Solve 
solver = falcon.solver.ipopt(problem);           % Solve with IPOPT
solver.Options.MajorIterLimit = 3000;            % Maximum number of iterations
solver.Options.MajorFeasTol = 1e-5;              % Feasibility tolerance
solver.Options.MajorOptTol = 1e-5;               % Optimality tolerance
solver.Solve();                                  % Solve command

% 1. Optimal agent trajectory
LeaderOptimal=[phase.RealTime' phase.StateGrid.Values(1:4,:)' phase.ControlGrids.Values' phase.LagrangeCostFunctions.OutputGrid.Values'];
save TotalResultsOptimalAgent LeaderOptimal;

% 2. Obstacle trajectory
Obstacle=[ ];
[m,n]=size(LeaderOptimal);
for i=1:m
    Obstacle(i,1)=LeaderOptimal(i,1);
    Obstacle(i,2)=ObsIniX+(LeaderOptimal(i,1)/Divider)*SpeedX;  % Obstacle center x axis
    Obstacle(i,3)=ObsIniY+(LeaderOptimal(i,1)/Divider)*SpeedY;  % Obstacle center y axis
    Obstacle(i,4)=ObsIniR+(LeaderOptimal(i,1)/Divider)*SpeedR; % Obstacle dynamic radius
end
Obstacle(:,5)=Obstacle(:,2)-(Obstacle(:,4)+ObsSafRad);
Obstacle(:,6)=Obstacle(:,2)+(Obstacle(:,4)+ObsSafRad);  
Obstacle(:,7)=Obstacle(:,3)-(Obstacle(:,4)+ObsSafRad);
Obstacle(:,8)=Obstacle(:,3)+(Obstacle(:,4)+ObsSafRad);
clear m n i;

% Plots optimal agent
% Plot 1. Optimal agent trajectory
FigureOptimalTrajectory=figure('Name','Optimal agent trajectory');title('Optimal agent trajectory');
axis([PlotXLow PlotXUpp PlotYLow PlotYUpp]);
xlabel('x');ylabel('y');   % Figure settings for axis
hold on;
rectangle('Position',[xv(1) yv(1) (xv(2)-xv(1)) (yv(2)-yv(1))],'EdgeColor','k');     % Goal area
scatter(LeaderOptimal(1,2),LeaderOptimal(1,3),'MarkerFaceColor','y');                % Plot initial point
CoordinateX=num2str(LeaderOptimal(1,2));                                             % Initial point x - axis string format
CoordinateY=num2str(LeaderOptimal(1,3));                                             % Initial point y - axis string format
StartCoordinates=strcat(' S(',CoordinateX,',',CoordinateY,')');                      % Initial point text format
text(Leader(1,2),Leader(1,3),StartCoordinates,'Color','k');                          % Plot initial point coordinates
scatter(LeaderOptimal(end,2),LeaderOptimal(end,3),'MarkerFaceColor','y');            % Plot final point
CoordinateX=num2str(LeaderOptimal(end,2));                                           % Final point x - axis string format
CoordinateY=num2str(LeaderOptimal(end,3));                                           % Final point y - axis string format
EndCoordinates=strcat(' G(',CoordinateX,',',CoordinateY,')');                        % Final point text format
text(Leader(end,2),Leader(end,3),EndCoordinates,'Color','k');                        % Plot final point coordinates
plot(LeaderOptimal(:,2),LeaderOptimal(:,3),'k');                                     % Plot one piece solution trajectory
savefig(FigureOptimalTrajectory,'OptimalAgentTrajectory.fig');
clear CoordinateX CoordinateY StartCoordinates EndCoordinates;

% Plot 2. Optimal agent vs obstacle realtime progression
if PlotRealTimeProgression
   FigureOptimalRealtime=figure('Name','Optimal agent real time progression');title('Optimal agent real time progression');
   axis([PlotXLow PlotXUpp PlotYLow PlotYUpp]);
   xlabel('x');ylabel('y');   % Figure settings for axis
   hold on;
   rectangle('Position',[xv(1) yv(1) (xv(2)-xv(1)) (yv(2)-yv(1))],'EdgeColor','k');     % Goal area
   scatter(LeaderOptimal(1,2),LeaderOptimal(1,3),'MarkerFaceColor','y');                % Plot initial point
   CoordinateX=num2str(LeaderOptimal(1,2));                                             % Initial point x - axis string format
   CoordinateY=num2str(LeaderOptimal(1,3));                                             % Initial point y - axis string format
   StartCoordinates=strcat(' S(',CoordinateX,',',CoordinateY,')');                      % Initial point text format
   text(Leader(1,2),Leader(1,3),StartCoordinates,'Color','k');                          % Plot initial point coordinates
   scatter(LeaderOptimal(end,2),LeaderOptimal(end,3),'MarkerFaceColor','y');            % Plot final point
   CoordinateX=num2str(LeaderOptimal(end,2));                                           % Final point x - axis string format
   CoordinateY=num2str(LeaderOptimal(end,3));                                           % Final point y - axis string format
   EndCoordinates=strcat(' G(',CoordinateX,',',CoordinateY,')');                        % Final point text format
   text(Leader(end,2),Leader(end,3),EndCoordinates,'Color','k');                        % Plot final point coordinates
   [m,n]=size(Obstacle);
   v=VideoWriter('OptimalAgent_Progression.avi');
   v.FrameRate=5;
   open(v);
   for i=1:100:m
        h1=scatter(LeaderOptimal(i,2),LeaderOptimal(i,3),'MarkerFaceColor','r');
        h2=rectangle('Position',[Obstacle(i,5) Obstacle(i,7) Obstacle(i,6)-Obstacle(i,5) Obstacle(i,8)-Obstacle(i,7)]);
        F(i)=getframe(gcf);
        writeVideo(v,F(i));
        if i~=m
           delete(h1);
           delete(h2);
        end
end
close(v);
clear theta m n i x0 y0 R F h1 h2 h3 CoordinateX CoordinateY StartCoordinates EndCoordinates;
end

% Display total cost
OptimalSolutionTime=solver.output.cpu;
save OptimalSolutionTime OptimalSolutionTime;
% ------------------------------------------------------------------------------------------------------------------------------------------------------------------
% End of Optimal Agent
% ------------------------------------------------------------------------------------------------------------------------------------------------------------------

% Clear Mex files and ObstacleConstraints.mat
rmdir C:\FALCON.m.v1.24.2002191427\falcon\SetupA_Final\fm_constraints s
rmdir C:\FALCON.m.v1.24.2002191427\falcon\SetupA_Final\fm_models s
delete *.mexw64
delete ObstacleOptimalSolutionConstraints.mat
% End of file