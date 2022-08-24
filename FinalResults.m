 clear;clc;close all;                                            % Clear workspace
% ------------------------------------------------------------------------------------------------------------------------------------------------------------------
% Load data from experiment
% ------------------------------------------------------------------------------------------------------------------------------------------------------------------
load TotalResultsX21.mat 
load TotalResultsOptimalAgent.mat
load TotalCost.mat
load SolverStats.mat
load TotalCost.mat;
load SubOptimalCosts.mat;
load AlgorithmParametersX1.mat delta sampling
% ------------------------------------------------------------------------------------------------------------------------------------------------------------------
% End of Load data from experiment
% ------------------------------------------------------------------------------------------------------------------------------------------------------------------

% ------------------------------------------------------------------------------------------------------------------------------------------------------------------
% A. Prepare Final Costs
% ------------------------------------------------------------------------------------------------------------------------------------------------------------------
% Experiment parameters
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

% A.1. Swarm costs
[m,n]=size(TotalResults);
mTotal=0;
TotalPoints=[ ];
for i=1:m
    for j=1:n
        if ~isempty(TotalResults{i,j})
           [mTemp,nTemp]=size(TotalResults{i,j});
           mTotal=mTotal+mTemp;
        end
    end
    TotalPoints=[TotalPoints; mTotal];
    mTotal=0;
end
clear m n i j mTemp nTemp mTotal;
points=min(TotalPoints);

Total={ };
NewTotal=[ ];
NewTotal2=[ ];NewTotal2A=[ ];NewTotal2B=[ ];
AgentCost=[ ];
AgentCounter=1;
FinalCostCounter=1;
[m,n]=size(TotalResults);
for i=1:m
     for j=1:n
          if ~isempty(TotalResults{i,j})
             Total=[Total;TotalResults{i,j}(:,1) TotalResults{i,j}(:,2) TotalResults{i,j}(:,3) TotalResults{i,j}(:,4) TotalResults{i,j}(:,5) TotalResults{i,j}(:,6) TotalResults{i,j}(:,7)];
          end
     end
     clear j;
     
     [r,t]=size(Total);
     for j=1:r
          NewTotal=[NewTotal;cell2mat(Total(j,1))];
     end
     clear r t j;
     
     steps=linspace(0,NewTotal(end,1),points);
     interv=steps(2)-steps(1);  
     NewTotal2(1,1)=NewTotal(1,1);NewTotal2(points,1)=NewTotal(end,1);
     NewTotal2(1,2)=NewTotal(1,2);NewTotal2(points,2)=NewTotal(end,2);
     NewTotal2(1,3)=NewTotal(1,3);NewTotal2(points,3)=NewTotal(end,3);
     NewTotal2(1,4)=NewTotal(1,4);NewTotal2(points,4)=NewTotal(end,4);
     NewTotal2(1,5)=NewTotal(1,5);NewTotal2(points,5)=NewTotal(end,5);
     NewTotal2(1,6)=NewTotal(1,6);NewTotal2(points,6)=NewTotal(end,6);
     NewTotal2(1,7)=NewTotal(1,7);NewTotal2(points,7)=NewTotal(end,7);
     
     NewTotal2Counter=2;
     for k=2:length(steps)-1
         x1=interp1(NewTotal(:,1),NewTotal(:,2),interv*(k-1));
         x2=interp1(NewTotal(:,1),NewTotal(:,3),interv*(k-1));
         x3=interp1(NewTotal(:,1),NewTotal(:,4),interv*(k-1));
         x4=interp1(NewTotal(:,1),NewTotal(:,5),interv*(k-1));
         u1=interp1(NewTotal(:,1),NewTotal(:,6),interv*(k-1));
         u2=interp1(NewTotal(:,1),NewTotal(:,7),interv*(k-1));
         NewTotal2(NewTotal2Counter,1)=interv*(k-1);
         NewTotal2(NewTotal2Counter,2)=x1;
         NewTotal2(NewTotal2Counter,3)=x2;
         NewTotal2(NewTotal2Counter,4)=x3;
         NewTotal2(NewTotal2Counter,5)=x4;
         NewTotal2(NewTotal2Counter,6)=u1;
         NewTotal2(NewTotal2Counter,7)=u2;
         NewTotal2Counter=NewTotal2Counter+1;
     end
     clear x1 x4 u1 u2 k;   
     
     [r,t]=size(NewTotal2);
     CounterA=1;CounterB=1;
     for i=1:r
         NewRadius=( CeR + (NewTotal2(r,1)/100) )^2;
         if (NewTotal2(i,2)-CeX)^2+(NewTotal2(i,3)-CeY)^2<=NewRadius^2
             NewTotal2A(CounterA,:)=NewTotal2(i,:);
             CounterA=CounterA+1;
         else
             NewTotal2B(CounterB,:)=NewTotal2(i,:);
             CounterB=CounterB+1;
         end
     end
     clear r t CounterA CounterB NewRadius;
     
     CostTime=NewTotal2(end,1);
     CostStatesA=trapz(0.01*NewTotal2A(:,2).^2+0.01*NewTotal2A(:,3).^2+NewTotal2A(:,4).^2+1*NewTotal2A(:,5).^2);
     CostControlsA=2*trapz(NewTotal2A(:,6).^2+NewTotal2A(:,7).^2);
     CostStatesB=trapz(0.01*NewTotal2B(:,2).^2+0.01*NewTotal2B(:,3).^2+NewTotal2B(:,4).^2+1*NewTotal2B(:,5).^2);
     CostControlsB=trapz(NewTotal2B(:,6).^2+NewTotal2B(:,7).^2);
     
     FinalCosts(FinalCostCounter,1)=CostTime+CostStatesA+CostControlsA+CostStatesB+CostControlsB;
     FinalCostCounter=FinalCostCounter+1;
     Total={ };
     NewTotal=[ ];
     NewTotal2=[ ];NewTotal2A=[ ];NewTotal2B=[ ];
     AgentCost=[ ];
     AgentCounter=1;
end
clear m n i j Total NewTotal AgentCost AgentCounter NewTotal2 NewTotal2Counter interv steps FinalCostCounter CostTime NewTotal2A NewTotal2B;

% A.2. One piece costs
[m,n]=size(LeaderOptimal);
steps=linspace(0,LeaderOptimal(m,1),points);
interv=steps(2)-steps(1);

 LeaderOptimal2(1,1)= LeaderOptimal(1,1);LeaderOptimal2(points,1)= LeaderOptimal(end,1);
 LeaderOptimal2(1,2)= LeaderOptimal(1,2);LeaderOptimal2(points,2)= LeaderOptimal(end,2);
 LeaderOptimal2(1,3)= LeaderOptimal(1,3);LeaderOptimal2(points,3)= LeaderOptimal(end,3);
 LeaderOptimal2(1,4)= LeaderOptimal(1,4);LeaderOptimal2(points,4)= LeaderOptimal(end,4);
 LeaderOptimal2(1,5)= LeaderOptimal(1,5);LeaderOptimal2(points,5)= LeaderOptimal(end,5);
 LeaderOptimal2(1,6)= LeaderOptimal(1,6);LeaderOptimal2(points,6)= LeaderOptimal(end,6);
 LeaderOptimal2(1,7)= LeaderOptimal(1,7);LeaderOptimal2(points,7)= LeaderOptimal(end,7);
 LeaderCostCounter=2;
for k=2:length(steps)-1
    x1=interp1(LeaderOptimal(:,1),LeaderOptimal(:,2),interv*(k-1));
    x2=interp1(LeaderOptimal(:,1),LeaderOptimal(:,3),interv*(k-1));
    x3=interp1(LeaderOptimal(:,1),LeaderOptimal(:,4),interv*(k-1));
    x4=interp1(LeaderOptimal(:,1),LeaderOptimal(:,5),interv*(k-1));
    u1=interp1(LeaderOptimal(:,1),LeaderOptimal(:,6),interv*(k-1));
    u2=interp1(LeaderOptimal(:,1),LeaderOptimal(:,7),interv*(k-1));
    LeaderOptimal2(LeaderCostCounter,1)=interv*(k-1);
    LeaderOptimal2(LeaderCostCounter,2)=x1;
    LeaderOptimal2(LeaderCostCounter,3)=x2;
    LeaderOptimal2(LeaderCostCounter,4)=x3;
    LeaderOptimal2(LeaderCostCounter,5)=x4;
    LeaderOptimal2(LeaderCostCounter,6)=u1;
    LeaderOptimal2(LeaderCostCounter,7)=u2;
    LeaderCostCounter=LeaderCostCounter+1;
end

[r,t]=size(LeaderOptimal2);
CounterA=1;CounterB=1;
for i=1:r
 NewRadius=( CeR + (LeaderOptimal2(r,1)/100) )^2;
 if (LeaderOptimal2(i,2)-CeX)^2+(LeaderOptimal2(i,3)-CeY)^2<=NewRadius^2
     LeaderOptimal2A(CounterA,:)=LeaderOptimal2(i,:);
     CounterA=CounterA+1;
 else
     LeaderOptimal2B(CounterB,:)=LeaderOptimal2(i,:);
     CounterB=CounterB+1;
 end
end
clear r t CounterA CounterB NewRadius;

CostTime=LeaderOptimal2(end,1);
CostStatesA=trapz(0.01*LeaderOptimal2A(:,2).^2+0.01*LeaderOptimal2A(:,3).^2+LeaderOptimal2A(:,4).^2+1*LeaderOptimal2A(:,5).^2);
CostControlsA=2*trapz(LeaderOptimal2A(:,6).^2+LeaderOptimal2A(:,7).^2);
CostStatesB=trapz(0.01*LeaderOptimal2B(:,2).^2+0.01*LeaderOptimal2B(:,3).^2+LeaderOptimal2B(:,4).^2+1*LeaderOptimal2B(:,5).^2);
CostControlsB=trapz(LeaderOptimal2B(:,6).^2+LeaderOptimal2B(:,7).^2);
     
TotalLeaderOptimalCost=CostTime+CostStatesA+CostControlsA+CostStatesB+CostControlsB;
clear m n steps interv k x3 x6 LeaderCostCounter TotalCost TotalPoints SwarmCosts SubOptimalCosts CostTime;
clear u1 u2 x1 x2 x4 CostStatesA CostControlsA CostStatesB CostControlsB LeaderOptimal2A LeaderOptimal2B LeaderOptimal2; 
% ------------------------------------------------------------------------------------------------------------------------------------------------------------------
% End of A. Prepare Final Costs
% ------------------------------------------------------------------------------------------------------------------------------------------------------------------

% ------------------------------------------------------------------------------------------------------------------------------------------------------------------
% B. Metrics
% ------------------------------------------------------------------------------------------------------------------------------------------------------------------
load SolverStats.mat
[m,n]=size(SolverStats);
SwarmRunTime=0;
TotalIterations=0;
for i=1:m
     for j=1:n
          if ~isempty(SolverStats{i,j})
             SwarmRunTime=SwarmRunTime+SolverStats{i,j}(end);
             TotalIterations=TotalIterations+SolverStats{i,j}(2);
          end
     end
end
clear m n i j;

[m,n]=size(TotalResults);
TotalProblemsSolved=0;
for i=2:m
    for j=1:n
        if ~isempty(TotalResults{i,j})
            TotalProblemsSolved=TotalProblemsSolved+1;
        end
    end
end
clear m n i j;
TotalProblemsSolved

% Time and cost error
load OptimalSolutionTime.mat;
load SubOptimalSolutionTime.mat;
OnePieceSolutionTime=OptimalSolutionTime;
OnePieceSolutionTime
SwarmRunTime
SubOptimalSolutionTime
SubOptimalAndSwarmTime=SubOptimalSolutionTime+SwarmRunTime
OnePieceMinusSwarmTotalTime=OnePieceSolutionTime-(SwarmRunTime+SubOptimalSolutionTime)
FirstAgentTimeToGoal=TotalResults{1,1}(end,1)
delta
FirstAgentTimeToGoalDivDelta=FirstAgentTimeToGoal/delta
FirstAgentCost=FinalCosts(1,1)
LastAgentCost=FinalCosts(end,1)
FirstAgentCostDivLastAgentCost=FinalCosts(1,1)/FinalCosts(end,1)
SwarmCost=FinalCosts(end,1)
OnePieceCost=TotalLeaderOptimalCost
SwarmMinusOnePiece_Cost=FinalCosts(end,1)-TotalLeaderOptimalCost

clear OnePieceSolutionTime SwarmRunTime SubOptimalSolutionTime OnePieceMinusSwarmTotalTime OnePieceTimeDivSwarmTotalTime FirstAgentTimeToGoal;
clear FirstAgentCost LastAgentCost FirstAgentCostDivLastAgentCost FirstAgentTimeToGoalDivDelta;
clear SubOptimalAndSwarmTime SwarmMinusOnePiece_Cost TotalIterations TotalProblemsSolved; 
% ------------------------------------------------------------------------------------------------------------------------------------------------------------------
% End of B. Metrics
% ------------------------------------------------------------------------------------------------------------------------------------------------------------------

% ------------------------------------------------------------------------------------------------------------------------------------------------------------------
% C. Verification
% ------------------------------------------------------------------------------------------------------------------------------------------------------------------
ConductOptimalityCheck=1;
if ConductOptimalityCheck>0    % Check if optimal solution was found in all sub-problems
    [m,n]=size(SolverStats);
    OptimalCheckerCounter=1;
    for i=1:m
        for j=1:n
            if ~isempty(SolverStats{i,j})
                if SolverStats{i,j}(1)==0
                   OptimalChecker(OptimalCheckerCounter)=0;
                else
                   OptimalChecker(OptimalCheckerCounter)=1; 
                end
                OptimalCheckerCounter=OptimalCheckerCounter+1;
            end
        end
    end

    if sum(OptimalChecker)==0
       disp('Optimal solution found in all sub-problems');
    else
        disp('Optimal solution not found in all sub-problems');
    end
    clear m n i j OptimalChecker;
end

ConductCostIncreaseCheck=1;
if ConductCostIncreaseCheck>0     % Check for cost increase
    [m,n]=size(FinalCosts);
    CostCheckerFlag=false;
    for i=1:m-1
        CostChecker(i,1)=FinalCosts(i+1)-FinalCosts(i);
    end
    
    [m,n]=size(CostChecker);
    for i=1:m
        if CostChecker(i,1)>0
           disp('Cost increase found');
           i
           CostCheckerFlag=true;
        end
    end
    if ~CostCheckerFlag
       disp('No cost increase found'); 
    end
    clear m n i;
end

DeltaChecker;
CollisionCheck=1;
if CollisionCheck>0
   CollisionChecker; % Check for collision
end

delta
sampling
ConvergenceChecker=abs(CostChecker(end,1))
clear ConductOptimalityCheck ConductCostIncreaseCheck CollisionCheck;
% ------------------------------------------------------------------------------------------------------------------------------------------------------------------
% End of C.Verification
% ------------------------------------------------------------------------------------------------------------------------------------------------------------------

% ------------------------------------------------------------------------------------------------------------------------------------------------------------------
% D. Plots
% ------------------------------------------------------------------------------------------------------------------------------------------------------------------
% ------------------------------------------------------------------------------------------------------------------------------------------------------------------
% D.1. OnePiece and Swarm first agent trajectories
% ------------------------------------------------------------------------------------------------------------------------------------------------------------------
% Prepare graph
FirstAgentOnePieceSolution=figure('Name','First agent and one piece solution vs obstacle');
PlotXLow=-20;PlotXUpp=20;
PlotYLow=0;PlotYUpp=355;
axis([PlotXLow PlotXUpp PlotYLow PlotYUpp]);
xlabel('x');ylabel('y');
hold on;

% Plot first agent and one piece solution
plot(TotalResults{1,1}(:,2),TotalResults{1,1}(:,3),'--k');
plot(LeaderOptimal(:,2),LeaderOptimal(:,3),'-k');
x=[2;10];y=[300;300];                       
line(x,y,'Color','k','LineStyle','-.');
rectangle('Position',[2 300 8 50],'Linestyle','-.');
text(5,0,'S','Color','k','VerticalAlignment','bottom','FontSize',13);
scatter(5,0,'MarkerFaceColor','k');
lgd=legend('Agent 1','One piece','Goal area','Location','northeast');
lgd.FontSize=12;
lgd.AutoUpdate='off';

PointsSubOptimal=[39];
SubOptimal(:,1)=TotalResults{1,1}(:,1);
SubOptimal(:,2)=TotalResults{1,1}(:,2);
SubOptimal(:,3)=TotalResults{1,1}(:,3);
[m,n]=size(PointsSubOptimal);
for i=1:n
    ToPlotObstacle(i,1)=SubOptimal(PointsSubOptimal(i),1); 
    ToPlotObstacle(i,9)=SubOptimal(PointsSubOptimal(i),2);  % Keep first agent coordinates x
    ToPlotObstacle(i,10)=SubOptimal(PointsSubOptimal(i),3); % Keep first agent coordinates y
end
ToPlotObstacle(:,2)=ObsIniX+(ToPlotObstacle(:,1)/Divider)*SpeedX;
ToPlotObstacle(:,3)=ObsIniY+(ToPlotObstacle(:,1)/Divider)*SpeedY; 
ToPlotObstacle(:,4)=ObsIniR+(ToPlotObstacle(:,1)/Divider)*SpeedR; 
ToPlotObstacle(:,5)=ToPlotObstacle(:,2)-(ToPlotObstacle(:,4));
ToPlotObstacle(:,6)=ToPlotObstacle(:,2)+(ToPlotObstacle(:,4));  
ToPlotObstacle(:,7)=ToPlotObstacle(:,3)-(ToPlotObstacle(:,4));
ToPlotObstacle(:,8)=ToPlotObstacle(:,3)+(ToPlotObstacle(:,4));

% First we plot the puddle and then the obstacles/positions of agents (to avoid overlay)
t=TotalResults{1,1}(PointsSubOptimal(end),1);
xCenter = CeX;
yCenter = CeY;
theta = 0 : 0.01 : 2*pi;
radius = CeR+t/100;
x = radius * cos(theta) + xCenter;
y = radius * sin(theta) + yCenter;
color=[220/255 220/255 220/255];
circles = plot(x, y);
fill(x, y, color)

[m,n]=size(ToPlotObstacle);
for i=1:m
    rectangle('Position',[ToPlotObstacle(i,5) ToPlotObstacle(i,7) ToPlotObstacle(i,6)-ToPlotObstacle(i,5) ToPlotObstacle(i,8)-ToPlotObstacle(i,7)]);
    scatter(ToPlotObstacle(i,2),ToPlotObstacle(i,3),125,'MarkerFaceColor','k');   
    scatter(ToPlotObstacle(i,9),ToPlotObstacle(i,10),125,'MarkerFaceColor','k');
end

% Display time
TotalResults{1,1}(PointsSubOptimal(1),1)

% Plot one piece solution point
LeadOpt{1,1}=LeaderOptimal(:,1);LeadOpt{1,2}=LeaderOptimal(:,2);LeadOpt{1,3}=LeaderOptimal(:,3);
TimeToPlot=TotalResults{1,1}(PointsSubOptimal(1),1);
AgentOneX=interp1(LeadOpt{1,1},LeadOpt{1,2},TimeToPlot);
AgentOneY=interp1(LeadOpt{1,1},LeadOpt{1,3},TimeToPlot);
scatter(AgentOneX,AgentOneY,125,'MarkerFaceColor','k'); 
savefig(FirstAgentOnePieceSolution,'FirstAgentOnePieceSolution.fig');

clear ToPlotObstacle PointsSubOptimal TimeToPlot;
clear m n i;
clear t xCenter yCenter theta radius x y color circles;
clear AgentOneX AgentOneY LeadOpt;

% Detailed obstacle avoidance for the first agent
% Prepare graph
FirstAgentDetailedAvoidance=figure('Name','First agent (detailed) vs obstacle');
PlotXLow=-5;PlotXUpp=15;
PlotYLow=135;PlotYUpp=165;
axis([PlotXLow PlotXUpp PlotYLow PlotYUpp]);
xlabel('x');ylabel('y');
hold on;

% Plot first agent and one piece solution
plot(TotalResults{1,1}(:,2),TotalResults{1,1}(:,3),'--k');
x=[2;10];y=[300;300];                       
line(x,y,'Color','k','LineStyle','-.');
rectangle('Position',[2 300 8 50],'Linestyle','-.');
text(5,0,'S','Color','k','VerticalAlignment','bottom','FontSize',13);
scatter(5,0,'MarkerFaceColor','k');
lgd=legend('Agent 1','Location','northeast');
lgd.FontSize=10;
lgd.AutoUpdate='off';
clear x y;

v=VideoWriter('FirstAgentDetailedAvoidance.avi');
v.FrameRate=1;
open(v);
PointsSubOptimal=[38 39 40 41 42];
[m,n]=size(PointsSubOptimal);
for i=1:n
    ToPlotObstacle(i,1)=SubOptimal(PointsSubOptimal(i),1); 
    ToPlotObstacle(i,9)=SubOptimal(PointsSubOptimal(i),2);  % Keep first agent coordinates x
    ToPlotObstacle(i,10)=SubOptimal(PointsSubOptimal(i),3); % Keep first agent coordinates y
end
ToPlotObstacle(:,2)=ObsIniX+(ToPlotObstacle(:,1)/Divider)*SpeedX;
ToPlotObstacle(:,3)=ObsIniY+(ToPlotObstacle(:,1)/Divider)*SpeedY; 
ToPlotObstacle(:,4)=ObsIniR+(ToPlotObstacle(:,1)/Divider)*SpeedR; 
ToPlotObstacle(:,5)=ToPlotObstacle(:,2)-(ToPlotObstacle(:,4));
ToPlotObstacle(:,6)=ToPlotObstacle(:,2)+(ToPlotObstacle(:,4));  
ToPlotObstacle(:,7)=ToPlotObstacle(:,3)-(ToPlotObstacle(:,4));
ToPlotObstacle(:,8)=ToPlotObstacle(:,3)+(ToPlotObstacle(:,4));

[m,n]=size(ToPlotObstacle);
for i=1:m
    if i==m-1 || i==m-1
       PlotXLow=-5;PlotXUpp=15;
       PlotYLow=135;PlotYUpp=165;
       axis([PlotXLow PlotXUpp PlotYLow PlotYUpp]);
       xlabel('x');ylabel('y');
    end
    t=ToPlotObstacle(i,1);
    DisplayTime=strcat('t=','',num2str(t));
    xCenter = CeX;
    yCenter = CeY;
    theta = 0 : 0.01 : 2*pi;
    radius = CeR+t/100;
    x = radius * cos(theta) + xCenter;
    y = radius * sin(theta) + yCenter;
    color=[220/255 220/255 220/255];
    circles = plot(x, y);
    fill(x, y, color)
    clear t xCenter yCenter theta radius x y color circles;
    
    h1=rectangle('Position',[ToPlotObstacle(i,5) ToPlotObstacle(i,7) ToPlotObstacle(i,6)-ToPlotObstacle(i,5) ToPlotObstacle(i,8)-ToPlotObstacle(i,7)]);
    h2=scatter(ToPlotObstacle(i,9),ToPlotObstacle(i,10),'MarkerFaceColor','r');
    h3=scatter(ToPlotObstacle(i,2),ToPlotObstacle(i,3),'MarkerFaceColor','k');   
    if i==m-1 || i==m
       h4=text(12,160,DisplayTime,'Color','k','FontSize',13); 
    else
       h4=text(5,160,DisplayTime,'Color','k','FontSize',13); 
    end
    clear DisplayTime;
    
    F(i)=getframe(gcf);
    writeVideo(v,F(i));
    if i~=m
       delete(h1);delete(h2);
       delete(h3);delete(h4);
    end
end
close(v);
% ------------------------------------------------------------------------------------------------------------------------------------------------------------------
% End of D.1. OnePiece and Swarm first agent trajectories
% ------------------------------------------------------------------------------------------------------------------------------------------------------------------
% ------------------------------------------------------------------------------------------------------------------------------------------------------------------
% D.2. Swarm progression
% ------------------------------------------------------------------------------------------------------------------------------------------------------------------
% Get t - x - y for all agents in cell format
[m,n]=size(TotalResults);
for i=1:m
    for j=1:n
        if ~isempty(TotalResults{i,j})
            temp=TotalResults{i,j}(:,1);
            TimePerAgentCell{j,i}=temp;
            clear temp;
            temp=TotalResults{i,j}(:,2);
            XPerAgentCell{j,i}=temp;
            clear temp;
            temp=TotalResults{i,j}(:,3);
            YPerAgentCell{j,i}=temp;
            clear temp;
        end
    end
end
clear m n i j;

% Get t - x - y for all agents in matrix format
[r,t]=size(TimePerAgentCell);
TimePerAgent=[ ];
XPerAgent=[ ];
YPerAgent=[ ];
for j=1:t
    TimePerAgent{j,1}=cell2mat(TimePerAgentCell(:,j));
    XPerAgent{j,1}=cell2mat(XPerAgentCell(:,j));
    YPerAgent{j,1}=cell2mat(YPerAgentCell(:,j));
end
clear r t j TimePerAgentCell XPerAgentCell YPerAgentCell;

% Prepare graph
SwarmProgression=figure('Name','Swarm progression');
PlotXLow=-10;PlotXUpp=20;
PlotYLow=0;PlotYUpp=355;
axis([PlotXLow PlotXUpp PlotYLow PlotYUpp]);
xlabel('x');ylabel('y');
hold on;

% Plot 2nd, 3rd, 4th, and last agent
plot(XPerAgent{2,1},YPerAgent{2,1},'--k')
plot(XPerAgent{3,1},YPerAgent{3,1},'-.k')
plot(XPerAgent{4,1},YPerAgent{4,1},':k')
plot(XPerAgent{end,1},YPerAgent{end,1},'k')
x=[2;10];y=[300;300];                       
line(x,y,'Color','k','LineStyle','--');
rectangle('Position',[2 300 8 50],'Linestyle','-.');
lgd=legend('Agent 2','Agent 3','Agent 4','Agent 22','Goal area','Location','northeast');
lgd.FontSize=12;
lgd.AutoUpdate='off';
clear x y;

PointSuboptimal=44;
% Plot puddle
TimeToPlot=SubOptimal(PointSuboptimal,1); 
xCenter = CeX;
yCenter = CeY;
theta = 0 : 0.01 : 2*pi;
radius = CeR+TimeToPlot/100;
x = radius * cos(theta) + xCenter;
y = radius * sin(theta) + yCenter;
color=[220/255 220/255 220/255];
circles = plot(x, y);
fill(x, y, color)
clear theta radius x y color circles;

SubOptimal(:,1)=TotalResults{1,1}(:,1);
SubOptimal(:,2)=TotalResults{1,1}(:,2);
SubOptimal(:,3)=TotalResults{1,1}(:,3);
[m,n]=size(SubOptimal);
for i=1:m
    ToPlotObstacle(:,1)=SubOptimal(PointSuboptimal,1); 
    ToPlotObstacle(:,9)=SubOptimal(PointSuboptimal,2);
    ToPlotObstacle(:,10)=SubOptimal(PointSuboptimal,3);
end
ToPlotObstacle(:,2)=ObsIniX+(ToPlotObstacle(:,1)/Divider)*SpeedX;
ToPlotObstacle(:,3)=ObsIniY+(ToPlotObstacle(:,1)/Divider)*SpeedY; 
ToPlotObstacle(:,4)=ObsIniR+(ToPlotObstacle(:,1)/Divider)*SpeedR; 
ToPlotObstacle(:,5)=ToPlotObstacle(:,2)-ToPlotObstacle(:,4);
ToPlotObstacle(:,6)=ToPlotObstacle(:,2)+ToPlotObstacle(:,4);  
ToPlotObstacle(:,7)=ToPlotObstacle(:,3)-ToPlotObstacle(:,4);
ToPlotObstacle(:,8)=ToPlotObstacle(:,3)+ToPlotObstacle(:,4);
clear m n i;

[m,n]=size(ToPlotObstacle);
for i=1:m
    rectangle('Position',[ToPlotObstacle(i,5) ToPlotObstacle(i,7) ToPlotObstacle(i,6)-ToPlotObstacle(i,5) ToPlotObstacle(i,8)-ToPlotObstacle(i,7)]);
    scatter(ToPlotObstacle(i,2),ToPlotObstacle(i,3),125,'MarkerFaceColor','k');
%   TextToPlot=strcat('O');
%   text(ToPlotObstacle(i,2),ToPlotObstacle(i,3),TextToPlot,'Color','k','FontSize',13); 
end
clear m n i;

% Plot puddle center P
scatter(xCenter,yCenter,125,'k','x');
% CoordinateX=num2str(xCenter,'%.2f');
% CoordinateY=num2str(yCenter,'%.2f');
% TextToPlot=strcat('P');
% text(xCenter,yCenter,TextToPlot,'Color','k','FontSize',13); 
% clear xCenter yCenter theta radius x y CoordinateX CoordinateY;

% Plot second agent point
AgentOneX=interp1(TimePerAgent{2,1},XPerAgent{2,1},TimeToPlot);
AgentOneY=interp1(TimePerAgent{2,1},YPerAgent{2,1},TimeToPlot);
scatter(AgentOneX,AgentOneY,125,'MarkerFaceColor','k');

% Plot third agent point
AgentOneX=interp1(TimePerAgent{3,1},XPerAgent{3,1},TimeToPlot);
AgentOneY=interp1(TimePerAgent{3,1},YPerAgent{3,1},TimeToPlot);
scatter(AgentOneX,AgentOneY,125,'MarkerFaceColor','k');

% Plot fourth agent point
AgentOneX=interp1(TimePerAgent{4,1},XPerAgent{4,1},TimeToPlot);
AgentOneY=interp1(TimePerAgent{4,1},YPerAgent{4,1},TimeToPlot);
scatter(AgentOneX,AgentOneY,125,'MarkerFaceColor','k');

% Plot last agent point
AgentOneX=interp1(TimePerAgent{end,1},XPerAgent{end,1},TimeToPlot);
AgentOneY=interp1(TimePerAgent{end,1},YPerAgent{end,1},TimeToPlot);
scatter(AgentOneX,AgentOneY,125,'MarkerFaceColor','k');

% Plot starting point
text(5,0,'S','Color','k','VerticalAlignment','bottom','FontSize',13);
scatter(5,0,'MarkerFaceColor','k');

TimeToPlot
savefig(SwarmProgression,'SwarmProgression.fig');

clear x y lgd m n i ToPlotObstacle temp TextToPlot AgentOneX AgentOneY TimeToPlot;
clear x1 x2 x3 x4 Keep u1 u2 XPerAgent xq xv YPerAgent yq yv t TimePerAgent;
clear PointSuboptimal h1 h2 h3 h4 F;
% ------------------------------------------------------------------------------------------------------------------------------------------------------------------
% End of D.2. Swarm progression
% ------------------------------------------------------------------------------------------------------------------------------------------------------------------
% ------------------------------------------------------------------------------------------------------------------------------------------------------------------
% End of experiment plots
% ------------------------------------------------------------------------------------------------------------------------------------------------------------------

% ------------------------------------------------------------------------------------------------------------------------------------------------------------------
% Videos
% ------------------------------------------------------------------------------------------------------------------------------------------------------------------
[m,n]=size(TotalResults);
for i=1:m
    for j=1:n
        if ~isempty(TotalResults{i,j})
            temp=TotalResults{i,j}(:,1);
            TimePerAgentCell{j,i}=temp;
            clear temp;
            temp=TotalResults{i,j}(:,2);
            XPerAgentCell{j,i}=temp;
            clear temp;
            temp=TotalResults{i,j}(:,3);
            YPerAgentCell{j,i}=temp;
            clear temp;
            temp=TotalResults{i,j}(:,4);
            VXPerAgentCell{j,i}=temp;
            clear temp;
            temp=TotalResults{i,j}(:,5);
            VYPerAgentCell{j,i}=temp;
            clear temp;
        end
    end
end
clear m n i j;

% Get t - x - y for all agents in matrix format
[r,t]=size(TimePerAgentCell);
TimePerAgent=[ ];
XPerAgent=[ ];
YPerAgent=[ ];
VXPerAgent=[ ];
VYPerAgent=[ ];
for j=1:t
    TimePerAgent{j,1}=cell2mat(TimePerAgentCell(:,j));
    XPerAgent{j,1}=cell2mat(XPerAgentCell(:,j));
    YPerAgent{j,1}=cell2mat(YPerAgentCell(:,j));
    VXPerAgent{j,1}=cell2mat(VXPerAgentCell(:,j));
    VYPerAgent{j,1}=cell2mat(VYPerAgentCell(:,j));
end
clear r t j;

% Swarm videos
InterpPoints=101;

% First agent
FigureFirstAgent=figure('Name','First agent vs obstacle');title('First agent vs obstacle');
PlotXLow=-50;PlotXUpp=50;
PlotYLow=0;PlotYUpp=355;
axis([PlotXLow PlotXUpp PlotYLow PlotYUpp]);
xlabel('x');ylabel('y');
hold on;
x=[2;10];y=[300;300];                       
line(x,y,'Color','k');
rectangle('Position',[2 300 8 50]);
lgd=legend('Goal area','Location','northeast');
lgd.AutoUpdate='off';
text(5,0,'S','Color','k','VerticalAlignment','bottom','FontSize',13);
scatter(5,0,'MarkerFaceColor','k');

FinalTime=TotalResults{1,1}(end,1);
steps=linspace(0,FinalTime,InterpPoints);

TempTime=TimePerAgent{1,1}(:);
TempX=XPerAgent{1,1}(:);
TempY=YPerAgent{1,1}(:);
TempVX=VXPerAgent{1,1}(:);
TempVT=VYPerAgent{1,1}(:);
[m,n]=size(steps);
for i=1:n
    ToPlot(i,1)=steps(i);
    ToPlot(i,2)=interp1(TempTime,TempX,steps(i));
    ToPlot(i,3)=interp1(TempTime,TempY,steps(i));
    ToPlot(i,11)=interp1(TempTime,TempVX,steps(i));
    ToPlot(i,12)=interp1(TempTime,TempVT,steps(i));
end
clear TempTime TempTime2 TempX TempY TempX2 TempY2 m n i FinalTime steps;

ToPlot(:,4)=ObsIniX+(ToPlot(:,1)/Divider)*SpeedX;
ToPlot(:,5)=ObsIniY+(ToPlot(:,1)/Divider)*SpeedY; 
ToPlot(:,6)=ObsIniR+(ToPlot(:,1)/Divider)*SpeedR; 
ToPlot(:,7)=ToPlot(:,4)-ToPlot(:,6);
ToPlot(:,8)=ToPlot(:,4)+ToPlot(:,6);  
ToPlot(:,9)=ToPlot(:,5)-ToPlot(:,6);
ToPlot(:,10)=ToPlot(:,5)+ToPlot(:,6);

v=VideoWriter('FirstAgent.avi');
v.FrameRate=20;
open(v);

% x=[22;28];y=[20;20];                       
% line(x,y,'Color','k');
% text(22,20,'-3','Color','k','VerticalAlignment','bottom','HorizontalAlignment','center','FontSize',10);
% text(25,20,'0','Color','k','VerticalAlignment','bottom','HorizontalAlignment','center','FontSize',10);
% text(28,20,'3','Color','k','VerticalAlignment','bottom','HorizontalAlignment','center','FontSize',10);
% text(25,32,'vx1','Color','k','VerticalAlignment','bottom','HorizontalAlignment','center','FontSize',10);
% h3=quiver(25,0,0,20,'LineWidth',1,'MaxHeadSize',0.5,'color',[0 0 0]);
% 
% x=[32;38];y=[20;20];                       
% line(x,y,'Color','k');
% text(32,20,'-3','Color','k','VerticalAlignment','bottom','HorizontalAlignment','center','FontSize',10);
% text(35,20,'0','Color','k','VerticalAlignment','bottom','HorizontalAlignment','center','FontSize',10);
% text(38,20,'3','Color','k','VerticalAlignment','bottom','HorizontalAlignment','center','FontSize',10);
% text(35,32,'vy1','Color','k','VerticalAlignment','bottom','HorizontalAlignment','center','FontSize',10);
% h4=quiver(35,0,0,20,'LineWidth',1,'MaxHeadSize',0.5,'color',[0 0 0]);

[m,n]=size(ToPlot);
for i=1:m
    t=ToPlot(i,1);
    xCenter = CeX;
    yCenter = CeY;
    theta = 0 : 0.01 : 2*pi;
    radius = CeR+t/100;
    x = radius * cos(theta) + xCenter;
    y = radius * sin(theta) + yCenter;
    color=[220/255 220/255 220/255];
    circles = plot(x, y);
    fill(x, y, color)
    clear t xCenter yCenter theta radius color circles;
    h1=rectangle('Position',[ToPlot(i,7) ToPlot(i,9) ToPlot(i,8)-ToPlot(i,7) ToPlot(i,10)-ToPlot(i,9)]);
    h2=scatter(ToPlot(i,2),ToPlot(i,3),'MarkerFaceColor','r');
    F(i)=getframe(gcf);
    writeVideo(v,F(i));
    if i~=m
       delete(h1);
       delete(h2);
    end
%     delete(h3);delete(h4);
%     h3=quiver(25+ToPlot(i,11),0,0,20,'LineWidth',1,'MaxHeadSize',0.5,'color',[1 0 0]);
%     h4=quiver(35+ToPlot(i,12),0,0,20,'LineWidth',1,'MaxHeadSize',0.5,'color',[1 0 0]);
end
close(v);

ToPlotAgent1=ToPlot;
[m,n]=size(ToPlotAgent1);
for i=1:m
    if ~isnan(ToPlotAgent1(i,2))
        LastX=ToPlotAgent1(i,2);
    else
        ToPlotAgent1(i,2)=LastX;
    end
    if ~isnan(ToPlotAgent1(i,3))
        LastY=ToPlotAgent1(i,3);
    else
        ToPlotAgent1(i,3)=LastY;
    end
end
clear m n i h1 h2 ToPlot x y v h1 h2 h3 h4 F LastX Last Y;

% Second agent
FigureSecondAgent=figure('Name','Second vs first agent');title('Second vs first agent');
PlotXLow=-50;PlotXUpp=50;
PlotYLow=0;PlotYUpp=355;
axis([PlotXLow PlotXUpp PlotYLow PlotYUpp]);
xlabel('x');ylabel('y');
hold on;
x=[2;10];y=[300;300];                       
line(x,y,'Color','k');
rectangle('Position',[2 300 8 50]);
plot(TotalResults{1,1}(:,2),TotalResults{1,1}(:,3),'-.k');
plot(XPerAgent{2,1},YPerAgent{2,1},'--k');
lgd=legend('Goal area','\color{blue} Agent 1','\color{red} Agent 2','Location','northeast');
lgd.FontSize=12;
lgd.AutoUpdate='off';
text(5,0,'S','Color','k','VerticalAlignment','bottom','FontSize',13);
scatter(5,0,'MarkerFaceColor','k');

FinalTime=TotalResults{1,1}(end,1);
steps=linspace(0,FinalTime,InterpPoints);

TempTime=TimePerAgent{2,1}(:);
TempX=XPerAgent{2,1}(:);
TempY=YPerAgent{2,1}(:);
TempVX=VXPerAgent{2,1}(:);
TempVT=VYPerAgent{2,1}(:);
[m,n]=size(steps);
for i=1:n
    ToPlot(i,1)=steps(i);
    ToPlot(i,2)=interp1(TempTime,TempX,steps(i));
    ToPlot(i,3)=interp1(TempTime,TempY,steps(i));
    ToPlot(i,11)=interp1(TempTime,TempVX,steps(i));
    ToPlot(i,12)=interp1(TempTime,TempVT,steps(i));
end
clear TempTime TempTime2 TempX TempY TempX2 TempY2 m n i FinalTime steps;

ToPlot(:,4)=ObsIniX+(ToPlot(:,1)/Divider)*SpeedX;
ToPlot(:,5)=ObsIniY+(ToPlot(:,1)/Divider)*SpeedY; 
ToPlot(:,6)=ObsIniR+(ToPlot(:,1)/Divider)*SpeedR; 
ToPlot(:,7)=ToPlot(:,4)-ToPlot(:,6);
ToPlot(:,8)=ToPlot(:,4)+ToPlot(:,6);  
ToPlot(:,9)=ToPlot(:,5)-ToPlot(:,6);
ToPlot(:,10)=ToPlot(:,5)+ToPlot(:,6);

[m,n]=size(ToPlot);
for i=1:m
    if ~isnan(ToPlot(i,2))
        LastX=ToPlot(i,2);
    else
        ToPlot(i,2)=LastX;
    end
    if ~isnan(ToPlot(i,3))
        LastY=ToPlot(i,3);
    else
        ToPlot(i,3)=LastY;
    end
end

v=VideoWriter('SecondAgent.avi');
v.FrameRate=20;
open(v);

% x=[22;28];y=[20;20];                       
% line(x,y,'Color','k');
% text(22,20,'-3','Color','k','VerticalAlignment','bottom','HorizontalAlignment','center','FontSize',10);
% text(25,20,'0','Color','k','VerticalAlignment','bottom','HorizontalAlignment','center','FontSize',10);
% text(28,20,'3','Color','k','VerticalAlignment','bottom','HorizontalAlignment','center','FontSize',10);
% text(25,32,'vx2','Color','k','VerticalAlignment','bottom','HorizontalAlignment','center','FontSize',10);
% h4=quiver(25,0,0,20,'LineWidth',1,'MaxHeadSize',0.5,'color',[0 0 0]);
% 
% x=[32;38];y=[20;20];                       
% line(x,y,'Color','k');
% text(32,20,'-3','Color','k','VerticalAlignment','bottom','HorizontalAlignment','center','FontSize',10);
% text(35,20,'0','Color','k','VerticalAlignment','bottom','HorizontalAlignment','center','FontSize',10);
% text(38,20,'3','Color','k','VerticalAlignment','bottom','HorizontalAlignment','center','FontSize',10);
% text(35,32,'vy2','Color','k','VerticalAlignment','bottom','HorizontalAlignment','center','FontSize',10);
% h5=quiver(35,0,0,20,'LineWidth',1,'MaxHeadSize',0.5,'color',[0 0 0]);
% 
% x=[22;28];y=[80;80];                       
% line(x,y,'Color','k');
% text(22,80,'-3','Color','k','VerticalAlignment','bottom','HorizontalAlignment','center','FontSize',10);
% text(25,80,'0','Color','k','VerticalAlignment','bottom','HorizontalAlignment','center','FontSize',10);
% text(28,80,'3','Color','k','VerticalAlignment','bottom','HorizontalAlignment','center','FontSize',10);
% text(25,92,'vx1','Color','k','VerticalAlignment','bottom','HorizontalAlignment','center','FontSize',10);
% h6=quiver(25,60,0,20,'LineWidth',1,'MaxHeadSize',0.5,'color',[0 0 0]);
% 
% x=[32;38];y=[80;80];                       
% line(x,y,'Color','k');
% text(32,80,'-3','Color','k','VerticalAlignment','bottom','HorizontalAlignment','center','FontSize',10);
% text(35,80,'0','Color','k','VerticalAlignment','bottom','HorizontalAlignment','center','FontSize',10);
% text(38,80,'3','Color','k','VerticalAlignment','bottom','HorizontalAlignment','center','FontSize',10);
% text(35,92,'vy1','Color','k','VerticalAlignment','bottom','HorizontalAlignment','center','FontSize',10);
% h7=quiver(35,60,0,20,'LineWidth',1,'MaxHeadSize',0.5,'color',[0 0 0]);
% 
% x=[42;48];y=[80;80];                       
% line(x,y,'Color','k');
% text(45,80,'vy1-vy2','Color','k','VerticalAlignment','bottom','HorizontalAlignment','center','FontSize',10);
% h8=text(45,60,'0','Color','k','VerticalAlignment','bottom','HorizontalAlignment','center','FontSize',10);

[m,n]=size(ToPlot);
for i=1:m
    % Plot puddle
    t=ToPlot(i,1);
    xCenter = CeX;
    yCenter = CeY;
    theta = 0 : 0.01 : 2*pi;
    radius = CeR+t/100;
    x = radius * cos(theta) + xCenter;
    y = radius * sin(theta) + yCenter;
    color=[220/255 220/255 220/255];
    circles = plot(x, y);
    fill(x, y, color); 
    clear t xCenter yCenter theta radius color circles;
    % Plot obstacle
    h1=rectangle('Position',[ToPlot(i,7) ToPlot(i,9) ToPlot(i,8)-ToPlot(i,7) ToPlot(i,10)-ToPlot(i,9)]);
    h2=scatter(ToPlot(i,2),ToPlot(i,3),'MarkerFaceColor','r');
    h3=scatter(ToPlotAgent1(i,2),ToPlotAgent1(i,3),'MarkerFaceColor','b');
    F(i)=getframe(gcf);
    writeVideo(v,F(i));
    if i~=m
       delete(h1);
       delete(h2);
       delete(h3);
    end
%     delete(h4);delete(h5);delete(h6);delete(h7);delete(h8);
%     h4=quiver(25+ToPlot(i,11),0,0,20,'LineWidth',1,'MaxHeadSize',0.5,'color',[1 0 0]);
%     h5=quiver(35+ToPlot(i,12),0,0,20,'LineWidth',1,'MaxHeadSize',0.5,'color',[1 0 0]);
%     h6=quiver(25+ToPlotAgent1(i,11),60,0,20,'LineWidth',1,'MaxHeadSize',0.5,'color',[0 0 1]);
%     h7=quiver(35+ToPlotAgent1(i,12),60,0,20,'LineWidth',1,'MaxHeadSize',0.5,'color',[0 0 1]);
%     if ~isnan(ToPlotAgent1(i,12)-ToPlot(i,12))
%        h8=text(45,60,num2str(ToPlotAgent1(i,12)-ToPlot(i,12),3),'Color','k','VerticalAlignment','bottom','HorizontalAlignment','center','FontSize',10);
%     else
%        h8=text(45,60,'0','Color','k','VerticalAlignment','bottom','HorizontalAlignment','center','FontSize',10); 
%     end
end
close(v);

ToPlotAgent2=ToPlot;
clear m n i h1 h2 ToPlot x y v h1 h2 h3 h4 F LastX Last Y;

% Third agent
FigureThirdAgent=figure('Name','Third vs second agent');title('Third vs second agent');
PlotXLow=-50;PlotXUpp=50;
PlotYLow=0;PlotYUpp=355;
axis([PlotXLow PlotXUpp PlotYLow PlotYUpp]);
xlabel('x');ylabel('y');
hold on;
x=[2;10];y=[300;300];                       
line(x,y,'Color','k');
rectangle('Position',[2 300 8 50]);
plot(XPerAgent{2,1},YPerAgent{2,1},'-.k');
plot(XPerAgent{3,1},YPerAgent{3,1},'--k');
lgd=legend('Goal area','\color{blue} Agent 2','\color{red} Agent 3','Location','northeast');
lgd.FontSize=12;
lgd.AutoUpdate='off';
text(5,0,'S','Color','k','VerticalAlignment','bottom','FontSize',13);
scatter(5,0,'MarkerFaceColor','k');

FinalTime=TimePerAgent{1,1}(end);
steps=linspace(0,FinalTime,InterpPoints);

TempTime=TimePerAgent{3,1}(:);
TempX=XPerAgent{3,1}(:);
TempY=YPerAgent{3,1}(:);
TempVX=VXPerAgent{3,1}(:);
TempVT=VYPerAgent{3,1}(:);
[m,n]=size(steps);
for i=1:n
    ToPlot(i,1)=steps(i);
    ToPlot(i,2)=interp1(TempTime,TempX,steps(i));
    ToPlot(i,3)=interp1(TempTime,TempY,steps(i));
    ToPlot(i,11)=interp1(TempTime,TempVX,steps(i));
    ToPlot(i,12)=interp1(TempTime,TempVT,steps(i));
end
clear TempTime TempTime2 TempX TempY TempX2 TempY2 m n i FinalTime steps;

ToPlot(:,4)=ObsIniX+(ToPlot(:,1)/Divider)*SpeedX;
ToPlot(:,5)=ObsIniY+(ToPlot(:,1)/Divider)*SpeedY; 
ToPlot(:,6)=ObsIniR+(ToPlot(:,1)/Divider)*SpeedR; 
ToPlot(:,7)=ToPlot(:,4)-ToPlot(:,6);
ToPlot(:,8)=ToPlot(:,4)+ToPlot(:,6);  
ToPlot(:,9)=ToPlot(:,5)-ToPlot(:,6);
ToPlot(:,10)=ToPlot(:,5)+ToPlot(:,6);

[m,n]=size(ToPlot);
for i=1:m
    if ~isnan(ToPlot(i,2))
        LastX=ToPlot(i,2);
    else
        ToPlot(i,2)=LastX;
    end
    if ~isnan(ToPlot(i,3))
        LastY=ToPlot(i,3);
    else
        ToPlot(i,3)=LastY;
    end
end

v=VideoWriter('ThirdAgent.avi');
v.FrameRate=20;
open(v);

% x=[22;28];y=[20;20];                       
% line(x,y,'Color','k');
% text(22,20,'-3','Color','k','VerticalAlignment','bottom','HorizontalAlignment','center','FontSize',10);
% text(25,20,'0','Color','k','VerticalAlignment','bottom','HorizontalAlignment','center','FontSize',10);
% text(28,20,'3','Color','k','VerticalAlignment','bottom','HorizontalAlignment','center','FontSize',10);
% text(25,32,'vx3','Color','k','VerticalAlignment','bottom','HorizontalAlignment','center','FontSize',10);
% h4=quiver(25,0,0,20,'LineWidth',1,'MaxHeadSize',0.5,'color',[0 0 0]);
% 
% x=[32;38];y=[20;20];                       
% line(x,y,'Color','k');
% text(32,20,'-3','Color','k','VerticalAlignment','bottom','HorizontalAlignment','center','FontSize',10);
% text(35,20,'0','Color','k','VerticalAlignment','bottom','HorizontalAlignment','center','FontSize',10);
% text(38,20,'3','Color','k','VerticalAlignment','bottom','HorizontalAlignment','center','FontSize',10);
% text(35,32,'vy3','Color','k','VerticalAlignment','bottom','HorizontalAlignment','center','FontSize',10);
% h5=quiver(35,0,0,20,'LineWidth',1,'MaxHeadSize',0.5,'color',[0 0 0]);
% 
% x=[22;28];y=[80;80];                       
% line(x,y,'Color','k');
% text(22,80,'-3','Color','k','VerticalAlignment','bottom','HorizontalAlignment','center','FontSize',10);
% text(25,80,'0','Color','k','VerticalAlignment','bottom','HorizontalAlignment','center','FontSize',10);
% text(28,80,'3','Color','k','VerticalAlignment','bottom','HorizontalAlignment','center','FontSize',10);
% text(25,92,'vx2','Color','k','VerticalAlignment','bottom','HorizontalAlignment','center','FontSize',10);
% h6=quiver(25,60,0,20,'LineWidth',1,'MaxHeadSize',0.5,'color',[0 0 0]);
% 
% x=[32;38];y=[80;80];                       
% line(x,y,'Color','k');
% text(32,80,'-3','Color','k','VerticalAlignment','bottom','HorizontalAlignment','center','FontSize',10);
% text(35,80,'0','Color','k','VerticalAlignment','bottom','HorizontalAlignment','center','FontSize',10);
% text(38,80,'3','Color','k','VerticalAlignment','bottom','HorizontalAlignment','center','FontSize',10);
% text(35,92,'vy2','Color','k','VerticalAlignment','bottom','HorizontalAlignment','center','FontSize',10);
% h7=quiver(35,60,0,20,'LineWidth',1,'MaxHeadSize',0.5,'color',[0 0 0]);
% 
% x=[42;48];y=[80;80];                       
% line(x,y,'Color','k');
% text(45,80,'vy2-vy3','Color','k','VerticalAlignment','bottom','HorizontalAlignment','center','FontSize',10);
% h8=text(45,60,'0','Color','k','VerticalAlignment','bottom','HorizontalAlignment','center','FontSize',10);

[m,n]=size(ToPlot);
for i=1:m
    % Plot puddle
    t=ToPlot(i,1);
    xCenter = CeX;
    yCenter = CeY;
    theta = 0 : 0.01 : 2*pi;
    radius = CeR+t/100;
    x = radius * cos(theta) + xCenter;
    y = radius * sin(theta) + yCenter;
    color=[220/255 220/255 220/255];
    circles = plot(x, y);
    fill(x, y, color); 
    clear t xCenter yCenter theta radius color circles;
    % Plot obstacle
    h1=rectangle('Position',[ToPlot(i,7) ToPlot(i,9) ToPlot(i,8)-ToPlot(i,7) ToPlot(i,10)-ToPlot(i,9)]);     
    % Plot second agent position (in time)
    h3=scatter(ToPlotAgent2(i,2),ToPlotAgent2(i,3),'MarkerFaceColor','b');
    h2=scatter(ToPlot(i,2),ToPlot(i,3),'MarkerFaceColor','r');1
    F(i)=getframe(gcf);
    writeVideo(v,F(i));
    if i~=m
       delete(h1);
       delete(h2);
       delete(h3);
    end
%     delete(h4);delete(h5);delete(h6);delete(h7);delete(h8);
%     h4=quiver(25+ToPlot(i,11),0,0,20,'LineWidth',1,'MaxHeadSize',0.5,'color',[1 0 0]);
%     h5=quiver(35+ToPlot(i,12),0,0,20,'LineWidth',1,'MaxHeadSize',0.5,'color',[1 0 0]);
%     h6=quiver(25+ToPlotAgent2(i,11),60,0,20,'LineWidth',1,'MaxHeadSize',0.5,'color',[0 0 1]);
%     h7=quiver(35+ToPlotAgent2(i,12),60,0,20,'LineWidth',1,'MaxHeadSize',0.5,'color',[0 0 1]);
%     if ~isnan(ToPlotAgent2(i,12)-ToPlot(i,12))
%        h8=text(45,60,num2str(ToPlotAgent2(i,12)-ToPlot(i,12),3),'Color','k','VerticalAlignment','bottom','HorizontalAlignment','center','FontSize',10);
%     else
%        h8=text(45,60,'0','Color','k','VerticalAlignment','bottom','HorizontalAlignment','center','FontSize',10); 
%     end
end
close(v);

ToPlotAgent3=ToPlot;
clear m n i h1 h2 ToPlot x y v;

% Dummy 21th agent
FinalTime=TimePerAgent{1,1}(end);
steps=linspace(0,FinalTime,InterpPoints);

TempTime=TimePerAgent{21,1}(:);
TempX=XPerAgent{21,1}(:);
TempY=YPerAgent{21,1}(:);
TempVX=VXPerAgent{21,1}(:);
TempVT=VYPerAgent{21,1}(:);
[m,n]=size(steps);
for i=1:n
    ToPlot(i,1)=steps(i);
    ToPlot(i,2)=interp1(TempTime,TempX,steps(i));
    ToPlot(i,3)=interp1(TempTime,TempY,steps(i));
    ToPlot(i,11)=interp1(TempTime,TempVX,steps(i));
    ToPlot(i,12)=interp1(TempTime,TempVT,steps(i));
end
clear TempTime TempTime2 TempX TempY TempX2 TempY2 m n i FinalTime steps;

ToPlot(:,4)=ObsIniX+(ToPlot(:,1)/Divider)*SpeedX;
ToPlot(:,5)=ObsIniY+(ToPlot(:,1)/Divider)*SpeedY; 
ToPlot(:,6)=ObsIniR+(ToPlot(:,1)/Divider)*SpeedR; 
ToPlot(:,7)=ToPlot(:,4)-ToPlot(:,6);
ToPlot(:,8)=ToPlot(:,4)+ToPlot(:,6);  
ToPlot(:,9)=ToPlot(:,5)-ToPlot(:,6);
ToPlot(:,10)=ToPlot(:,5)+ToPlot(:,6);

[m,n]=size(ToPlot);
for i=1:m
    if ~isnan(ToPlot(i,2))
        LastX=ToPlot(i,2);
    else
        ToPlot(i,2)=LastX;
    end
    if ~isnan(ToPlot(i,3))
        LastY=ToPlot(i,3);
    else
        ToPlot(i,3)=LastY;
    end
end
ToPlotAgent21=ToPlot;
clear m n i h1 h2 ToPlot x y v;

% Last agent
FigureLastAgent=figure('Name','Twentysecond vs twentyfirst agent');title('22nd vs 21st agent');
PlotXLow=-50;PlotXUpp=50;
PlotYLow=0;PlotYUpp=355;
axis([PlotXLow PlotXUpp PlotYLow PlotYUpp]);
xlabel('x');ylabel('y');
hold on;
x=[2;10];y=[300;300];                       
line(x,y,'Color','k');
rectangle('Position',[2 300 8 50]);
plot(XPerAgent{21,1},YPerAgent{21,1},'-.k');
plot(XPerAgent{22,1},YPerAgent{22,1},'--k');
lgd=legend('Goal area','\color{blue} Agent 21','\color{red} Agent 22','Location','northeast');
lgd.FontSize=12;
lgd.AutoUpdate='off';
text(5,0,'S','Color','k','VerticalAlignment','bottom','FontSize',13);
scatter(5,0,'MarkerFaceColor','k');

FinalTime=TimePerAgent{1,1}(end);
steps=linspace(0,FinalTime,InterpPoints);

TempTime=TimePerAgent{22,1}(:);
TempX=XPerAgent{22,1}(:);
TempY=YPerAgent{22,1}(:);
TempVX=VXPerAgent{22,1}(:);
TempVT=VYPerAgent{22,1}(:);
[m,n]=size(steps);
for i=1:n
    ToPlot(i,1)=steps(i);
    ToPlot(i,2)=interp1(TempTime,TempX,steps(i));
    ToPlot(i,3)=interp1(TempTime,TempY,steps(i));
    ToPlot(i,11)=interp1(TempTime,TempVX,steps(i));
    ToPlot(i,12)=interp1(TempTime,TempVT,steps(i));
end
clear TempTime TempTime2 TempX TempY TempX2 TempY2 m n i FinalTime steps;

ToPlot(:,4)=ObsIniX+(ToPlot(:,1)/Divider)*SpeedX;
ToPlot(:,5)=ObsIniY+(ToPlot(:,1)/Divider)*SpeedY; 
ToPlot(:,6)=ObsIniR+(ToPlot(:,1)/Divider)*SpeedR; 
ToPlot(:,7)=ToPlot(:,4)-ToPlot(:,6);
ToPlot(:,8)=ToPlot(:,4)+ToPlot(:,6);  
ToPlot(:,9)=ToPlot(:,5)-ToPlot(:,6);
ToPlot(:,10)=ToPlot(:,5)+ToPlot(:,6);

[m,n]=size(ToPlot);
for i=1:m
    if ~isnan(ToPlot(i,2))
        LastX=ToPlot(i,2);
    else
        ToPlot(i,2)=LastX;
    end
    if ~isnan(ToPlot(i,3))
        LastY=ToPlot(i,3);
    else
        ToPlot(i,3)=LastY;
    end
end

v=VideoWriter('LastAgent.avi');
v.FrameRate=20;
open(v);

% x=[22;28];y=[20;20];                       
% line(x,y,'Color','k');
% text(22,20,'-3','Color','k','VerticalAlignment','bottom','HorizontalAlignment','center','FontSize',10);
% text(25,20,'0','Color','k','VerticalAlignment','bottom','HorizontalAlignment','center','FontSize',10);
% text(28,20,'3','Color','k','VerticalAlignment','bottom','HorizontalAlignment','center','FontSize',10);
% text(25,32,'vx20','Color','k','VerticalAlignment','bottom','HorizontalAlignment','center','FontSize',10);
% h4=quiver(25,0,0,20,'LineWidth',1,'MaxHeadSize',0.5,'color',[0 0 0]);
% 
% x=[32;38];y=[20;20];                       
% line(x,y,'Color','k');
% text(32,20,'-3','Color','k','VerticalAlignment','bottom','HorizontalAlignment','center','FontSize',10);
% text(35,20,'0','Color','k','VerticalAlignment','bottom','HorizontalAlignment','center','FontSize',10);
% text(38,20,'3','Color','k','VerticalAlignment','bottom','HorizontalAlignment','center','FontSize',10);
% text(35,32,'vy20','Color','k','VerticalAlignment','bottom','HorizontalAlignment','center','FontSize',10);
% h5=quiver(35,0,0,20,'LineWidth',1,'MaxHeadSize',0.5,'color',[0 0 0]);
% 
% x=[22;28];y=[80;80];                       
% line(x,y,'Color','k');
% text(22,80,'-3','Color','k','VerticalAlignment','bottom','HorizontalAlignment','center','FontSize',10);
% text(25,80,'0','Color','k','VerticalAlignment','bottom','HorizontalAlignment','center','FontSize',10);
% text(28,80,'3','Color','k','VerticalAlignment','bottom','HorizontalAlignment','center','FontSize',10);
% text(25,92,'vx19','Color','k','VerticalAlignment','bottom','HorizontalAlignment','center','FontSize',10);
% h6=quiver(25,60,0,20,'LineWidth',1,'MaxHeadSize',0.5,'color',[0 0 0]);
% 
% x=[32;38];y=[80;80];                       
% line(x,y,'Color','k');
% text(32,80,'-3','Color','k','VerticalAlignment','bottom','HorizontalAlignment','center','FontSize',10);
% text(35,80,'0','Color','k','VerticalAlignment','bottom','HorizontalAlignment','center','FontSize',10);
% text(38,80,'3','Color','k','VerticalAlignment','bottom','HorizontalAlignment','center','FontSize',10);
% text(35,92,'v19','Color','k','VerticalAlignment','bottom','HorizontalAlignment','center','FontSize',10);
% h7=quiver(35,60,0,20,'LineWidth',1,'MaxHeadSize',0.5,'color',[0 0 0]);
% 
% x=[42;48];y=[80;80];                       
% line(x,y,'Color','k');
% text(45,92,'vy19-','Color','k','VerticalAlignment','bottom','HorizontalAlignment','center','FontSize',10);
% text(45,80,'vy20','Color','k','VerticalAlignment','bottom','HorizontalAlignment','center','FontSize',10);
% h8=text(45,60,'0','Color','k','VerticalAlignment','bottom','HorizontalAlignment','center','FontSize',10);

[m,n]=size(ToPlot);
for i=1:m
    % Plot puddle
    t=ToPlot(i,1);
    xCenter = CeX;
    yCenter = CeY;
    theta = 0 : 0.01 : 2*pi;
    radius = CeR+t/100;
    x = radius * cos(theta) + xCenter;
    y = radius * sin(theta) + yCenter;
    color=[220/255 220/255 220/255];
    circles = plot(x, y);
    fill(x, y, color); 
    clear t xCenter yCenter theta radius color circles;
    % Plot obstacle
    h1=rectangle('Position',[ToPlot(i,7) ToPlot(i,9) ToPlot(i,8)-ToPlot(i,7) ToPlot(i,10)-ToPlot(i,9)]);     
    % Plot second agent position (in time)
    h3=scatter(ToPlotAgent21(i,2),ToPlotAgent21(i,3),'MarkerFaceColor','b');    
    h2=scatter(ToPlot(i,2),ToPlot(i,3),'MarkerFaceColor','r');
    F(i)=getframe(gcf);
    writeVideo(v,F(i));
    if i~=m
       delete(h1);
       delete(h2);
       delete(h3);
    end
%     delete(h4);delete(h5);delete(h6);delete(h7);delete(h8);
%     h4=quiver(25+ToPlot(i,11),0,0,20,'LineWidth',1,'MaxHeadSize',0.5,'color',[1 0 0]);
%     h5=quiver(35+ToPlot(i,12),0,0,20,'LineWidth',1,'MaxHeadSize',0.5,'color',[1 0 0]);
%     h6=quiver(25+ToPlotAgent19(i,11),60,0,20,'LineWidth',1,'MaxHeadSize',0.5,'color',[0 0 1]);
%     h7=quiver(35+ToPlotAgent19(i,12),60,0,20,'LineWidth',1,'MaxHeadSize',0.5,'color',[0 0 1]);
%     if ~isnan(ToPlotAgent19(i,12)-ToPlot(i,12))
%        h8=text(45,60,num2str(ToPlotAgent2(i,12)-ToPlot(i,12),3),'Color','k','VerticalAlignment','bottom','HorizontalAlignment','center','FontSize',10);
%     else
%        h8=text(45,60,'0','Color','k','VerticalAlignment','bottom','HorizontalAlignment','center','FontSize',10); 
%     end
end
close(v);


% ------------------------------------------------------------------------------------------------------------------------------------------------------------------
% End of : Videos
% ------------------------------------------------------------------------------------------------------------------------------------------------------------------
% End of file