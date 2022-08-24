clear CollisionCheckerTable;

[m,n]=size(TotalResults);
for i=1:m
    AgentsTrajectoryT=[ ];
    AgentsTrajectoryX=[ ];
    AgentsTrajectoryY=[ ];
    for j=1:n
        if ~isempty(TotalResults{i,j})
            AgentsTrajectoryT=[AgentsTrajectoryT;TotalResults{i,j}(:,1)];
            AgentsTrajectoryX=[AgentsTrajectoryX;TotalResults{i,j}(:,2)];
            AgentsTrajectoryY=[AgentsTrajectoryY;TotalResults{i,j}(:,3)];
        end
    end

    ObstacleChecker=[ ];
    [v,b]=size(AgentsTrajectoryT);
    for r=1:v
        ObstacleChecker(r,1)=AgentsTrajectoryT(r,1);
        ObstacleChecker(r,2)=ObsIniX+(AgentsTrajectoryT(r,1)/Divider)*SpeedX;     % Obstacle center x axis
        ObstacleChecker(r,3)=ObsIniY+(AgentsTrajectoryT(r,1)/Divider)*SpeedY;     % Obstacle center y axis
        ObstacleChecker(r,4)=ObsIniR+(AgentsTrajectoryT(r,1)/Divider)*SpeedR;     % Obstacle dynamic radius
    end
    ObstacleChecker(:,5)=ObstacleChecker(:,2)-(ObstacleChecker(:,4));   % Detection area around each leader's position is a rectangle of LeaDecRad radius so we
    ObstacleChecker(:,6)=ObstacleChecker(:,2)+(ObstacleChecker(:,4));   % take the four courners and produce a LeaDecRad*LeaDecRad bounding box
    ObstacleChecker(:,7)=ObstacleChecker(:,3)-(ObstacleChecker(:,4));
    ObstacleChecker(:,8)=ObstacleChecker(:,3)+(ObstacleChecker(:,4));

    k=1;
    for r=1:v
        xmin=ObstacleChecker(r,5);
        xmax=ObstacleChecker(r,6);
        ymin=ObstacleChecker(r,7);
        ymax=ObstacleChecker(r,8);
        Obspolyshape=polyshape([xmin xmin xmax xmax],[ymin ymax ymax ymin]);
        xq=AgentsTrajectoryX(r,1);
        yq=AgentsTrajectoryY(r,1);
        xv=Obspolyshape.Vertices(:,1);
        yv=Obspolyshape.Vertices(:,2);
        [in,on] = inpolygon(xq,yq,xv,yv);
        if in>0 || on>0
           CollisionCheckerTable{i,1}(1,k)=1;
        else
           CollisionCheckerTable{i,1}(1,k)=0;
        end
        k=k+1;
    end
    clear sum;
    CollisionCheckerTable{i,2}=sum(CollisionCheckerTable{i,1});
    clear ObstacleChecker v b r xmin xmax ymin ymax Obspolyshape in on AgentsTrajectoryT AgentsTrajectoryX AgentsTrajectoryY k;
end

[m,n]=size(CollisionCheckerTable);
FinalCheck=0;
for i=1:m
    FinalCheck=FinalCheck+CollisionCheckerTable{i,2};
end

if FinalCheck==0
   disp('No collisions found');
else
   disp('Collisions found');
end