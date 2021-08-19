% Joe Perrella 03/13/2020

clc, clear
%Clear all the figures
FigList = findall(groot, 'Type', 'figure');
for iFig = 1:numel(FigList)
    try
        clf(FigList(iFig));
    catch
        % Nothing to do
    end
end
tic()

delta = .5;

%%Global obstacles and configuration space
obstacles_raw = csvread('H3_obstacles.txt');
xobs = obstacles_raw(:,1);
yobs = obstacles_raw(:,2);
robs = obstacles_raw(:,3);
th = 0:pi/1000:2*pi;
xunitobs = zeros(1,10);
yunitobs = zeros(1,10);
numObs = size(obstacles_raw,1);
for i = 1:numObs
    for j = 1:length(th)
        xunitobs(i,j) = robs(i)*cos(th(j)) + xobs(i);
        yunitobs(i,j) = robs(i)*sin(th(j)) + yobs(i);
    end
end

%Robot
robot_raw = csvread('H3_robot.txt');
xrob = robot_raw(:,1);
yrob = robot_raw(:,2);
%IF YOU SORT THE ROBOT POINTS BY DISTANCE AWAY FROM CENTER, IT WILL REDUCE
%THE POINT-WISE COMPUTATION FOR GOAL AND OBSTACLE CHECKING.
%SHOULD BE FURTHEST AWAY POINTS FIRST, AND CLOSEST POINTS LAST

for i = 1:37
    robot_raw(i,3) = sqrt((xrob(i)-xrob(1))^2 + (yrob(i)-yrob(1))^2);
end
%sort


%Collision checking space is x y from 0 to 100
C = [0 100 0 100];

%min and Max angular and linear accelerations
aMin = -2;
aMax = 2;
gMin = -pi/2;
gMax = pi/2;


%% Problem 1
disp('--1--')
epsilon = 5
%Define the system State Variable at start:
start = [80, 60, 0, pi, 0, 0, 0, 0]
%edge = [x1  y1 t1  th  v  w  a  g]

goal = [0 0 20]


xGoal = goal(3)*cos(th(:)) + goal(1);
yGoal = goal(3)*sin(th(:)) + goal(2);
goalFlag = false;
%put the robot at the start location first.
xrobStart = xrob+start(1);
yrobStart = yrob+start(2);
%need to rotate all of the points about the center point, by start(4)
for i = 2:37
    hyp = sqrt((xrobStart(i)-xrobStart(1))^2 + (yrobStart(i)-yrobStart(1))^2);
    prevAngle = atan2((yrobStart(i)-yrobStart(1)),(xrobStart(i)-xrobStart(1)));
    newAngle = prevAngle+start(4);
    xrobStart(i) = xrobStart(1)+hyp*cos(newAngle);
    yrobStart(i) = yrobStart(1)+hyp*sin(newAngle);
end



% Initialize the plot
figure(1), hold on
h = legend('show','location','best');
%Plot goal region
plot(xGoal,yGoal,'g','DisplayName','Goal');
%Plot initial position of the robot
plot(xrobStart,yrobStart,'.b','HandleVisibility','off');
axis(C);
xlabel('x');
ylabel('y');
grid on;
axis square;
%Plot the obstacles
for i = 1:numObs
    plot(xunitobs(i,:), yunitobs(i,:),'r','HandleVisibility','off');
end
plot(0,0,'r','DisplayName','Obstacle');


%Edges contain from & to nodes, times, as well as theta, linear velocity V,
%and angular velocity W, linear acceleration a, and angular acceleration g.
edges = [100000 100000 100000 100000 100000 100000 100000 100000 100000 100000 100000];
%edge = [  x1     y1     t1     x2     y2     t2   theta     v      w      a      g  ]


while goalFlag == false
    %Take a C-Space sample
    sampleTheta = -2*pi + 4*pi*rand; %-2pi to 2pi
    sampleVel = -5 + 10*rand; %-5 to 5
    sampleAng = -pi/2 + pi*rand; %-pi/2 to pi/2
    sample = [100*rand 100*rand sampleTheta sampleVel sampleAng];
    %plot(sample(1),sample(2),'*k','HandleVisibility','off')
    
    %reset obstacle flag
    obsFlag = false;
    
    if size(edges,1)==1
        closestNode = start;
    else
        dist = zeros(size(edges,1),2);
        %initialize minimum distance artificially high so that it must be overwritten
        minNeighborDist = 10000;
        
        %loop through all the edges.
        for i = 1:size(edges,1)
            
            %grab nodes from this edge.
            node1 = edges(i,1:3);
            node2 = edges(i,4:6);
            
            %grab theta of these nodes and find difference between it and
            %the sample
            nodeTheta = edges(i,7);
            dTheta = abs(sample(3)-nodeTheta);

            
            %define line containing the edges end nodes and its normal
            line = [(node2(1)-node1(1)),(node2(2)-node1(2))];
            lineMag = sqrt(line(1)^2 +line(2)^2);
            n = line/lineMag;
            
            %%finds closest node on the line containing node1 and node2
            node = sample(1:2) + (node1(1:2)-sample(1:2))-(dot((node1(1:2)-sample(1:2)), n))*n;
            %IF THE CLOSEST NODE IS ON THE LINE, AVERAGE THE TIME BETWEEN
            %THE TWO
            node(3) = (node1(3)+node2(3))/2;
            
            dotProduct = (sample(1)-node1(1))*(node2(1)-node1(1))+(sample(2)-node1(2))*(node2(2)-node1(2));
            squaredLength = (node2(1)-node1(1))*(node2(1)-node1(1))+(node2(2)-node1(2))*(node2(2)-node1(2));
            %checking if the closest node on the line is between two points
            if (dotProduct<0 || squaredLength<dotProduct)
                %if not between, check which node is closer (factoring in
                %rotation)
                node1Dist = sqrt((node1(1)-sample(1)).^2 + (node1(2)-sample(2)).^2)+dTheta;
                node2Dist = sqrt((node2(1)-sample(1)).^2 + (node2(2)-sample(2)).^2)+dTheta;
                %if node1 closer node2 and less than the current minimum,
                %set to closest. check opposite as well. otherwise toss.
                if (node1Dist< node2Dist && node1Dist<minNeighborDist)
                    minNeighborDist = node1Dist;
                    nodeFlag = true;
                    closestNode = node1;
                    closestNode = cat(2, closestNode,edges(i,7:11));
                elseif (node2Dist< node1Dist && node2Dist<minNeighborDist)
                    minNeighborDist = node2Dist;
                    nodeFlag = true;
                    closestNode = node2;
                    closestNode = cat(2, closestNode,edges(i,7:11));
                end
            else
                %if it is between, check vs. the shortest length
                nodeDist = sqrt((node(1)-sample(1)).^2 + (node(2)-sample(2)).^2)+dTheta;
                %if node less than the current minimum, set to min and
                %closest node. repeats until all edges have been
                %traversed.
                if nodeDist<minNeighborDist
                    nodeFlag = false;
                    minNeighborDist = nodeDist;
                    closestNode = node;
                    closestNode = cat(2, closestNode,edges(i,7:11));
                end
            end
        end
    end
    
    %now that closestNode to sample on tree has been found
    %compute vector from closestNode to sample
    
    vec = [sample(1)-closestNode(1),sample(2)-closestNode(2)];
    vecMag = sqrt(vec(1)^2 + vec(2)^2);
    
    %Only travel the distance to the sample if it is closer than epsilon
    if vecMag < epsilon
        epsilon = vecMag;
    end
    
    %compute change in linear velocity and change in angular velocity to
    %get to from the closestNode to Sample
    dV = sample(4) - closestNode(5); %vf - vi
    dW = sample(5) - closestNode(6); %wf - wi
    
    %compute the average speed while traveling this new edge
    avgV = (2*closestNode(5) + dV)/2;
    %compute the time to travel the distance epsilon
    time = abs(epsilon/avgV);
    
    %Compute the acceleration (linear and angular: constants), and set
    %within limits if they exceed.
    a = dV/time;
    if a < aMin
        a = aMin;
    elseif a > aMax
        a = aMax;
    end
    
    g = dW/time;
    if g < gMin
        g = gMin;
    elseif g > gMax
        g = gMax;
    end
    
    %Initialize the start of the trajectory
    traj = closestNode;
    %plot(traj(1),traj(2),'.g','HandleVisibility','off');
    
    %Loop through delta time chunks of the trajectory until total time met
    for j = 0:delta:time
        %This if statement makes the robot stop as soon as it has reached
        %the edge of the goal
        if goalFlag == false
            
            %get most recent trajectory point
            trajLength = size(traj,1);
            if size(traj,1)~=1
                trajInput = traj(trajLength,:);
            else
                trajInput = traj;
            end
            
            %Use forward Euler method to solve the initial value problem:
            %Input either the trajectory from the existing graph, so that
            %it is continuous, or from the most recent delta chunk along
            %the new trajectory
            newTraj = RoboSim(trajInput,a,g,delta);
            
            newTraj(7) = a;
            newTraj(8) = g;
            %Add new trajectory chunk to the path
            traj = cat(1, traj, newTraj);
            
            
            %translate the rest of the robot points to the delta (which is new center) location
            xrobDel = xrob+traj(trajLength+1,1);
            yrobDel = yrob+traj(trajLength+1,2);
            
            
            %rotate all of the points about the center point
            for i = 2:37
                %distance from center to that point
                hyp = sqrt((xrobDel(i)-xrobDel(1))^2 + (yrobDel(i)-yrobDel(1))^2);
                %old angle between center and that point
                prevAngle = atan2((yrobDel(i)-yrobDel(1)),(xrobDel(i)-xrobDel(1)));
                %new angle based on rotation of trajector
                newAngle = prevAngle+newTraj(4);
                %compute new robot point locations
                xrobDel(i) = xrobDel(1)+hyp*cos(newAngle);
                yrobDel(i) = yrobDel(1)+hyp*sin(newAngle);
            end
            
            %collision check all obstacles, unless collision already
            %detected
            for i = 1:numObs
                if obsFlag==false
                    %define the current obstacle we are checking against
                    currObs = obstacles_raw(i,:);
                    distSquare = (xrobDel(1) - currObs(1)).^2 + (yrobDel(1) - currObs(2)).^2;
                    %Add bubble to the center robot point, radius 1
                    rSquare = (currObs(3)+1)^2;
                    if (distSquare < rSquare)
                        %If obstacle collides with robot bubble, check each
                        %robot point
                        for j = 1:length(xrob)
                            %distance between
                            distSquare = (xrobDel(j) - currObs(1)).^2 + (yrobDel(j) - currObs(2)).^2;
                            rSquare = currObs(3)^2;
                            
                            %robot point is within an obstacle
                            if (distSquare < rSquare)
                                %collision has occurred
                                obsFlag = true;
                            end
                        end
                        
                        %If there is no collision with an obstacle, check
                        %that the robot center is within the R^2 space
                    elseif (xrobDel(1)>100)
                        obsFlag = true;
                    elseif (xrobDel(1)<0)
                        obsFlag = true;
                    elseif (yrobDel(1)>100)
                        obsFlag = true;
                    elseif (yrobDel(1)<0)
                        obsFlag = true;
                    end
                end
            end
            
            %%goal check
            goalDistSquare = (xrobDel(1) - goal(1))^2 + (yrobDel(1) - goal(2))^2;
            rGoalSquare = (goal(3)+1)^2;
            if (goalDistSquare < rGoalSquare && obsFlag == false)
                %If inside the bubble, then do the same check, but loop through all the
                %points that make up the robot
                
                rGoalSquare = goal(3)^2;
                for i = 1:length(xrob)
                    %Stop if even 1 point is inside goal
                    if goalFlag ==false
                        %xrobDel and yrobDel contain the values of the points relative to the
                        %center, which is xrobDel(1) yrobDel(1)
                        goalDistSquare = (xrobDel(i) - goal(1)).^2 + (yrobDel(i) - goal(2)).^2;
                        
                        %if the point is within the goal region, and we havent hit an
                        %obstacle
                        if (goalDistSquare < rGoalSquare && obsFlag == false)
                            goalNode = newTraj
                            goalFlag = true;
                            
                            %%create new edge, which contains x and y of the closestNode
                            %%from the graph, and the trajectory point where goal is
                            %%reached
                            goalEdge(1,1:3) =  closestNode(1:3);
                            goalEdge(1,4:6) =  newTraj(1:3);
                            goalEdge(1,7:11) =  closestNode(4:8);
                            %Add this to the total edges list
                            edges = cat(1,edges,goalEdge);
                        end
                    end
                end
            end
        end
        %loop through and check again along the path until epsilon is met
    end
    
    %if not obstacle, add edges and plot
    if obsFlag == false
        
        trajLength = size(traj,1);
        %loop through all the sub trajectories and add them as edges
        for i = 2:trajLength
            
            newEdge(1,1:3) =  traj(i-1,1:3);
            newEdge(1,4:6) =  traj(i,1:3);
            newEdge(1,7:11) =  traj(i-1,4:8);
            
            %path = cat(2,path,traj(i,:))
            
            plot([newEdge(1) newEdge(4)],[newEdge(2) newEdge(5)],'k','HandleVisibility','off');
            edges = cat(1,edges,newEdge);
            
        end
    else
    end
    
    %reset epsilon if it had been shortened due to a close sample
    epsilon = 5;
    
    
end
path = [];
pathLength = size(edges,1);
path(:,1) = edges(2:pathLength,3);
path(:,2) = edges(2:pathLength,1);
path(:,3) = edges(2:pathLength,2);
path(:,4:8) = edges(2:pathLength,7:11);

csvwrite('HW3_P1.txt', path);

plot(xrobDel,yrobDel,'.g','HandleVisibility','off');
%Plot initial position of the robot
plot(xrobStart,yrobStart,'.b','HandleVisibility','off');

plot(0,0,'k','DisplayName','Tree');
legend('Location','eastoutside')
legend('boxoff')
title('R^2')
hold off;

toc()


%% Problem 2
disp('--2--')
epsilon = 10
%Define the system State Variable at start:
start = [5, 60, 0, 0, 0, 0, 0, 0]
%edge = [x1  y1 t1  th  v  w  a  g]

goal = [100 60 20]


xGoal = goal(3)*cos(th(:)) + goal(1);
yGoal = goal(3)*sin(th(:)) + goal(2);
goalFlag = false;
%put the robot at the start location first.
xrobStart = xrob+start(1);
yrobStart = yrob+start(2);
%need to rotate all of the points about the center point, by start(4)
for i = 2:37
    hyp = sqrt((xrobStart(i)-xrobStart(1))^2 + (yrobStart(i)-yrobStart(1))^2);
    prevAngle = atan2((yrobStart(i)-yrobStart(1)),(xrobStart(i)-xrobStart(1)));
    newAngle = prevAngle+start(4);
    xrobStart(i) = xrobStart(1)+hyp*cos(newAngle);
    yrobStart(i) = yrobStart(1)+hyp*sin(newAngle);
end



% Initialize the plot
figure(2), hold on
h = legend('show','location','best');
%Plot goal region
plot(xGoal,yGoal,'g','DisplayName','Goal');
%Plot initial position of the robot
plot(xrobStart,yrobStart,'.b','HandleVisibility','off');
axis(C);
xlabel('x');
ylabel('y');
grid on;
axis square;
%Plot the obstacles
for i = 1:numObs
    plot(xunitobs(i,:), yunitobs(i,:),'r','HandleVisibility','off');
end
plot(0,0,'r','DisplayName','Obstacle');


%Edges contain from & to nodes, times, as well as theta, linear velocity V,
%and angular velocity W, linear acceleration a, and angular acceleration g.
edges = [100000 100000 100000 100000 100000 100000 100000 100000 100000 100000 100000];
%edge = [  x1     y1     t1     x2     y2     t2   theta     v      w      a      g  ]


while goalFlag == false
    %Take a C-Space sample
    sampleTheta = -2*pi + 4*pi*rand; %-2pi to 2pi
    sampleVel = -5 + 10*rand; %-5 to 5
    sampleAng = -pi/2 + pi*rand; %-pi/2 to pi/2
    sample = [100*rand 100*rand sampleTheta sampleVel sampleAng];
    %plot(sample(1),sample(2),'*k','HandleVisibility','off')
    
    %reset obstacle flag
    obsFlag = false;
    
    if size(edges,1)==1
        closestNode = start;
    else
        dist = zeros(size(edges,1),2);
        %initialize minimum distance artificially high so that it must be overwritten
        minNeighborDist = 10000;
        
        %loop through all the edges.
        for i = 1:size(edges,1)
            
            %grab nodes from this edge.
            node1 = edges(i,1:3);
            node2 = edges(i,4:6);
            
            %grab theta of these nodes and find difference between it and
            %the sample
            nodeTheta = edges(i,7);
            dTheta = abs(sample(3)-nodeTheta);

            
            %define line containing the edges end nodes and its normal
            line = [(node2(1)-node1(1)),(node2(2)-node1(2))];
            lineMag = sqrt(line(1)^2 +line(2)^2);
            n = line/lineMag;
            
            %%finds closest node on the line containing node1 and node2
            node = sample(1:2) + (node1(1:2)-sample(1:2))-(dot((node1(1:2)-sample(1:2)), n))*n;
            %IF THE CLOSEST NODE IS ON THE LINE, AVERAGE THE TIME BETWEEN
            %THE TWO
            node(3) = (node1(3)+node2(3))/2;
            
            dotProduct = (sample(1)-node1(1))*(node2(1)-node1(1))+(sample(2)-node1(2))*(node2(2)-node1(2));
            squaredLength = (node2(1)-node1(1))*(node2(1)-node1(1))+(node2(2)-node1(2))*(node2(2)-node1(2));
            %checking if the closest node on the line is between two points
            if (dotProduct<0 || squaredLength<dotProduct)
                %if not between, check which node is closer (factoring in
                %rotation)
                node1Dist = sqrt((node1(1)-sample(1)).^2 + (node1(2)-sample(2)).^2)+dTheta;
                node2Dist = sqrt((node2(1)-sample(1)).^2 + (node2(2)-sample(2)).^2)+dTheta;
                %if node1 closer node2 and less than the current minimum,
                %set to closest. check opposite as well. otherwise toss.
                if (node1Dist< node2Dist && node1Dist<minNeighborDist)
                    minNeighborDist = node1Dist;
                    nodeFlag = true;
                    closestNode = node1;
                    closestNode = cat(2, closestNode,edges(i,7:11));
                elseif (node2Dist< node1Dist && node2Dist<minNeighborDist)
                    minNeighborDist = node2Dist;
                    nodeFlag = true;
                    closestNode = node2;
                    closestNode = cat(2, closestNode,edges(i,7:11));
                end
            else
                %if it is between, check vs. the shortest length
                nodeDist = sqrt((node(1)-sample(1)).^2 + (node(2)-sample(2)).^2)+dTheta;
                %if node less than the current minimum, set to min and
                %closest node. repeats until all edges have been
                %traversed.
                if nodeDist<minNeighborDist
                    nodeFlag = false;
                    minNeighborDist = nodeDist;
                    closestNode = node;
                    closestNode = cat(2, closestNode,edges(i,7:11));
                end
            end
        end
    end

    
    %now that closestNode to sample on tree has been found
    %compute vector from closestNode to sample
    
    vec = [sample(1)-closestNode(1),sample(2)-closestNode(2)];
    vecMag = sqrt(vec(1)^2 + vec(2)^2);
    
    %Only travel the distance to the sample if it is closer than epsilon
    if vecMag < epsilon
        epsilon = vecMag;
    end
    
    %compute change in linear velocity and change in angular velocity to
    %get to from the closestNode to Sample
    dV = sample(4) - closestNode(5); %vf - vi
    dW = sample(5) - closestNode(6); %wf - wi
    
    %compute the average speed while traveling this new edge
    avgV = (2*closestNode(5) + dV)/2;
    %compute the time to travel the distance epsilon
    time = abs(epsilon/avgV);
    
    %Compute the acceleration (linear and angular: constants), and set
    %within limits if they exceed.
    a = dV/time;
    if a < aMin
        a = aMin;
    elseif a > aMax
        a = aMax;
    end
    
    g = dW/time;
    if g < gMin
        g = gMin;
    elseif g > gMax
        g = gMax;
    end
    
    %Initialize the start of the trajectory
    traj = closestNode;
    %plot(traj(1),traj(2),'.g','HandleVisibility','off');
    
    %Loop through delta time chunks of the trajectory until total time met
    for j = 0:delta:time
        %This if statement makes the robot stop as soon as it has reached
        %the edge of the goal
        if goalFlag == false
            
            %get most recent trajectory point
            trajLength = size(traj,1);
            if size(traj,1)~=1
                trajInput = traj(trajLength,:);
            else
                trajInput = traj;
            end
            
            %Use forward Euler method to solve the initial value problem:
            %Input either the trajectory from the existing graph, so that
            %it is continuous, or from the most recent delta chunk along
            %the new trajectory
            newTraj = RoboSim(trajInput,a,g,delta);
            
            newTraj(7) = a;
            newTraj(8) = g;
            %Add new trajectory chunk to the path
            traj = cat(1, traj, newTraj);
            
            
            %translate the rest of the robot points to the delta (which is new center) location
            xrobDel = xrob+traj(trajLength+1,1);
            yrobDel = yrob+traj(trajLength+1,2);
            
            
            %rotate all of the points about the center point
            for i = 2:37
                %distance from center to that point
                hyp = sqrt((xrobDel(i)-xrobDel(1))^2 + (yrobDel(i)-yrobDel(1))^2);
                %old angle between center and that point
                prevAngle = atan2((yrobDel(i)-yrobDel(1)),(xrobDel(i)-xrobDel(1)));
                %new angle based on rotation of trajector
                newAngle = prevAngle+newTraj(4);
                %compute new robot point locations
                xrobDel(i) = xrobDel(1)+hyp*cos(newAngle);
                yrobDel(i) = yrobDel(1)+hyp*sin(newAngle);
            end
            
            %collision check all obstacles, unless collision already
            %detected
            for i = 1:numObs
                if obsFlag==false
                    %define the current obstacle we are checking against
                    currObs = obstacles_raw(i,:);
                    distSquare = (xrobDel(1) - currObs(1)).^2 + (yrobDel(1) - currObs(2)).^2;
                    %Add bubble to the center robot point, radius 1
                    rSquare = (currObs(3)+1)^2;
                    if (distSquare < rSquare)
                        %If obstacle collides with robot bubble, check each
                        %robot point
                        for j = 1:length(xrob)
                            %distance between
                            distSquare = (xrobDel(j) - currObs(1)).^2 + (yrobDel(j) - currObs(2)).^2;
                            rSquare = currObs(3)^2;
                            
                            %robot point is within an obstacle
                            if (distSquare < rSquare)
                                %collision has occurred
                                obsFlag = true;
                            end
                        end
                        
                        %If there is no collision with an obstacle, check
                        %that the robot center is within the R^2 space
                    elseif (xrobDel(1)>100)
                        obsFlag = true;
                    elseif (xrobDel(1)<0)
                        obsFlag = true;
                    elseif (yrobDel(1)>100)
                        obsFlag = true;
                    elseif (yrobDel(1)<0)
                        obsFlag = true;
                    end
                end
            end
            
            %%goal check
            goalDistSquare = (xrobDel(1) - goal(1))^2 + (yrobDel(1) - goal(2))^2;
            rGoalSquare = (goal(3)+1)^2;
            if (goalDistSquare < rGoalSquare && obsFlag == false)
                %If inside the bubble, then do the same check, but loop through all the
                %points that make up the robot
                
                rGoalSquare = goal(3)^2;
                for i = 1:length(xrob)
                    %Stop if even 1 point is inside goal
                    if goalFlag ==false
                        %xrobDel and yrobDel contain the values of the points relative to the
                        %center, which is xrobDel(1) yrobDel(1)
                        goalDistSquare = (xrobDel(i) - goal(1)).^2 + (yrobDel(i) - goal(2)).^2;
                        
                        %if the point is within the goal region, and we havent hit an
                        %obstacle
                        if (goalDistSquare < rGoalSquare && obsFlag == false)
                            goalNode = newTraj
                            goalFlag = true;
                            
                            %%create new edge, which contains x and y of the closestNode
                            %%from the graph, and the trajectory point where goal is
                            %%reached
                            goalEdge(1,1:3) =  closestNode(1:3);
                            goalEdge(1,4:6) =  newTraj(1:3);
                            goalEdge(1,7:11) =  closestNode(4:8);
                            %Add this to the total edges list
                            edges = cat(1,edges,goalEdge);
                        end
                    end
                end
            end
        end
        %loop through and check again along the path until epsilon is met
    end
    
    %if not obstacle, add edges and plot
    if obsFlag == false
        
        trajLength = size(traj,1);
        %loop through all the sub trajectories and add them as edges
        for i = 2:trajLength
            
            newEdge(1,1:3) =  traj(i-1,1:3);
            newEdge(1,4:6) =  traj(i,1:3);
            newEdge(1,7:11) =  traj(i-1,4:8);
            
            %path = cat(2,path,traj(i,:))
            
            plot([newEdge(1) newEdge(4)],[newEdge(2) newEdge(5)],'k','HandleVisibility','off');
            edges = cat(1,edges,newEdge);
            
        end
    else
    end
    
    %reset epsilon if it had been shortened due to a close sample
    epsilon = 10;
    
    
end
path = [];
pathLength = size(edges,1);
path(:,1) = edges(2:pathLength,3);
path(:,2) = edges(2:pathLength,1);
path(:,3) = edges(2:pathLength,2);
path(:,4:8) = edges(2:pathLength,7:11);

csvwrite('HW3_P2.txt', path);


plot(xrobDel,yrobDel,'.g','HandleVisibility','off');
%Plot initial position of the robot
plot(xrobStart,yrobStart,'.b','HandleVisibility','off');
plot(0,0,'k','DisplayName','Tree');
legend('Location','eastoutside')
legend('boxoff')
title('R^2')
hold off;

toc()


%% Problem 3
disp('--3--')
epsilon = 5
%Define the system State Variable at start:
start = [10, 90, 0, 3*pi/2, 0, 0, 0, 0]
%edge = [x1  y1 t1  th  v  w  a  g]

goal = [100 90 10]


xGoal = goal(3)*cos(th(:)) + goal(1);
yGoal = goal(3)*sin(th(:)) + goal(2);
goalFlag = false;
%put the robot at the start location first.
xrobStart = xrob+start(1);
yrobStart = yrob+start(2);
%need to rotate all of the points about the center point, by start(4)
for i = 2:37
    hyp = sqrt((xrobStart(i)-xrobStart(1))^2 + (yrobStart(i)-yrobStart(1))^2);
    prevAngle = atan2((yrobStart(i)-yrobStart(1)),(xrobStart(i)-xrobStart(1)));
    newAngle = prevAngle+start(4);
    xrobStart(i) = xrobStart(1)+hyp*cos(newAngle);
    yrobStart(i) = yrobStart(1)+hyp*sin(newAngle);
end



% Initialize the plot
figure(3), hold on
h = legend('show','location','best');
%Plot goal region
plot(xGoal,yGoal,'g','DisplayName','Goal');
%Plot initial position of the robot
plot(xrobStart,yrobStart,'.b','HandleVisibility','off');
axis(C);
xlabel('x');
ylabel('y');
grid on;
axis square;
%Plot the obstacles
for i = 1:numObs
    plot(xunitobs(i,:), yunitobs(i,:),'r','HandleVisibility','off');
end
plot(0,0,'r','DisplayName','Obstacle');


%Edges contain from & to nodes, times, as well as theta, linear velocity V,
%and angular velocity W, linear acceleration a, and angular acceleration g.
edges = [100000 100000 100000 100000 100000 100000 100000 100000 100000 100000 100000];
%edge = [  x1     y1     t1     x2     y2     t2   theta     v      w      a      g  ]


while goalFlag == false
    %Take a C-Space sample
    sampleTheta = -2*pi + 4*pi*rand; %-2pi to 2pi
    sampleVel = -5 + 10*rand; %-5 to 5
    sampleAng = -pi/2 + pi*rand; %-pi/2 to pi/2
    sample = [100*rand 100*rand sampleTheta sampleVel sampleAng];
    %plot(sample(1),sample(2),'*k','HandleVisibility','off')
    
    %reset obstacle flag
    obsFlag = false;
    
    if size(edges,1)==1
        closestNode = start;
    else
        dist = zeros(size(edges,1),2);
        %initialize minimum distance artificially high so that it must be overwritten
        minNeighborDist = 10000;
        
        %loop through all the edges.
        for i = 1:size(edges,1)
            
            %grab nodes from this edge.
            node1 = edges(i,1:3);
            node2 = edges(i,4:6);
            
            %grab theta of these nodes and find difference between it and
            %the sample
            nodeTheta = edges(i,7);
            dTheta = abs(sample(3)-nodeTheta);

            
            %define line containing the edges end nodes and its normal
            line = [(node2(1)-node1(1)),(node2(2)-node1(2))];
            lineMag = sqrt(line(1)^2 +line(2)^2);
            n = line/lineMag;
            
            %%finds closest node on the line containing node1 and node2
            node = sample(1:2) + (node1(1:2)-sample(1:2))-(dot((node1(1:2)-sample(1:2)), n))*n;
            %IF THE CLOSEST NODE IS ON THE LINE, AVERAGE THE TIME BETWEEN
            %THE TWO
            node(3) = (node1(3)+node2(3))/2;
            
            dotProduct = (sample(1)-node1(1))*(node2(1)-node1(1))+(sample(2)-node1(2))*(node2(2)-node1(2));
            squaredLength = (node2(1)-node1(1))*(node2(1)-node1(1))+(node2(2)-node1(2))*(node2(2)-node1(2));
            %checking if the closest node on the line is between two points
            if (dotProduct<0 || squaredLength<dotProduct)
                %if not between, check which node is closer (factoring in
                %rotation)
                node1Dist = sqrt((node1(1)-sample(1)).^2 + (node1(2)-sample(2)).^2)+dTheta;
                node2Dist = sqrt((node2(1)-sample(1)).^2 + (node2(2)-sample(2)).^2)+dTheta;
                %if node1 closer node2 and less than the current minimum,
                %set to closest. check opposite as well. otherwise toss.
                if (node1Dist< node2Dist && node1Dist<minNeighborDist)
                    minNeighborDist = node1Dist;
                    nodeFlag = true;
                    closestNode = node1;
                    closestNode = cat(2, closestNode,edges(i,7:11));
                elseif (node2Dist< node1Dist && node2Dist<minNeighborDist)
                    minNeighborDist = node2Dist;
                    nodeFlag = true;
                    closestNode = node2;
                    closestNode = cat(2, closestNode,edges(i,7:11));
                end
            else
                %if it is between, check vs. the shortest length
                nodeDist = sqrt((node(1)-sample(1)).^2 + (node(2)-sample(2)).^2)+dTheta;
                %if node less than the current minimum, set to min and
                %closest node. repeats until all edges have been
                %traversed.
                if nodeDist<minNeighborDist
                    nodeFlag = false;
                    minNeighborDist = nodeDist;
                    closestNode = node;
                    closestNode = cat(2, closestNode,edges(i,7:11));
                end
            end
        end
    end

    
    %now that closestNode to sample on tree has been found
    %compute vector from closestNode to sample
    
    vec = [sample(1)-closestNode(1),sample(2)-closestNode(2)];
    vecMag = sqrt(vec(1)^2 + vec(2)^2);
    
    %Only travel the distance to the sample if it is closer than epsilon
    if vecMag < epsilon
        epsilon = vecMag;
    end
    
    %compute change in linear velocity and change in angular velocity to
    %get to from the closestNode to Sample
    dV = sample(4) - closestNode(5); %vf - vi
    dW = sample(5) - closestNode(6); %wf - wi
    
    %compute the average speed while traveling this new edge
    avgV = (2*closestNode(5) + dV)/2;
    %compute the time to travel the distance epsilon
    time = abs(epsilon/avgV);
    
    %Compute the acceleration (linear and angular: constants), and set
    %within limits if they exceed.
    a = dV/time;
    if a < aMin
        a = aMin;
    elseif a > aMax
        a = aMax;
    end
    
    g = dW/time;
    if g < gMin
        g = gMin;
    elseif g > gMax
        g = gMax;
    end
    
    %Initialize the start of the trajectory
    traj = closestNode;
    %plot(traj(1),traj(2),'.g','HandleVisibility','off');
    
    %Loop through delta time chunks of the trajectory until total time met
    for j = 0:delta:time
        %This if statement makes the robot stop as soon as it has reached
        %the edge of the goal
        if goalFlag == false
            
            %get most recent trajectory point
            trajLength = size(traj,1);
            if size(traj,1)~=1
                trajInput = traj(trajLength,:);
            else
                trajInput = traj;
            end
            
            %Use forward Euler method to solve the initial value problem:
            %Input either the trajectory from the existing graph, so that
            %it is continuous, or from the most recent delta chunk along
            %the new trajectory
            newTraj = RoboSim(trajInput,a,g,delta);
            
            newTraj(7) = a;
            newTraj(8) = g;
            %Add new trajectory chunk to the path
            traj = cat(1, traj, newTraj);
            
            
            %translate the rest of the robot points to the delta (which is new center) location
            xrobDel = xrob+traj(trajLength+1,1);
            yrobDel = yrob+traj(trajLength+1,2);
            
            
            %rotate all of the points about the center point
            for i = 2:37
                %distance from center to that point
                hyp = sqrt((xrobDel(i)-xrobDel(1))^2 + (yrobDel(i)-yrobDel(1))^2);
                %old angle between center and that point
                prevAngle = atan2((yrobDel(i)-yrobDel(1)),(xrobDel(i)-xrobDel(1)));
                %new angle based on rotation of trajector
                newAngle = prevAngle+newTraj(4);
                %compute new robot point locations
                xrobDel(i) = xrobDel(1)+hyp*cos(newAngle);
                yrobDel(i) = yrobDel(1)+hyp*sin(newAngle);
            end
            
            %collision check all obstacles, unless collision already
            %detected
            for i = 1:numObs
                if obsFlag==false
                    %define the current obstacle we are checking against
                    currObs = obstacles_raw(i,:);
                    distSquare = (xrobDel(1) - currObs(1)).^2 + (yrobDel(1) - currObs(2)).^2;
                    %Add bubble to the center robot point, radius 1
                    rSquare = (currObs(3)+1)^2;
                    if (distSquare < rSquare)
                        %If obstacle collides with robot bubble, check each
                        %robot point
                        for j = 1:length(xrob)
                            %distance between
                            distSquare = (xrobDel(j) - currObs(1)).^2 + (yrobDel(j) - currObs(2)).^2;
                            rSquare = currObs(3)^2;
                            
                            %robot point is within an obstacle
                            if (distSquare < rSquare)
                                %collision has occurred
                                obsFlag = true;
                            end
                        end
                        
                        %If there is no collision with an obstacle, check
                        %that the robot center is within the R^2 space
                    elseif (xrobDel(1)>100)
                        obsFlag = true;
                    elseif (xrobDel(1)<0)
                        obsFlag = true;
                    elseif (yrobDel(1)>100)
                        obsFlag = true;
                    elseif (yrobDel(1)<0)
                        obsFlag = true;
                    end
                end
            end
            
            %%goal check
            goalDistSquare = (xrobDel(1) - goal(1))^2 + (yrobDel(1) - goal(2))^2;
            rGoalSquare = (goal(3)+1)^2;
            if (goalDistSquare < rGoalSquare && obsFlag == false)
                %If inside the bubble, then do the same check, but loop through all the
                %points that make up the robot
                
                rGoalSquare = goal(3)^2;
                for i = 1:length(xrob)
                    %Stop if even 1 point is inside goal
                    if goalFlag ==false
                        %xrobDel and yrobDel contain the values of the points relative to the
                        %center, which is xrobDel(1) yrobDel(1)
                        goalDistSquare = (xrobDel(i) - goal(1)).^2 + (yrobDel(i) - goal(2)).^2;
                        
                        %if the point is within the goal region, and we havent hit an
                        %obstacle
                        if (goalDistSquare < rGoalSquare && obsFlag == false)
                            goalNode = newTraj
                            goalFlag = true;
                            
                            %%create new edge, which contains x and y of the closestNode
                            %%from the graph, and the trajectory point where goal is
                            %%reached
                            goalEdge(1,1:3) =  closestNode(1:3);
                            goalEdge(1,4:6) =  newTraj(1:3);
                            goalEdge(1,7:11) =  closestNode(4:8);
                            %Add this to the total edges list
                            edges = cat(1,edges,goalEdge);
                        end
                    end
                end
            end
        end
        %loop through and check again along the path until epsilon is met
    end
    
    %if not obstacle, add edges and plot
    if obsFlag == false
        
        trajLength = size(traj,1);
        %loop through all the sub trajectories and add them as edges
        for i = 2:trajLength
            
            newEdge(1,1:3) =  traj(i-1,1:3);
            newEdge(1,4:6) =  traj(i,1:3);
            newEdge(1,7:11) =  traj(i-1,4:8);
            
            %path = cat(2,path,traj(i,:))
            
            plot([newEdge(1) newEdge(4)],[newEdge(2) newEdge(5)],'k','HandleVisibility','off');
            edges = cat(1,edges,newEdge);
            
        end
    else
    end
    
    %reset epsilon if it had been shortened due to a close sample
    epsilon = 5;
    
    
end
path = [];
pathLength = size(edges,1);
path(:,1) = edges(2:pathLength,3);
path(:,2) = edges(2:pathLength,1);
path(:,3) = edges(2:pathLength,2);
path(:,4:8) = edges(2:pathLength,7:11);

csvwrite('HW3_P3.txt', path);


plot(xrobDel,yrobDel,'.g','HandleVisibility','off');
%Plot initial position of the robot
plot(xrobStart,yrobStart,'.b','HandleVisibility','off');
plot(0,0,'k','DisplayName','Tree');
legend('Location','eastoutside')
legend('boxoff')
title('R^2')
hold off;

toc()

%% Problem 4
disp('--4--')
epsilon = 10
%Define the system State Variable at start:
start = [63, 80, 0, pi/2, 0, 0, 0, 0]
%edge = [x1  y1 t1  th  v  w  a  g]

goal = [60 10 10]


xGoal = goal(3)*cos(th(:)) + goal(1);
yGoal = goal(3)*sin(th(:)) + goal(2);
goalFlag = false;
%put the robot at the start location first.
xrobStart = xrob+start(1);
yrobStart = yrob+start(2);
%need to rotate all of the points about the center point, by start(4)
for i = 2:37
    hyp = sqrt((xrobStart(i)-xrobStart(1))^2 + (yrobStart(i)-yrobStart(1))^2);
    prevAngle = atan2((yrobStart(i)-yrobStart(1)),(xrobStart(i)-xrobStart(1)));
    newAngle = prevAngle+start(4);
    xrobStart(i) = xrobStart(1)+hyp*cos(newAngle);
    yrobStart(i) = yrobStart(1)+hyp*sin(newAngle);
end



% Initialize the plot
figure(4), hold on
h = legend('show','location','best');
%Plot goal region
plot(xGoal,yGoal,'g','DisplayName','Goal');
%Plot initial position of the robot
plot(xrobStart,yrobStart,'.b','HandleVisibility','off');
axis(C);
xlabel('x');
ylabel('y');
grid on;
axis square;
%Plot the obstacles
for i = 1:numObs
    plot(xunitobs(i,:), yunitobs(i,:),'r','HandleVisibility','off');
end
plot(0,0,'r','DisplayName','Obstacle');


%Edges contain from & to nodes, times, as well as theta, linear velocity V,
%and angular velocity W, linear acceleration a, and angular acceleration g.
edges = [100000 100000 100000 100000 100000 100000 100000 100000 100000 100000 100000];
%edge = [  x1     y1     t1     x2     y2     t2   theta     v      w      a      g  ]


while goalFlag == false
    %Take a C-Space sample
    sampleTheta = -2*pi + 4*pi*rand; %-2pi to 2pi
    sampleVel = -5 + 10*rand; %-5 to 5
    sampleAng = -pi/2 + pi*rand; %-pi/2 to pi/2
    sample = [100*rand 100*rand sampleTheta sampleVel sampleAng];
    %plot(sample(1),sample(2),'*k','HandleVisibility','off')
    
    %reset obstacle flag
    obsFlag = false;
    
    if size(edges,1)==1
        closestNode = start;
    else
        dist = zeros(size(edges,1),2);
        %initialize minimum distance artificially high so that it must be overwritten
        minNeighborDist = 10000;
        
        %loop through all the edges.
        for i = 1:size(edges,1)
            
            %grab nodes from this edge.
            node1 = edges(i,1:3);
            node2 = edges(i,4:6);
            
            %grab theta of these nodes and find difference between it and
            %the sample
            nodeTheta = edges(i,7);
            dTheta = abs(sample(3)-nodeTheta);

            
            %define line containing the edges end nodes and its normal
            line = [(node2(1)-node1(1)),(node2(2)-node1(2))];
            lineMag = sqrt(line(1)^2 +line(2)^2);
            n = line/lineMag;
            
            %%finds closest node on the line containing node1 and node2
            node = sample(1:2) + (node1(1:2)-sample(1:2))-(dot((node1(1:2)-sample(1:2)), n))*n;
            %IF THE CLOSEST NODE IS ON THE LINE, AVERAGE THE TIME BETWEEN
            %THE TWO
            node(3) = (node1(3)+node2(3))/2;
            
            dotProduct = (sample(1)-node1(1))*(node2(1)-node1(1))+(sample(2)-node1(2))*(node2(2)-node1(2));
            squaredLength = (node2(1)-node1(1))*(node2(1)-node1(1))+(node2(2)-node1(2))*(node2(2)-node1(2));
            %checking if the closest node on the line is between two points
            if (dotProduct<0 || squaredLength<dotProduct)
                %if not between, check which node is closer (factoring in
                %rotation)
                node1Dist = sqrt((node1(1)-sample(1)).^2 + (node1(2)-sample(2)).^2)+dTheta;
                node2Dist = sqrt((node2(1)-sample(1)).^2 + (node2(2)-sample(2)).^2)+dTheta;
                %if node1 closer node2 and less than the current minimum,
                %set to closest. check opposite as well. otherwise toss.
                if (node1Dist< node2Dist && node1Dist<minNeighborDist)
                    minNeighborDist = node1Dist;
                    nodeFlag = true;
                    closestNode = node1;
                    closestNode = cat(2, closestNode,edges(i,7:11));
                elseif (node2Dist< node1Dist && node2Dist<minNeighborDist)
                    minNeighborDist = node2Dist;
                    nodeFlag = true;
                    closestNode = node2;
                    closestNode = cat(2, closestNode,edges(i,7:11));
                end
            else
                %if it is between, check vs. the shortest length
                nodeDist = sqrt((node(1)-sample(1)).^2 + (node(2)-sample(2)).^2)+dTheta;
                %if node less than the current minimum, set to min and
                %closest node. repeats until all edges have been
                %traversed.
                if nodeDist<minNeighborDist
                    nodeFlag = false;
                    minNeighborDist = nodeDist;
                    closestNode = node;
                    closestNode = cat(2, closestNode,edges(i,7:11));
                end
            end
        end
    end
    
    %now that closestNode to sample on tree has been found
    %compute vector from closestNode to sample
    
    vec = [sample(1)-closestNode(1),sample(2)-closestNode(2)];
    vecMag = sqrt(vec(1)^2 + vec(2)^2);
    
    %Only travel the distance to the sample if it is closer than epsilon
    if vecMag < epsilon
        epsilon = vecMag;
    end
    
    %compute change in linear velocity and change in angular velocity to
    %get to from the closestNode to Sample
    dV = sample(4) - closestNode(5); %vf - vi
    dW = sample(5) - closestNode(6); %wf - wi
    
    %compute the average speed while traveling this new edge
    avgV = (2*closestNode(5) + dV)/2;
    %compute the time to travel the distance epsilon
    time = abs(epsilon/avgV);
    
    %Compute the acceleration (linear and angular: constants), and set
    %within limits if they exceed.
    a = dV/time;
    if a < aMin
        a = aMin;
    elseif a > aMax
        a = aMax;
    end
    
    g = dW/time;
    if g < gMin
        g = gMin;
    elseif g > gMax
        g = gMax;
    end
    
    %Initialize the start of the trajectory
    traj = closestNode;
    %plot(traj(1),traj(2),'.g','HandleVisibility','off');
    
    %Loop through delta time chunks of the trajectory until total time met
    for j = 0:delta:time
        %This if statement makes the robot stop as soon as it has reached
        %the edge of the goal
        if goalFlag == false
            
            %get most recent trajectory point
            trajLength = size(traj,1);
            if size(traj,1)~=1
                trajInput = traj(trajLength,:);
            else
                trajInput = traj;
            end
            
            %Use forward Euler method to solve the initial value problem:
            %Input either the trajectory from the existing graph, so that
            %it is continuous, or from the most recent delta chunk along
            %the new trajectory
            newTraj = RoboSim(trajInput,a,g,delta);
            
            newTraj(7) = a;
            newTraj(8) = g;
            %Add new trajectory chunk to the path
            traj = cat(1, traj, newTraj);
            
            
            %translate the rest of the robot points to the delta (which is new center) location
            xrobDel = xrob+traj(trajLength+1,1);
            yrobDel = yrob+traj(trajLength+1,2);
            
            
            %rotate all of the points about the center point
            for i = 2:37
                %distance from center to that point
                hyp = sqrt((xrobDel(i)-xrobDel(1))^2 + (yrobDel(i)-yrobDel(1))^2);
                %old angle between center and that point
                prevAngle = atan2((yrobDel(i)-yrobDel(1)),(xrobDel(i)-xrobDel(1)));
                %new angle based on rotation of trajector
                newAngle = prevAngle+newTraj(4);
                %compute new robot point locations
                xrobDel(i) = xrobDel(1)+hyp*cos(newAngle);
                yrobDel(i) = yrobDel(1)+hyp*sin(newAngle);
            end
            
            %collision check all obstacles, unless collision already
            %detected
            for i = 1:numObs
                if obsFlag==false
                    %define the current obstacle we are checking against
                    currObs = obstacles_raw(i,:);
                    distSquare = (xrobDel(1) - currObs(1)).^2 + (yrobDel(1) - currObs(2)).^2;
                    %Add bubble to the center robot point, radius 1
                    rSquare = (currObs(3)+1)^2;
                    if (distSquare < rSquare)
                        %If obstacle collides with robot bubble, check each
                        %robot point
                        for j = 1:length(xrob)
                            %distance between
                            distSquare = (xrobDel(j) - currObs(1)).^2 + (yrobDel(j) - currObs(2)).^2;
                            rSquare = currObs(3)^2;
                            
                            %robot point is within an obstacle
                            if (distSquare < rSquare)
                                %collision has occurred
                                obsFlag = true;
                            end
                        end
                        
                        %If there is no collision with an obstacle, check
                        %that the robot center is within the R^2 space
                    elseif (xrobDel(1)>100)
                        obsFlag = true;
                    elseif (xrobDel(1)<0)
                        obsFlag = true;
                    elseif (yrobDel(1)>100)
                        obsFlag = true;
                    elseif (yrobDel(1)<0)
                        obsFlag = true;
                    end
                end
            end
            
            %%goal check
            goalDistSquare = (xrobDel(1) - goal(1))^2 + (yrobDel(1) - goal(2))^2;
            rGoalSquare = (goal(3)+1)^2;
            if (goalDistSquare < rGoalSquare && obsFlag == false)
                %If inside the bubble, then do the same check, but loop through all the
                %points that make up the robot
                
                rGoalSquare = goal(3)^2;
                for i = 1:length(xrob)
                    %Stop if even 1 point is inside goal
                    if goalFlag ==false
                        %xrobDel and yrobDel contain the values of the points relative to the
                        %center, which is xrobDel(1) yrobDel(1)
                        goalDistSquare = (xrobDel(i) - goal(1)).^2 + (yrobDel(i) - goal(2)).^2;
                        
                        %if the point is within the goal region, and we havent hit an
                        %obstacle
                        if (goalDistSquare < rGoalSquare && obsFlag == false)
                            goalNode = newTraj
                            goalFlag = true;
                            
                            %%create new edge, which contains x and y of the closestNode
                            %%from the graph, and the trajectory point where goal is
                            %%reached
                            goalEdge(1,1:3) =  closestNode(1:3);
                            goalEdge(1,4:6) =  newTraj(1:3);
                            goalEdge(1,7:11) =  closestNode(4:8);
                            %Add this to the total edges list
                            edges = cat(1,edges,goalEdge);
                        end
                    end
                end
            end
        end
        %loop through and check again along the path until epsilon is met
    end
    
    %if not obstacle, add edges and plot
    if obsFlag == false
        
        trajLength = size(traj,1);
        %loop through all the sub trajectories and add them as edges
        for i = 2:trajLength
            
            newEdge(1,1:3) =  traj(i-1,1:3);
            newEdge(1,4:6) =  traj(i,1:3);
            newEdge(1,7:11) =  traj(i-1,4:8);
            
            %path = cat(2,path,traj(i,:))
            
            plot([newEdge(1) newEdge(4)],[newEdge(2) newEdge(5)],'k','HandleVisibility','off');
            edges = cat(1,edges,newEdge);
            
        end
    else
    end
    
    %reset epsilon if it had been shortened due to a close sample
    epsilon = 10;
    
    
end

path = [];
pathLength = size(edges,1);
path(:,1) = edges(2:pathLength,3);
path(:,2) = edges(2:pathLength,1);
path(:,3) = edges(2:pathLength,2);
path(:,4:8) = edges(2:pathLength,7:11);

csvwrite('HW3_P4.txt', path);

plot(xrobDel,yrobDel,'.g','HandleVisibility','off');
%Plot initial position of the robot
plot(xrobStart,yrobStart,'.b','HandleVisibility','off');
plot(0,0,'k','DisplayName','Tree');
legend('Location','eastoutside')
legend('boxoff')
title('R^2')
hold off;

toc()

%% Problem 5
disp('--5--')
epsilon = 20
%Define the system State Variable at start:
start = [40, 40, 0, pi/4, 0, 0, 0, 0]
%edge = [x1  y1 t1  th  v  w  a  g]

goal = [0 0 20]


xGoal = goal(3)*cos(th(:)) + goal(1);
yGoal = goal(3)*sin(th(:)) + goal(2);
goalFlag = false;
%put the robot at the start location first.
xrobStart = xrob+start(1);
yrobStart = yrob+start(2);
%need to rotate all of the points about the center point, by start(4)
for i = 2:37
    hyp = sqrt((xrobStart(i)-xrobStart(1))^2 + (yrobStart(i)-yrobStart(1))^2);
    prevAngle = atan2((yrobStart(i)-yrobStart(1)),(xrobStart(i)-xrobStart(1)));
    newAngle = prevAngle+start(4);
    xrobStart(i) = xrobStart(1)+hyp*cos(newAngle);
    yrobStart(i) = yrobStart(1)+hyp*sin(newAngle);
end



% Initialize the plot
figure(5), hold on
h = legend('show','location','best');
%Plot goal region
plot(xGoal,yGoal,'g','DisplayName','Goal');
%Plot initial position of the robot
plot(xrobStart,yrobStart,'.b','HandleVisibility','off');
axis(C);
xlabel('x');
ylabel('y');
grid on;
axis square;
%Plot the obstacles
for i = 1:numObs
    plot(xunitobs(i,:), yunitobs(i,:),'r','HandleVisibility','off');
end
plot(0,0,'r','DisplayName','Obstacle');


%Edges contain from & to nodes, times, as well as theta, linear velocity V,
%and angular velocity W, linear acceleration a, and angular acceleration g.
edges = [100000 100000 100000 100000 100000 100000 100000 100000 100000 100000 100000];
%edge = [  x1     y1     t1     x2     y2     t2   theta     v      w      a      g  ]


while goalFlag == false
    %Take a C-Space sample
    sampleTheta = -2*pi + 4*pi*rand; %-2pi to 2pi
    sampleVel = -5 + 10*rand; %-5 to 5
    sampleAng = -pi/2 + pi*rand; %-pi/2 to pi/2
    sample = [100*rand 100*rand sampleTheta sampleVel sampleAng];
    %plot(sample(1),sample(2),'*k','HandleVisibility','off')
    
    %reset obstacle flag
    obsFlag = false;
    
    if size(edges,1)==1
        closestNode = start;
    else
        dist = zeros(size(edges,1),2);
        %initialize minimum distance artificially high so that it must be overwritten
        minNeighborDist = 10000;
        
        %loop through all the edges.
        for i = 1:size(edges,1)
            
            %grab nodes from this edge.
            node1 = edges(i,1:3);
            node2 = edges(i,4:6);
            
            %grab theta of these nodes and find difference between it and
            %the sample
            nodeTheta = edges(i,7);
            dTheta = abs(sample(3)-nodeTheta);

            
            %define line containing the edges end nodes and its normal
            line = [(node2(1)-node1(1)),(node2(2)-node1(2))];
            lineMag = sqrt(line(1)^2 +line(2)^2);
            n = line/lineMag;
            
            %%finds closest node on the line containing node1 and node2
            node = sample(1:2) + (node1(1:2)-sample(1:2))-(dot((node1(1:2)-sample(1:2)), n))*n;
            %IF THE CLOSEST NODE IS ON THE LINE, AVERAGE THE TIME BETWEEN
            %THE TWO
            node(3) = (node1(3)+node2(3))/2;
            
            dotProduct = (sample(1)-node1(1))*(node2(1)-node1(1))+(sample(2)-node1(2))*(node2(2)-node1(2));
            squaredLength = (node2(1)-node1(1))*(node2(1)-node1(1))+(node2(2)-node1(2))*(node2(2)-node1(2));
            %checking if the closest node on the line is between two points
            if (dotProduct<0 || squaredLength<dotProduct)
                %if not between, check which node is closer (factoring in
                %rotation)
                node1Dist = sqrt((node1(1)-sample(1)).^2 + (node1(2)-sample(2)).^2)+dTheta;
                node2Dist = sqrt((node2(1)-sample(1)).^2 + (node2(2)-sample(2)).^2)+dTheta;
                %if node1 closer node2 and less than the current minimum,
                %set to closest. check opposite as well. otherwise toss.
                if (node1Dist< node2Dist && node1Dist<minNeighborDist)
                    minNeighborDist = node1Dist;
                    nodeFlag = true;
                    closestNode = node1;
                    closestNode = cat(2, closestNode,edges(i,7:11));
                elseif (node2Dist< node1Dist && node2Dist<minNeighborDist)
                    minNeighborDist = node2Dist;
                    nodeFlag = true;
                    closestNode = node2;
                    closestNode = cat(2, closestNode,edges(i,7:11));
                end
            else
                %if it is between, check vs. the shortest length
                nodeDist = sqrt((node(1)-sample(1)).^2 + (node(2)-sample(2)).^2)+dTheta;
                %if node less than the current minimum, set to min and
                %closest node. repeats until all edges have been
                %traversed.
                if nodeDist<minNeighborDist
                    nodeFlag = false;
                    minNeighborDist = nodeDist;
                    closestNode = node;
                    closestNode = cat(2, closestNode,edges(i,7:11));
                end
            end
        end
    end
    
    %now that closestNode to sample on tree has been found
    %compute vector from closestNode to sample
    
    vec = [sample(1)-closestNode(1),sample(2)-closestNode(2)];
    vecMag = sqrt(vec(1)^2 + vec(2)^2);
    
    %Only travel the distance to the sample if it is closer than epsilon
    if vecMag < epsilon
        epsilon = vecMag;
    end
    
    %compute change in linear velocity and change in angular velocity to
    %get to from the closestNode to Sample
    dV = sample(4) - closestNode(5); %vf - vi
    dW = sample(5) - closestNode(6); %wf - wi
    
    %compute the average speed while traveling this new edge
    avgV = (2*closestNode(5) + dV)/2;
    %compute the time to travel the distance epsilon
    time = abs(epsilon/avgV);
    
    %Compute the acceleration (linear and angular: constants), and set
    %within limits if they exceed.
    a = dV/time;
    if a < aMin
        a = aMin;
    elseif a > aMax
        a = aMax;
    end
    
    g = dW/time;
    if g < gMin
        g = gMin;
    elseif g > gMax
        g = gMax;
    end
    
    %Initialize the start of the trajectory
    traj = closestNode;
    %plot(traj(1),traj(2),'.g','HandleVisibility','off');
    
    %Loop through delta time chunks of the trajectory until total time met
    for j = 0:delta:time
        %This if statement makes the robot stop as soon as it has reached
        %the edge of the goal
        if goalFlag == false
            
            %get most recent trajectory point
            trajLength = size(traj,1);
            if size(traj,1)~=1
                trajInput = traj(trajLength,:);
            else
                trajInput = traj;
            end
            
            %Use forward Euler method to solve the initial value problem:
            %Input either the trajectory from the existing graph, so that
            %it is continuous, or from the most recent delta chunk along
            %the new trajectory
            newTraj = RoboSim(trajInput,a,g,delta);
            
            newTraj(7) = a;
            newTraj(8) = g;
            %Add new trajectory chunk to the path
            traj = cat(1, traj, newTraj);
            
            
            %translate the rest of the robot points to the delta (which is new center) location
            xrobDel = xrob+traj(trajLength+1,1);
            yrobDel = yrob+traj(trajLength+1,2);
            
            
            %rotate all of the points about the center point
            for i = 2:37
                %distance from center to that point
                hyp = sqrt((xrobDel(i)-xrobDel(1))^2 + (yrobDel(i)-yrobDel(1))^2);
                %old angle between center and that point
                prevAngle = atan2((yrobDel(i)-yrobDel(1)),(xrobDel(i)-xrobDel(1)));
                %new angle based on rotation of trajector
                newAngle = prevAngle+newTraj(4);
                %compute new robot point locations
                xrobDel(i) = xrobDel(1)+hyp*cos(newAngle);
                yrobDel(i) = yrobDel(1)+hyp*sin(newAngle);
            end
            
            %collision check all obstacles, unless collision already
            %detected
            for i = 1:numObs
                if obsFlag==false
                    %define the current obstacle we are checking against
                    currObs = obstacles_raw(i,:);
                    distSquare = (xrobDel(1) - currObs(1)).^2 + (yrobDel(1) - currObs(2)).^2;
                    %Add bubble to the center robot point, radius 1
                    rSquare = (currObs(3)+1)^2;
                    if (distSquare < rSquare)
                        %If obstacle collides with robot bubble, check each
                        %robot point
                        for j = 1:length(xrob)
                            %distance between
                            distSquare = (xrobDel(j) - currObs(1)).^2 + (yrobDel(j) - currObs(2)).^2;
                            rSquare = currObs(3)^2;
                            
                            %robot point is within an obstacle
                            if (distSquare < rSquare)
                                %collision has occurred
                                obsFlag = true;
                            end
                        end
                        
                        %If there is no collision with an obstacle, check
                        %that the robot center is within the R^2 space
                    elseif (xrobDel(1)>100)
                        obsFlag = true;
                    elseif (xrobDel(1)<0)
                        obsFlag = true;
                    elseif (yrobDel(1)>100)
                        obsFlag = true;
                    elseif (yrobDel(1)<0)
                        obsFlag = true;
                    end
                end
            end
            
            %%goal check
            goalDistSquare = (xrobDel(1) - goal(1))^2 + (yrobDel(1) - goal(2))^2;
            rGoalSquare = (goal(3)+1)^2;
            if (goalDistSquare < rGoalSquare && obsFlag == false)
                %If inside the bubble, then do the same check, but loop through all the
                %points that make up the robot
                
                rGoalSquare = goal(3)^2;
                for i = 1:length(xrob)
                    %Stop if even 1 point is inside goal
                    if goalFlag ==false
                        %xrobDel and yrobDel contain the values of the points relative to the
                        %center, which is xrobDel(1) yrobDel(1)
                        goalDistSquare = (xrobDel(i) - goal(1)).^2 + (yrobDel(i) - goal(2)).^2;
                        
                        %if the point is within the goal region, and we havent hit an
                        %obstacle
                        if (goalDistSquare < rGoalSquare && obsFlag == false)
                            goalNode = newTraj
                            goalFlag = true;
                            
                            %%create new edge, which contains x and y of the closestNode
                            %%from the graph, and the trajectory point where goal is
                            %%reached
                            goalEdge(1,1:3) =  closestNode(1:3);
                            goalEdge(1,4:6) =  newTraj(1:3);
                            goalEdge(1,7:11) =  closestNode(4:8);
                            %Add this to the total edges list
                            edges = cat(1,edges,goalEdge);
                        end
                    end
                end
            end
        end
        %loop through and check again along the path until epsilon is met
    end
    
    %if not obstacle, add edges and plot
    if obsFlag == false
        
        trajLength = size(traj,1);
        %loop through all the sub trajectories and add them as edges
        for i = 2:trajLength
            
            newEdge(1,1:3) =  traj(i-1,1:3);
            newEdge(1,4:6) =  traj(i,1:3);
            newEdge(1,7:11) =  traj(i-1,4:8);
            
            %path = cat(2,path,traj(i,:))
            
            plot([newEdge(1) newEdge(4)],[newEdge(2) newEdge(5)],'k','HandleVisibility','off');
            edges = cat(1,edges,newEdge);
            
        end
    else
    end
    
    %reset epsilon if it had been shortened due to a close sample
    epsilon = 20;
    
    
end


path = [];
pathLength = size(edges,1);
path(:,1) = edges(2:pathLength,3);
path(:,2) = edges(2:pathLength,1);
path(:,3) = edges(2:pathLength,2);
path(:,4:8) = edges(2:pathLength,7:11);

csvwrite('HW3_P5.txt', path);


plot(xrobDel,yrobDel,'.g','HandleVisibility','off');
%Plot initial position of the robot
plot(xrobStart,yrobStart,'.b','HandleVisibility','off');
plot(0,0,'k','DisplayName','Tree');
legend('Location','eastoutside')
legend('boxoff')
title('R^2')
hold off;

toc()




