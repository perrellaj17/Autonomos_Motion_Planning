% Copyright 2018, Michael Otte
% Modified Joe Perrella 02/28/2020

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

delta = .001;


%%Global obstacles and configuration space
obstacles_raw = csvread('obstacles.txt');
x = obstacles_raw(:,1);
y = obstacles_raw(:,2);
r = obstacles_raw(:,3);
th = 0:pi/50:2*pi;
xunit = zeros(1,10);
yunit = zeros(1,10);
numObs = 24;
for i = 1:numObs
    for j = 1:length(th)
        xunit(i,j) = r(i)*cos(th(j)) + x(i);
        yunit(i,j) = r(i)*sin(th(j)) + y(i);
    end
end
C = [0 100 0 100];


%% Problem 1
disp('--1--')
epsilon = 10
start = [75,85]
goal = [100 0 20]
xGoal = goal(3)*cos(th(:)) + goal(1);
yGoal = goal(3)*sin(th(:)) + goal(2);
goalFlag = false;


%plot the obstacles
figure(1), hold on
h = legend('show','location','best');
%plot Start
plot(start(1),start(2),'*m','DisplayName','Start')
%Plot goal region
plot(xGoal,yGoal,'g','DisplayName','Goal');

axis(C);
xlabel('x');
ylabel('y');
grid on;
axis square;

for i = 1:numObs
    plot(xunit(i,:), yunit(i,:),'r','HandleVisibility','off');
end
plot(0,0,'r','DisplayName','Obstacle');

e1 = [];
e2 = [];
edges = [100000 100000 100000 100000];
while goalFlag == false
    sample = [100*rand 100*rand];
    %reset obstacle flag
    obsFlag = false;
    %plot(sample(1),sample(2),'*k');
    if size(edges,1)==1
        closestNode = start;
    else
        %disp('-----')
        dist = zeros(size(edges,1),2);
        
        minNeighborDist = 10000;
        %loop through all the edges.
        for i = 1:size(edges,1)
            dist(i,1) = sqrt((edges(i,1)-sample(1)).^2 + (edges(i,2)-sample(2)).^2);
            dist(i,2) = sqrt((edges(i,3)-sample(1)).^2 + (edges(i,4)-sample(2)).^2);
            %grab nodes from this edge.
            node1 = edges(i,1:2);
            node2 = edges(i,3:4);
            
            %define line containing the edges end nodes and its normal
            line = [(node2(1)-node1(1)),(node2(2)-node1(2))];
            lineMag = sqrt(line(1)^2 +line(2)^2);
            n = line/lineMag;
            
            %%finds closest node on the line containing node1 and node2
            node = sample + (node1-sample)-(dot((node1-sample), n))*n;
            dotProduct = (sample(1)-node1(1))*(node2(1)-node1(1))+(sample(2)-node1(2))*(node2(2)-node1(2));
            squaredLength = (node2(1)-node1(1))*(node2(1)-node1(1))+(node2(2)-node1(2))*(node2(2)-node1(2));
            
            %checking if the closest node on the line is between two points
            if (dotProduct<0 || squaredLength<dotProduct)
                %if not between, check which node is closer.
                
                node1Dist = sqrt((node1(1)-sample(1)).^2 + (node1(2)-sample(2)).^2);
                node2Dist = sqrt((node2(1)-sample(1)).^2 + (node2(2)-sample(2)).^2);
                %if node1 closer node2 and less than the current minimum,
                %set to closest. check opposite as well. otherwise toss.
                if (node1Dist< node2Dist && node1Dist<minNeighborDist)
                    minNeighborDist = node1Dist;
                    nodeFlag = true;
                    closestNode = node1;
                elseif (node2Dist< node1Dist && node2Dist<minNeighborDist)
                    minNeighborDist = node2Dist;
                    nodeFlag = true;
                    closestNode = node2;
                end
            else
                %if it is between, check vs. the shortest length
                
                nodeDist = sqrt((node(1)-sample(1)).^2 + (node(2)-sample(2)).^2);
                %if node less than the current minimum, set to min and
                %closest node. repeats until all edges have been
                %traversed.
                if nodeDist<minNeighborDist
                    nodeFlag = false;
                    minNeighborDist = nodeDist;
                    closestNode = node;
                    closestNode1 = node1;
                    closestNode2 = node2;
                end
            end
        end

        e1 = [];
        e2 = [];
        %if node was between the two lines, define new edges, one for each segment.
        if nodeFlag ==false
            %adding edge from the new middle node to the edge nodes
            e1 =  cat(2, closestNode1,closestNode);
            e2 =  cat(2, closestNode2,closestNode);
        end

    end
    %now that closestNode on tree to sample has been found    
    %compute vector from closestNode to sample
    vec = [sample(1)-closestNode(1),sample(2)-closestNode(2)];
    vecMag = sqrt(vec(1)^2 + vec(2)^2);
    
    %Only travel the distance to the sample if it is closer than epsilon
    if vecMag < epsilon
        epsilon = vecMag;
    end
    
    %step increments of delta along potential new edge to collision check 
    for j = 0:delta:epsilon
        %multiply the unit vector going from current node to sample by delta.
        delVec = j*(vec./vecMag);
        
        %collision check at the location delta*j away from the node
        %in the direction of the sample
        delNode = closestNode+delVec;
        
        %obstacle check all obstacles 
        for i = 1:numObs
            currObs = obstacles_raw(i,:);
            
            distSquare = (delNode(1) - currObs(1)).^2 + (delNode(2) - currObs(2)).^2;
            rSquare = currObs(3)^2;

            if (distSquare < rSquare)
                %Check if inside obstacle or out of bounds, trigger obstacle flag
                obsFlag = true;
            elseif (delNode(1)>100)
                obsFlag = true;
            elseif (delNode(1)<0)
                obsFlag = true;
            elseif (delNode(2)>100)
                obsFlag = true;
            elseif (delNode(2)<0)
                obsFlag = true;
            end
        end
        %loop through and check again along the path until epsilon is met
    end
    
    %%goal check
    goalDistSquare = (delNode(1) - goal(1))^2 + (delNode(2) - goal(2))^2;
    rGoalSquare = goal(3)^2;
    if (goalDistSquare < rGoalSquare && obsFlag == false)
        % The object is inside of the goal,trip the goal flag and store for
        % plot
        goalNode = delNode
        goalFlag = true;
        goalEdge =  cat(2, closestNode,delNode);
        edges = cat(1,edges,goalEdge);
    end
    
    %if not goal, but also not obstacle, add edges and plot
    
    if obsFlag == false
        newEdge =  cat(2, closestNode,delNode);
        edges = cat(1,edges,newEdge);     
        edges = cat(1,edges,e1);
        edges = cat(1,edges,e2);
        plot([closestNode(1) delNode(1)],[closestNode(2) delNode(2)],'k','HandleVisibility','off');
    else
        %plot(sample(1),sample(2),'*r');
    end

    epsilon = 10;
end

plot(0,0,'k','DisplayName','Tree');
legend('Location','eastoutside')
legend('boxoff')
title('C')
hold off;

%% Problem 2

disp('--2--')
epsilon = 5
start = [20,10]
goal = [75 85 10]
goalFlag = false;
sample = round([100*rand 100*rand],0);
xGoal = goal(3)*cos(th(:)) + goal(1);
yGoal = goal(3)*sin(th(:)) + goal(2);


%plot the obstacles
figure(2), hold on
h = legend('show','location','best');
%plot Start
plot(start(1),start(2),'*m','DisplayName','Start')
%Plot goal region
plot(xGoal,yGoal,'g','DisplayName','Goal');

axis(C);
xlabel('x');
ylabel('y');
grid on;
axis square;

for i = 1:numObs
    plot(xunit(i,:), yunit(i,:),'r','HandleVisibility','off');
end
plot(0,0,'r','DisplayName','Obstacle');

e1 = [];
e2 = [];
edges = [100000 100000 100000 100000];
while goalFlag == false
    sample = [100*rand 100*rand];
    %reset obstacle flag
    obsFlag = false;
    %plot(sample(1),sample(2),'*k');
    if size(edges,1)==1
        closestNode = start;
    else
        %disp('-----')
        dist = zeros(size(edges,1),2);
        
        minNeighborDist = 10000;
        %loop through all the edges.
        for i = 1:size(edges,1)
            dist(i,1) = sqrt((edges(i,1)-sample(1)).^2 + (edges(i,2)-sample(2)).^2);
            dist(i,2) = sqrt((edges(i,3)-sample(1)).^2 + (edges(i,4)-sample(2)).^2);
            %grab nodes from this edge.
            node1 = edges(i,1:2);
            node2 = edges(i,3:4);
            
            %define line containing the edges end nodes and its normal
            line = [(node2(1)-node1(1)),(node2(2)-node1(2))];
            lineMag = sqrt(line(1)^2 +line(2)^2);
            n = line/lineMag;
            
            %%finds closest node on the line containing node1 and node2
            node = sample + (node1-sample)-(dot((node1-sample), n))*n;
            dotProduct = (sample(1)-node1(1))*(node2(1)-node1(1))+(sample(2)-node1(2))*(node2(2)-node1(2));
            squaredLength = (node2(1)-node1(1))*(node2(1)-node1(1))+(node2(2)-node1(2))*(node2(2)-node1(2));
            
            %checking if the closest node on the line is between two points
            if (dotProduct<0 || squaredLength<dotProduct)
                %if not between, check which node is closer.
                
                node1Dist = sqrt((node1(1)-sample(1)).^2 + (node1(2)-sample(2)).^2);
                node2Dist = sqrt((node2(1)-sample(1)).^2 + (node2(2)-sample(2)).^2);
                %if node1 closer node2 and less than the current minimum,
                %set to closest. check opposite as well. otherwise toss.
                if (node1Dist< node2Dist && node1Dist<minNeighborDist)
                    minNeighborDist = node1Dist;
                    nodeFlag = true;
                    closestNode = node1;
                elseif (node2Dist< node1Dist && node2Dist<minNeighborDist)
                    minNeighborDist = node2Dist;
                    nodeFlag = true;
                    closestNode = node2;
                end
            else
                %if it is between, check vs. the shortest length
                
                nodeDist = sqrt((node(1)-sample(1)).^2 + (node(2)-sample(2)).^2);
                %if node less than the current minimum, set to min and
                %closest node. repeats until all edges have been
                %traversed.
                if nodeDist<minNeighborDist
                    nodeFlag = false;
                    minNeighborDist = nodeDist;
                    closestNode = node;
                    closestNode1 = node1;
                    closestNode2 = node2;
                end
            end
        end

        e1 = [];
        e2 = [];
        %if node was between the two lines, define new edges, one for each segment.
        if nodeFlag ==false
            %adding edge from the new middle node to the edge nodes
            e1 =  cat(2, closestNode1,closestNode);
            e2 =  cat(2, closestNode2,closestNode);
        end

    end
    %now that closestNode on tree to sample has been found    
    %compute vector from closestNode to sample
    vec = [sample(1)-closestNode(1),sample(2)-closestNode(2)];
    vecMag = sqrt(vec(1)^2 + vec(2)^2);
    
    %Only travel the distance to the sample if it is closer than epsilon
    if vecMag < epsilon
        epsilon = vecMag;
    end
    
    %step increments of delta along potential new edge to collision check 
    for j = 0:delta:epsilon
        %multiply the unit vector going from current node to sample by delta.
        delVec = j*(vec./vecMag);
        
        %collision check at the location delta*j away from the node
        %in the direction of the sample
        delNode = closestNode+delVec;
        
        %obstacle check all obstacles 
        for i = 1:numObs
            currObs = obstacles_raw(i,:);
            
            distSquare = (delNode(1) - currObs(1)).^2 + (delNode(2) - currObs(2)).^2;
            rSquare = currObs(3)^2;

            if (distSquare < rSquare)
                %Check if inside obstacle or out of bounds, trigger obstacle flag
                obsFlag = true;
            elseif (delNode(1)>100)
                obsFlag = true;
            elseif (delNode(1)<0)
                obsFlag = true;
            elseif (delNode(2)>100)
                obsFlag = true;
            elseif (delNode(2)<0)
                obsFlag = true;
            end
        end
        %loop through and check again along the path until epsilon is met
    end
    
    %%goal check
    goalDistSquare = (delNode(1) - goal(1))^2 + (delNode(2) - goal(2))^2;
    rGoalSquare = goal(3)^2;
    if (goalDistSquare < rGoalSquare && obsFlag == false)
        % The object is inside of the goal,trip the goal flag and store for
        % plot
        goalNode = delNode
        goalFlag = true;
        goalEdge =  cat(2, closestNode,delNode);
        edges = cat(1,edges,goalEdge);
    end
    
    %if not goal, but also not obstacle, add edges and plot
    
    if obsFlag == false
        newEdge =  cat(2, closestNode,delNode);
        edges = cat(1,edges,newEdge);     
        edges = cat(1,edges,e1);
        edges = cat(1,edges,e2);
        plot([closestNode(1) delNode(1)],[closestNode(2) delNode(2)],'k','HandleVisibility','off');
    else
        %plot(sample(1),sample(2),'*r');
    end

    epsilon = 5;
end

plot(0,0,'k','DisplayName','Tree');
legend('Location','eastoutside')
legend('boxoff')
title('C')
hold off;

%% Problem 3
disp('--3--')
epsilon = 5
start = [60,60]
goal = [90 100 20]
goalFlag = false;
xGoal = goal(3)*cos(th(:)) + goal(1);
yGoal = goal(3)*sin(th(:)) + goal(2);


%plot the obstacles
figure(3), hold on
h = legend('show','location','best');
%plot Start
plot(start(1),start(2),'*m','DisplayName','Start')
%Plot goal region
plot(xGoal,yGoal,'g','DisplayName','Goal');

axis(C);
xlabel('x');
ylabel('y');
grid on;
axis square;

for i = 1:numObs
    plot(xunit(i,:), yunit(i,:),'r','HandleVisibility','off');
end
plot(0,0,'r','DisplayName','Obstacle');

e1 = [];
e2 = [];
edges = [100000 100000 100000 100000];
while goalFlag == false
    sample = [100*rand 100*rand];
    %reset obstacle flag
    obsFlag = false;
    %plot(sample(1),sample(2),'*k');
    if size(edges,1)==1
        closestNode = start;
    else
        %disp('-----')
        dist = zeros(size(edges,1),2);
        
        minNeighborDist = 10000;
        %loop through all the edges.
        for i = 1:size(edges,1)
            dist(i,1) = sqrt((edges(i,1)-sample(1)).^2 + (edges(i,2)-sample(2)).^2);
            dist(i,2) = sqrt((edges(i,3)-sample(1)).^2 + (edges(i,4)-sample(2)).^2);
            %grab nodes from this edge.
            node1 = edges(i,1:2);
            node2 = edges(i,3:4);
            
            %define line containing the edges end nodes and its normal
            line = [(node2(1)-node1(1)),(node2(2)-node1(2))];
            lineMag = sqrt(line(1)^2 +line(2)^2);
            n = line/lineMag;
            
            %%finds closest node on the line containing node1 and node2
            node = sample + (node1-sample)-(dot((node1-sample), n))*n;
            dotProduct = (sample(1)-node1(1))*(node2(1)-node1(1))+(sample(2)-node1(2))*(node2(2)-node1(2));
            squaredLength = (node2(1)-node1(1))*(node2(1)-node1(1))+(node2(2)-node1(2))*(node2(2)-node1(2));
            
            %checking if the closest node on the line is between two points
            if (dotProduct<0 || squaredLength<dotProduct)
                %if not between, check which node is closer.
                
                node1Dist = sqrt((node1(1)-sample(1)).^2 + (node1(2)-sample(2)).^2);
                node2Dist = sqrt((node2(1)-sample(1)).^2 + (node2(2)-sample(2)).^2);
                %if node1 closer node2 and less than the current minimum,
                %set to closest. check opposite as well. otherwise toss.
                if (node1Dist< node2Dist && node1Dist<minNeighborDist)
                    minNeighborDist = node1Dist;
                    nodeFlag = true;
                    closestNode = node1;
                elseif (node2Dist< node1Dist && node2Dist<minNeighborDist)
                    minNeighborDist = node2Dist;
                    nodeFlag = true;
                    closestNode = node2;
                end
            else
                %if it is between, check vs. the shortest length
                
                nodeDist = sqrt((node(1)-sample(1)).^2 + (node(2)-sample(2)).^2);
                %if node less than the current minimum, set to min and
                %closest node. repeats until all edges have been
                %traversed.
                if nodeDist<minNeighborDist
                    nodeFlag = false;
                    minNeighborDist = nodeDist;
                    closestNode = node;
                    closestNode1 = node1;
                    closestNode2 = node2;
                end
            end
        end

        e1 = [];
        e2 = [];
        %if node was between the two lines, define new edges, one for each segment.
        if nodeFlag ==false
            %adding edge from the new middle node to the edge nodes
            e1 =  cat(2, closestNode1,closestNode);
            e2 =  cat(2, closestNode2,closestNode);
        end

    end
    %now that closestNode on tree to sample has been found    
    %compute vector from closestNode to sample
    vec = [sample(1)-closestNode(1),sample(2)-closestNode(2)];
    vecMag = sqrt(vec(1)^2 + vec(2)^2);
    
    %Only travel the distance to the sample if it is closer than epsilon
    if vecMag < epsilon
        epsilon = vecMag;
    end
    
    %step increments of delta along potential new edge to collision check 
    for j = 0:delta:epsilon
        %multiply the unit vector going from current node to sample by delta.
        delVec = j*(vec./vecMag);
        
        %collision check at the location delta*j away from the node
        %in the direction of the sample
        delNode = closestNode+delVec;
        
        %obstacle check all obstacles 
        for i = 1:numObs
            currObs = obstacles_raw(i,:);
            
            distSquare = (delNode(1) - currObs(1)).^2 + (delNode(2) - currObs(2)).^2;
            rSquare = currObs(3)^2;

            if (distSquare < rSquare)
                %Check if inside obstacle or out of bounds, trigger obstacle flag
                obsFlag = true;
            elseif (delNode(1)>100)
                obsFlag = true;
            elseif (delNode(1)<0)
                obsFlag = true;
            elseif (delNode(2)>100)
                obsFlag = true;
            elseif (delNode(2)<0)
                obsFlag = true;
            end
        end
        %loop through and check again along the path until epsilon is met
    end
    
    %%goal check
    goalDistSquare = (delNode(1) - goal(1))^2 + (delNode(2) - goal(2))^2;
    rGoalSquare = goal(3)^2;
    if (goalDistSquare < rGoalSquare && obsFlag == false)
        % The object is inside of the goal,trip the goal flag and store for
        % plot
        goalNode = delNode
        goalFlag = true;
        goalEdge =  cat(2, closestNode,delNode);
        edges = cat(1,edges,goalEdge);
    end
    
    %if not goal, but also not obstacle, add edges and plot
    
    if obsFlag == false
        newEdge =  cat(2, closestNode,delNode);
        edges = cat(1,edges,newEdge);     
        edges = cat(1,edges,e1);
        edges = cat(1,edges,e2);
        plot([closestNode(1) delNode(1)],[closestNode(2) delNode(2)],'k','HandleVisibility','off');
    else
        %plot(sample(1),sample(2),'*r');
    end

    epsilon = 5;
end

plot(0,0,'k','DisplayName','Tree');
legend('Location','eastoutside')
legend('boxoff')
title('C')
hold off;

%% Problem 4

disp('--4--')
epsilon = 5
start = [60,80]
goal = [60 60 5]
goalFlag = false;
xGoal = goal(3)*cos(th(:)) + goal(1);
yGoal = goal(3)*sin(th(:)) + goal(2);

%plot the obstacles
figure(4), hold on
h = legend('show','location','best');
%plot Start
plot(start(1),start(2),'*m','DisplayName','Start')
%Plot goal region
plot(xGoal,yGoal,'g','DisplayName','Goal');

axis(C);
xlabel('x');
ylabel('y');
grid on;
axis square;

for i = 1:numObs
    plot(xunit(i,:), yunit(i,:),'r','HandleVisibility','off');
end
plot(0,0,'r','DisplayName','Obstacle');

e1 = [];
e2 = [];
edges = [100000 100000 100000 100000];
while goalFlag == false
    sample = [100*rand 100*rand];
    %reset obstacle flag
    obsFlag = false;
    %plot(sample(1),sample(2),'*k');
    if size(edges,1)==1
        closestNode = start;
    else
        %disp('-----')
        dist = zeros(size(edges,1),2);
        
        minNeighborDist = 10000;
        %loop through all the edges.
        for i = 1:size(edges,1)
            dist(i,1) = sqrt((edges(i,1)-sample(1)).^2 + (edges(i,2)-sample(2)).^2);
            dist(i,2) = sqrt((edges(i,3)-sample(1)).^2 + (edges(i,4)-sample(2)).^2);
            %grab nodes from this edge.
            node1 = edges(i,1:2);
            node2 = edges(i,3:4);
            
            %define line containing the edges end nodes and its normal
            line = [(node2(1)-node1(1)),(node2(2)-node1(2))];
            lineMag = sqrt(line(1)^2 +line(2)^2);
            n = line/lineMag;
            
            %%finds closest node on the line containing node1 and node2
            node = sample + (node1-sample)-(dot((node1-sample), n))*n;
            dotProduct = (sample(1)-node1(1))*(node2(1)-node1(1))+(sample(2)-node1(2))*(node2(2)-node1(2));
            squaredLength = (node2(1)-node1(1))*(node2(1)-node1(1))+(node2(2)-node1(2))*(node2(2)-node1(2));
            
            %checking if the closest node on the line is between two points
            if (dotProduct<0 || squaredLength<dotProduct)
                %if not between, check which node is closer.
                
                node1Dist = sqrt((node1(1)-sample(1)).^2 + (node1(2)-sample(2)).^2);
                node2Dist = sqrt((node2(1)-sample(1)).^2 + (node2(2)-sample(2)).^2);
                %if node1 closer node2 and less than the current minimum,
                %set to closest. check opposite as well. otherwise toss.
                if (node1Dist< node2Dist && node1Dist<minNeighborDist)
                    minNeighborDist = node1Dist;
                    nodeFlag = true;
                    closestNode = node1;
                elseif (node2Dist< node1Dist && node2Dist<minNeighborDist)
                    minNeighborDist = node2Dist;
                    nodeFlag = true;
                    closestNode = node2;
                end
            else
                %if it is between, check vs. the shortest length
                
                nodeDist = sqrt((node(1)-sample(1)).^2 + (node(2)-sample(2)).^2);
                %if node less than the current minimum, set to min and
                %closest node. repeats until all edges have been
                %traversed.
                if nodeDist<minNeighborDist
                    nodeFlag = false;
                    minNeighborDist = nodeDist;
                    closestNode = node;
                    closestNode1 = node1;
                    closestNode2 = node2;
                end
            end
        end

        e1 = [];
        e2 = [];
        %if node was between the two lines, define new edges, one for each segment.
        if nodeFlag ==false
            %adding edge from the new middle node to the edge nodes
            e1 =  cat(2, closestNode1,closestNode);
            e2 =  cat(2, closestNode2,closestNode);
        end

    end
    %now that closestNode on tree to sample has been found    
    %compute vector from closestNode to sample
    vec = [sample(1)-closestNode(1),sample(2)-closestNode(2)];
    vecMag = sqrt(vec(1)^2 + vec(2)^2);
    
    %Only travel the distance to the sample if it is closer than epsilon
    if vecMag < epsilon
        epsilon = vecMag;
    end
    
    %step increments of delta along potential new edge to collision check 
    for j = 0:delta:epsilon
        %multiply the unit vector going from current node to sample by delta.
        delVec = j*(vec./vecMag);
        
        %collision check at the location delta*j away from the node
        %in the direction of the sample
        delNode = closestNode+delVec;
        
        %obstacle check all obstacles 
        for i = 1:numObs
            currObs = obstacles_raw(i,:);
            
            distSquare = (delNode(1) - currObs(1)).^2 + (delNode(2) - currObs(2)).^2;
            rSquare = currObs(3)^2;

            if (distSquare < rSquare)
                %Check if inside obstacle or out of bounds, trigger obstacle flag
                obsFlag = true;
            elseif (delNode(1)>100)
                obsFlag = true;
            elseif (delNode(1)<0)
                obsFlag = true;
            elseif (delNode(2)>100)
                obsFlag = true;
            elseif (delNode(2)<0)
                obsFlag = true;
            end
        end
        %loop through and check again along the path until epsilon is met
    end
    
    %%goal check
    goalDistSquare = (delNode(1) - goal(1))^2 + (delNode(2) - goal(2))^2;
    rGoalSquare = goal(3)^2;
    if (goalDistSquare < rGoalSquare && obsFlag == false)
        % The object is inside of the goal,trip the goal flag and store for
        % plot
        goalNode = delNode
        goalFlag = true;
        goalEdge =  cat(2, closestNode,delNode);
        edges = cat(1,edges,goalEdge);
    end
    
    %if not goal, but also not obstacle, add edges and plot
    
    if obsFlag == false
        newEdge =  cat(2, closestNode,delNode);
        edges = cat(1,edges,newEdge);     
        edges = cat(1,edges,e1);
        edges = cat(1,edges,e2);
        plot([closestNode(1) delNode(1)],[closestNode(2) delNode(2)],'k','HandleVisibility','off');
    else
        %plot(sample(1),sample(2),'*r');
    end

    epsilon = 5;
end

plot(0,0,'k','DisplayName','Tree');
legend('Location','eastoutside')
legend('boxoff')
title('C')
hold off;


%% Problem 5
disp('--5--')
epsilon = 1
start = [1,99]
goal = [100 0 20]
goalFlag = false;

xGoal = goal(3)*cos(th(:)) + goal(1);
yGoal = goal(3)*sin(th(:)) + goal(2);

%plot the obstacles
figure(5), hold on
h = legend('show','location','best');
%plot Start
plot(start(1),start(2),'*m','DisplayName','Start')
%Plot goal region
plot(xGoal,yGoal,'g','DisplayName','Goal');

axis(C);
xlabel('x');
ylabel('y');
grid on;
axis square;

for i = 1:numObs
    plot(xunit(i,:), yunit(i,:),'r','HandleVisibility','off');
end
plot(0,0,'r','DisplayName','Obstacle');

e1 = [];
e2 = [];
edges = [100000 100000 100000 100000];
while goalFlag == false
    sample = [100*rand 100*rand];
    %reset obstacle flag
    obsFlag = false;
    %plot(sample(1),sample(2),'*k');
    if size(edges,1)==1
        closestNode = start;
    else
        %disp('-----')
        dist = zeros(size(edges,1),2);
        
        minNeighborDist = 10000;
        %loop through all the edges.
        for i = 1:size(edges,1)
            dist(i,1) = sqrt((edges(i,1)-sample(1)).^2 + (edges(i,2)-sample(2)).^2);
            dist(i,2) = sqrt((edges(i,3)-sample(1)).^2 + (edges(i,4)-sample(2)).^2);
            %grab nodes from this edge.
            node1 = edges(i,1:2);
            node2 = edges(i,3:4);
            
            %define line containing the edges end nodes and its normal
            line = [(node2(1)-node1(1)),(node2(2)-node1(2))];
            lineMag = sqrt(line(1)^2 +line(2)^2);
            n = line/lineMag;
            
            %%finds closest node on the line containing node1 and node2
            node = sample + (node1-sample)-(dot((node1-sample), n))*n;
            dotProduct = (sample(1)-node1(1))*(node2(1)-node1(1))+(sample(2)-node1(2))*(node2(2)-node1(2));
            squaredLength = (node2(1)-node1(1))*(node2(1)-node1(1))+(node2(2)-node1(2))*(node2(2)-node1(2));
            
            %checking if the closest node on the line is between two points
            if (dotProduct<0 || squaredLength<dotProduct)
                %if not between, check which node is closer.
                
                node1Dist = sqrt((node1(1)-sample(1)).^2 + (node1(2)-sample(2)).^2);
                node2Dist = sqrt((node2(1)-sample(1)).^2 + (node2(2)-sample(2)).^2);
                %if node1 closer node2 and less than the current minimum,
                %set to closest. check opposite as well. otherwise toss.
                if (node1Dist< node2Dist && node1Dist<minNeighborDist)
                    minNeighborDist = node1Dist;
                    nodeFlag = true;
                    closestNode = node1;
                elseif (node2Dist< node1Dist && node2Dist<minNeighborDist)
                    minNeighborDist = node2Dist;
                    nodeFlag = true;
                    closestNode = node2;
                end
            else
                %if it is between, check vs. the shortest length
                
                nodeDist = sqrt((node(1)-sample(1)).^2 + (node(2)-sample(2)).^2);
                %if node less than the current minimum, set to min and
                %closest node. repeats until all edges have been
                %traversed.
                if nodeDist<minNeighborDist
                    nodeFlag = false;
                    minNeighborDist = nodeDist;
                    closestNode = node;
                    closestNode1 = node1;
                    closestNode2 = node2;
                end
            end
        end

        e1 = [];
        e2 = [];
        %if node was between the two lines, define new edges, one for each segment.
        if nodeFlag ==false
            %adding edge from the new middle node to the edge nodes
            e1 =  cat(2, closestNode1,closestNode);
            e2 =  cat(2, closestNode2,closestNode);
        end

    end
    %now that closestNode on tree to sample has been found    
    %compute vector from closestNode to sample
    vec = [sample(1)-closestNode(1),sample(2)-closestNode(2)];
    vecMag = sqrt(vec(1)^2 + vec(2)^2);
    
    %Only travel the distance to the sample if it is closer than epsilon
    if vecMag < epsilon
        epsilon = vecMag;
    end
    
    %step increments of delta along potential new edge to collision check 
    for j = 0:delta:epsilon
        %multiply the unit vector going from current node to sample by delta.
        delVec = j*(vec./vecMag);
        
        %collision check at the location delta*j away from the node
        %in the direction of the sample
        delNode = closestNode+delVec;
        
        %obstacle check all obstacles 
        for i = 1:numObs
            currObs = obstacles_raw(i,:);
            
            distSquare = (delNode(1) - currObs(1)).^2 + (delNode(2) - currObs(2)).^2;
            rSquare = currObs(3)^2;

            if (distSquare < rSquare)
                %Check if inside obstacle or out of bounds, trigger obstacle flag
                obsFlag = true;
            elseif (delNode(1)>100)
                obsFlag = true;
            elseif (delNode(1)<0)
                obsFlag = true;
            elseif (delNode(2)>100)
                obsFlag = true;
            elseif (delNode(2)<0)
                obsFlag = true;
            end
        end
        %loop through and check again along the path until epsilon is met
    end
    
    %%goal check
    goalDistSquare = (delNode(1) - goal(1))^2 + (delNode(2) - goal(2))^2;
    rGoalSquare = goal(3)^2;
    if (goalDistSquare < rGoalSquare && obsFlag == false)
        % The object is inside of the goal,trip the goal flag and store for
        % plot
        goalNode = delNode
        goalFlag = true;
        goalEdge =  cat(2, closestNode,delNode);
        edges = cat(1,edges,goalEdge);
    end
    
    %if not goal, but also not obstacle, add edges and plot
    
    if obsFlag == false
        newEdge =  cat(2, closestNode,delNode);
        edges = cat(1,edges,newEdge);     
        edges = cat(1,edges,e1);
        edges = cat(1,edges,e2);
        plot([closestNode(1) delNode(1)],[closestNode(2) delNode(2)],'k','HandleVisibility','off');
    else
        %plot(sample(1),sample(2),'*r');
    end

    epsilon = 1;
end

plot(0,0,'k','DisplayName','Tree');
legend('Location','eastoutside')
legend('boxoff')
title('C')
hold off;



