% Copyright 2018, Michael Otte
%
% Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
%
%The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
%
%THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
%
% this will display the search tree and path
% assuming that the files have been generated

%MODIFIED JOE PERRELLA 2020

clf, clc, clear
search_tree_raw = csvread('search_tree.txt');
path_raw = csvread('output_path.txt');
nodes_raw = dlmread('nodes.txt');
edges_raw = dlmread('edges.txt');



%% a bit of data processing for faster plotting
search_tree = nan(3*size(search_tree_raw, 1), 3);

%x1
search_tree(1:3:end-2, 1) = search_tree_raw(:, 2);
%x2
search_tree(2:3:end-1, 1) = search_tree_raw(:, 6);
%y1
search_tree(1:3:end-2, 2) = search_tree_raw(:, 3);
%y2
search_tree(2:3:end-1, 2) = search_tree_raw(:, 7);
%z1
search_tree(1:3:end-2, 3) = search_tree_raw(:, 4);
%z2
search_tree(2:3:end-1, 3) = search_tree_raw(:, 8);


%take the x y and z from nodes_raw
nodes = nodes_raw(2:end,2:4);
%remove first row from edges raw
edges_raw = edges_raw(2:end,:);

edges = nan(3*size(edges_raw, 1), 2);

%x1
edges(1:3:end-2, 1) = nodes(edges_raw(:, 1),1);
%x2
edges(2:3:end-1, 1) = nodes(edges_raw(:, 2),1);
%y1
edges(1:3:end-2, 2) = nodes(edges_raw(:, 1),2);
%y2
edges(2:3:end-1, 2) = nodes(edges_raw(:, 2),2);
%z1
edges(1:3:end-2, 3) = nodes(edges_raw(:, 1),3);
%z2
edges(2:3:end-1, 3) = nodes(edges_raw(:, 2),3);

%% Path Smoothing

P = path_raw;

%need to add outer loop to smooth until no changes occur.
removalFlag = true;
delta = 0.1;


%WHAT NEEDS TO BE DONE: UPLOAD THE OBSTACLES INTO THIS AND TEST. PRINT THE
%PRE SMOOTHED AND AFTER SMOOTHED PATHS. 


numObs = 0;

goalIndex = size(path_raw,1);
goal = path_raw(goalIndex,2:4);
start = path_raw(1,2:4);
potentialstart = [nodes(1000,:); nodes(999,:); nodes(998,:); nodes(997,:)];


plot3(nodes(:,1), nodes(:,2), nodes(:,3), '.k')
hold on

plot3(potentialstart(:,1), potentialstart(:,2), potentialstart(:,3), '*r')


plot3(P(:,2), P(:,3), P(:,4), 'b', 'LineWidth', 3);
plot3(goal(1),goal(2),goal(3), '.','MarkerEdgeColor','#079D1A','MarkerSize',25,'MarkerFaceColor','g')
plot3(start(1),start(2),start(3), '.','MarkerEdgeColor','#0C079D','MarkerSize',25,'MarkerFaceColor','c')
%plot3(edges(:,1), edges(:,2), edges(:,3), 'k')
%PLOT THE POTENTIAL START LOCATIONS THAT WERE CHOSEN FROM


xlabel('X')
ylabel('Y')
zlabel('Z')

grid on
x = 1
%if we removed a node last time and still need to try to smooth more
while ((removalFlag == true) &&  (size(P,1)>2))
    
    
    %init removal flag as false for this iteration of Path P
    removalFlag = false;
    obsFlag = false;
    sizeP = size(P,1)-2
    i = 1
    while i < sizeP
        
        P(:,2:4);
        S = P(i,2:4) %start node
        %if (i+2)>
        A = P(i+2,2:4) %node 2 edges Ahead
        
        plot3(S(1), S(2), S(3), '.g','MarkerSize',25)
        plot3(A(1), A(2), A(3), '.r','MarkerSize',25)
        
        rMag = sqrt((A(1)-S(1))^2 +(A(2)-S(2))^2 + (A(3)-S(3))^2);
        %     theta = atan2((A(2)-S(2)),(A(1)-S(1)));
        %     phi = acos((A(3)-S(3)/rMag);
        
        %compute vector from start node to ahead node
        rVec = [A(1)-S(1),A(2)-S(2), A(3)-S(3)];
        
        %step increments of delta along potential new edge to collision check
        for j = delta:delta:rMag
            
            %multiply the unit vector going from current node to sample by delta.
            delVec = j*(rVec./rMag);
            
            %collision check at the location delta*j away from the start node
            %toward the ahead node
            delNode = S+delVec;
            
            if x > 1
            
                plot3(delNode(1), delNode(2), delNode(3), '.m','MarkerSize',10)
            else
                plot3(delNode(1), delNode(2), delNode(3), '.b','MarkerSize',10)
            end
            %CAN JUST WATCH IT RUN THROUGH THE WHOLE GRAPH WITHOUT
            %DOING OBSTACLE CHECKING FIRST
            
            %                 %obstacle check all obstacles
            %                 for i = 1:numObs
            %                     currObs = obstacles_raw(i,:);
            %                     distSquare = (delNode(1) - currObs(1)).^2 + (delNode(2) - currObs(2)).^2+ (delNode(3) - currObs(3)).^2;
            %                     rSquare = currObs(3)^2;
            %                     if (distSquare < rSquare)
            %                         %Check if inside obstacle trigger obstacle flag
            %                         obsFlag = true;
            %
            %                     end
            %                 end
            
        end
        %If we travel to the end of rMag without triggering obstacle flag
        if obsFlag == false
            %remove intermediary waste node
            P([i+1],2:4)
            P([i+1],:) = [];
            
            sizeP = size(P,1)-2
            %trigger removal flag
            removalFlag = true
            %reset the obstacle flag for next cycle of path
            obsFlag = false;
        end
        
        i = i+1;
        
    end
    x = 2
    
    %if the removal flag was not triggered after that pass through the
    %entire Path, then the Path was never updated, and this loop will end
end


%Make sure to write the smoothed path to the output_path.txt document
%writematrix(P, 'output_path.txt');

%% Plotting


%plot3(edges(:,1), edges(:,2), edges(:,3), 'k')
%plot3(search_tree(:, 1), search_tree(:, 2), search_tree(:, 3), 'r', 'LineWidth', 1.5);


hold off




