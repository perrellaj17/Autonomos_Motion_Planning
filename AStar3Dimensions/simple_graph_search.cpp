/*
 Copyright 2018, Michael Otte
 
 Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
 
 The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
 
 THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 
 MODIFIED BY JOE PERRELLA Feb 2020- Implement AStar algorithm
 
 
 
 QUEUE ref: Min priority heap
 Michael Otte, University of Colorado, 9-22-08
 Copyright 2008 and 2018, Michael Otte
 
 
 */


#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <sys/types.h>
#include <time.h>
#include <sys/time.h>
#include <math.h>
#include <fstream>

#include "heap.h"
#include "heap.cpp"

#include "graph.h"




void expand(Heap<Node> &H, Node* thisNode, Node* goalNode)
{
    // THISNODE IS V
    // U IS NEIGHBORNODE
    // 0 = not visited yet
    // 1 = in open list (i.e., in heap)
    // 2 = in closed list
    
    //while v.NextNeighbor() /= Null
    for(int n = 0; n < thisNode->numOutgoingEdges; n++)
    {
        
        //u <- v.NextNeighbor
        Edge* thisEdge = thisNode->outgoingEdges[n];  // pointer to this edge
        Node* neighborNode = thisEdge->endNode;  // pointer to the node on the other end of this edge
        
        float heuristic = sqrt(pow((goalNode->x - neighborNode->x), 2) + pow((goalNode->y - neighborNode->y), 2) + pow((goalNode->z - neighborNode->z), 2));
        
        // if the neighbor has not yet been visited, add it to the heap with its cost to start plus heuristic
        
        if(neighborNode->status == 0)
        {
            
            //u.costToStart = v.costToStart + Cost(u,v);
            double neighborKey = thisNode->costToStart + thisEdge->edgeCost;
            neighborNode->costToStart = neighborKey;
            
            H.addToHeap(neighborNode, neighborKey + heuristic);
            
            //set the neighbors parent to this node, because this node is lower cost
            neighborNode->parentNode = thisNode;
            // put neighbor Node in (visited) list
            neighborNode->status = 1;
        }
        //if the neighbor has been visited, but isnt closed and has a higher cost to start than it could
        else if (neighborNode->status == 1 && neighborNode->costToStart > (thisNode->costToStart + thisEdge->edgeCost))
        {
            
            //set the neighbors parent to this node, because this node is lower cost
            neighborNode->parentNode = thisNode;
            
            //store cost to start of neighbor
            //u.costToStart = v.costToStart + Cost(u,v);
            double neighborKey = thisNode->costToStart + thisEdge->edgeCost;
            neighborNode->costToStart = neighborKey;
            
            H.updateNodeInHeap(neighborNode, neighborKey + heuristic);
            
            
        }
    }
    
    thisNode->status = 2;    // now this node is in the closed list
}



int main()
{
    
     clock_t start = clock();("////////////////////////////////////////////////////////////////////////////////////\nRunning A* \n////////////////////////////////////////////////////////////////////////////////////\n");
    
    
    
    
        //initialize goalCostToStart high so that it will be automatically overridden
    double goalCostToStart = 10000000;
    
    
    int goalNodeIndex = 1677;
    
    //int goalNodeIndex = 1990; //top middle node
    //int goalNodeIndex = 2210; //top middle top right node
    //int goalNodeIndex = 2188; //top middle top left node
    //int goalNodeIndex = 1792; //top middle bottom right node
    //int goalNodeIndex = 1770; //top middle bottom leftnode
    
    //CHANGE THIS LATER TO TAKE AN INPUT FROM A TEXT FILE WITH THE NODE ID'S
    int startNodeArray [4] = {999, 998, 997, 996};
    
    //i limit is the numver of start nodes in startNodeArray -1
    //cycle through all of the potential start nodes
    for(int i = 0; i < 4; i++)
    {
        
        //initialize the graph representation of the 3D cartesian workspace
        //need to do this every time in order to reset the nodeStatus values
        Graph G;
        G.readGraphFromFiles("nodes.txt", "edges.txt");
        
        Heap<Node> H(100); //initialize new heap with 100 items but can grow
        // these are pointers to the start and goal nodes
        
        int startNodeIndex = startNodeArray[i];
        
        printf("Index:  %d \n",i);
        printf("Start Node:  %d ",startNodeIndex);

        

        Node* startNode = &G.nodes[startNodeIndex];
        Node* goalNode = &G.nodes[goalNodeIndex];
        
        printf("(%f, ",startNode->x);
        printf("%f, ",startNode->y);
        printf("%f) \n",startNode->z);
        
        
        
        startNode->costToStart = 0;
        H.addToHeap(startNode, startNode->costToStart); //we then add the start node to the heap
        startNode->status = 1;    // now the start node is in the open list
        
        
        // while there are nodes left in the heap and the goal hasnt been reached, search
        
        while(H.topHeap() != NULL)
        {
            
            Node* thisNode = H.popHeap();
            
            if (thisNode == goalNode)
            {
                //This is where we do path smoothing. Then we will know the actual smooth path goalCostToStart
                
                
                
                
                printf("The current goalCostStoStart is  %f \n",goalCostToStart);
                printf("The cost to start of this path is  %f \n",thisNode->costToStart);
                //get the cost to start of the current path once the goal Node has been reached
                
                // NEED TO RESET ALL THE NODES VALUES
                
                if(goalCostToStart > thisNode->costToStart)
                {
                    //reset goalCostToStart to this one
                    goalCostToStart = thisNode->costToStart;
                    
                    printf("The newest lowest cost is  %f \n",goalCostToStart);
                    
                    
                    //Clear contents of output_path and search_tree
                    std::ofstream ofs;
                    ofs.open("output_path.txt", std::ofstream::out | std::ofstream::trunc);
                    ofs.close();
                    
                    ofs.open("search_tree.txt", std::ofstream::out | std::ofstream::trunc);
                    ofs.close();
                    
                    //save
                    G.savePathToFile("output_path.txt", goalNode);
                    G.saveSearchTreeToFile("search_tree.txt");

                }
                G.deleteGraph();
                break;
            }
            else
            {
                expand(H, thisNode, goalNode);
                //want to store the cost to start of the current path
            }
        }
        
        
        //return 0;
    } //end cycle through the startNodeArray
    
    
    clock_t stop = clock();
    
    
        double elapsed = (double)(stop - start) * 1000.0 / CLOCKS_PER_SEC;

    
    printf("Time Duration: %f milliseconds",elapsed);
    
    
    
}
