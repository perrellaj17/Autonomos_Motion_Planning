/*
Copyright 2018, Michael Otte

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <sys/types.h>
#include <time.h>
#include <sys/time.h>
#include <math.h>


#include "heap.h"
#include "heap.cpp"

#include "graph.h"


// returns random number between 0 and 1
float rand_f()
{
  return (float)rand() / (float)RAND_MAX;
}

// returns a random number between low and high (inclusive)
int rand_d_between(int low, int high)
{
  int span = high - low;
  int r = (int)(rand_f() * (float)(span + 1));
  if(r > span)
  {
    // very rare, but can technically happen 
    r = span;
  }

  return r + low;
}

int main()
{
  srand(time(NULL)); // seed random number generator

  Graph G;
  G.readGraphFromFiles("nodes.txt", "edges.txt");
  G.printGraph();


  Heap<Node> H(100); // this is the heap (start's with space for 100 items
                     // but will grow automatically as needed).

  // do some heap opperations, we'll convert node id's to floats and
  // use those as the heap key (this is only for demenstration, usually
  // we'd have some key based on graph cost or something else).


  H.addToHeap(&G.nodes[75], 75.0);
  H.addToHeap(&G.nodes[80], 80.0);
  H.addToHeap(&G.nodes[40], 40.0);

  H.updateNodeInHeap(&G.nodes[75], 75.2);
  H.updateNodeInHeap(&G.nodes[80], 80.05);

  H.addToHeap(&G.nodes[60], 60.0);
  H.addToHeap(&G.nodes[5], 5.0);
  H.addToHeap(&G.nodes[95], 95.0);

  H.updateNodeInHeap(&G.nodes[5], 5.3);

  H.addToHeap(&G.nodes[97], 97.0);
  H.addToHeap(&G.nodes[54], 54.0);

  H.printHeap();

  // look at the top
  Node* topNode = H.topHeap();
  printf("top node: %d\n", topNode->id);


  // now pop the top 3 things
  for(int i = 0; i < 3; i++)
  {
    Node* popedNode = H.popHeap();
    printf("popped node: %d\n", popedNode->id);
  }

  // print the heap
  H.printHeap();


  // debugging check to see if heap is ok (don't use in practice)
  H.checkHeap();

  // insert some other stuff and update some stuff
  H.addToHeap(&G.nodes[83], 83.0);
  H.addToHeap(&G.nodes[41], 41.0);
  H.addToHeap(&G.nodes[69], 69.0);

  H.updateNodeInHeap(&G.nodes[41], 41.5);
  H.updateNodeInHeap(&G.nodes[69], 69.2);

  H.addToHeap(&G.nodes[57], 57.0);
  H.addToHeap(&G.nodes[1], 1.0);
  H.addToHeap(&G.nodes[2], 2.0);

  H.updateNodeInHeap(&G.nodes[41], 41.11);
  H.updateNodeInHeap(&G.nodes[1], 1.12);

  H.addToHeap(&G.nodes[98], 98.0);
  H.addToHeap(&G.nodes[20], 20.0);


  // debugging check to see if heap is ok (don't use in practice)
  H.checkHeap();

  // print the heap
  H.printHeap();

  // now pop everything off
  while(H.topHeap() != NULL)
  {
    Node* popedNode = H.popHeap();
    printf("popped node: %d\n", popedNode->id);
  }


// NOTE: I've commented out the following, which is a huge test that 
// things are working in the heap that uses random insertians,
// removals, and updates
//
//  // now we'll change the keys a bunch, and add and
//  // remove a bunch of things randomly (for error checking mostly)
//  for(int i = 0; i < 1000; i++)
//  {
//    float randomNumber = rand_f();
//    printf("random number: %f\n", randomNumber);
// 
//    if(randomNumber < .25)
//    {
//      // with 1/4 probability we do this (Assuming there are nodes in the heap)
//
//      // pick a random node, if it is already in the heap, then remove it
//      int r = rand_d_between(0, G.numNodes-1);
//      if(G.nodes[r].inHeap)
//      {
//        printf("removing node %d from heap (from heap position %d)\n", r, G.nodes[r].heapIndex);
//        H.removeNodeFromHeap(&G.nodes[r]);
//      }
//    }
//    else if(randomNumber < 0.5)
//    {
//      // with 1/4 probability we do this
//     
//      // pick a random node (may or may not be in the heap)
//
//      int r = rand_d_between(0, G.numNodes-1);
//      float newKey = rand_f()*1000.0;
//
//      printf("updating node %d from heap position %d to have new key %f\n", r, G.nodes[r].heapIndex, newKey);
//
//      H.updateNodeInHeap(&G.nodes[r], newKey);
//
//    }
//    else if(randomNumber < 0.75)
//    {
//      // with 1/4 probability we do this
//
//      // pick a random node, if it is not in the heap then add it
//
//      int r = rand_d_between(0, G.numNodes-1);
//      if(!G.nodes[r].inHeap)
//      {
//        float newKey = rand_f()*1000.0;     
//        printf("adding node %d to heap with key %f\n", r, newKey);
//
//        H.addToHeap(&G.nodes[r], newKey);
//      }
//    }
//    else
//    {
//      // with remaining 1/3 probability we do this
//
//      // pop the heap
//      if(H.topHeap() != NULL)
//      {
//        Node* popedNode = H.popHeap();
//        printf("popped node: %d\n", popedNode->id);
//      }
//    }
//
//    // debugging check to see if heap is ok (don't use in practice)
//    H.checkHeap();
//
//  }
//
//  H.printHeap();
//
//  // finally, we'll set they keys back to the ids, resorting the heap approperiatly
//  for(int i = 1; i < G.numNodes-1; i++)
//  {
//    if(G.nodes[i].inHeap)
//    {
//
//      H.updateNodeInHeap(&G.nodes[i], (double)G.nodes[i].id);
//    }
//  }
//
//
//  // debugging check to see if heap is ok (don't use in practice)
//  H.checkHeap();
//
//  H.printHeap();
//
//  // now pop everything off
//  while(H.topHeap() != NULL)
//  {
//    Node* popedNode = H.popHeap();
//    printf("popped node: %d\n", popedNode->id);
//  }


  return 0;
}
