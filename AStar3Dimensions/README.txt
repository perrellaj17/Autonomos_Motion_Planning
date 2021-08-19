Michael Otte (2018)

This is a mod of my original heap and graph search code from 2008.

Matlab functions are used to generate random graphs.

C++ code is used to do graph search (and save files containing
the resulting path and the search tree)

Matlab code is then used to display the graph and shortest path
data.

I've also included one set of output files.

-------------------------------------------------------------------
Here's how it works:

1) Move the nodes_#.txt and edges_#.txt to the Graph Search folder. Rename them as nodes.txt and edges.txt.

2) In C++ (in linux compile and then run as follows on the command line):

   g++ simple_graph_search.cpp -o run_simple_graph_search

   ./run_simple_graph_search

   The first command will compile the code, the second command will
   run a (naieve random walk generation of a spanning tree) graph search 
   and output the files output_path.txt and search_tree.txt

3) in matlab run the code in display_search.m to see the output saved
   in the files output_path.txt and search_tree.txt

-------------------------------------------------------------------------

Other code included is a testing/debugging function for the heap itself which
can be run as follows:

g++ heap_test.cpp -o heap_test

./heap_test

