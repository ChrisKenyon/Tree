#pragma once
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/subgraph.hpp>
#include <stack>
#include <queue>
#include <fstream>

using namespace boost;
using namespace std;

#define LargeValue 99999999
struct VertexProperties;
struct EdgeProperties;
typedef adjacency_list<vecS, vecS, bidirectionalS, VertexProperties, EdgeProperties> Graph;

struct VertexProperties
{
	int forestTreeNumber;
	Graph::vertex_descriptor pred; // predecessor node
	int weight;
	bool visited;
	bool marked;
};

// Create a struct to hold properties for each edge
struct EdgeProperties
{
	int weight;
	bool visited;
	bool marked;
};

void initializeGraph(Graph &g, Graph::vertex_descriptor &start, Graph::vertex_descriptor &end, ifstream &fin);
void findMinSpanningTree(Graph::vertex_descriptor u, Graph &g, Graph &tree);
void findMinSpanningForest(Graph &g, Graph &sf);
bool isConnected(Graph &g);
bool isCyclic(Graph &g);

void clearVisited(Graph &g);
void setNodeWeights(Graph &g, int w);
void clearMarked(Graph &g);

bool relax(Graph &g, Graph::vertex_descriptor u, Graph::vertex_descriptor v);
bool Dijkstra(Graph &g, Graph::vertex_descriptor start);
void printPaths(Graph &g, pair<Graph::vertex_iterator, Graph::vertex_iterator> vertItrRange, Graph::vertex_descriptor sourceNode);

stack<Graph::vertex_descriptor> getShortestPathFromBForD(Graph &g, Graph::vertex_descriptor target);

ostream& operator << (ostream &ostr, const Graph &g);