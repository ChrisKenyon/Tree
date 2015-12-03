#include "stdafx.h"
#include "graph.h"
#include "heapV.h"

using namespace std;
using namespace boost;

void findMinSpanningTree(Graph::vertex_descriptor start, Graph &g, Graph &tree)
{
	Dijkstra(g, start);
	map<size_t, size_t> treeVertexVal;
	pair<Graph::vertex_iterator, Graph::vertex_iterator> vertIter = vertices(g);
	for (; vertIter.first != vertIter.second; ++vertIter.first)
	{
		VertexProperties vert = g[*vertIter.first];
		if (vert.weight != LargeValue)
		{
			treeVertexVal[*vertIter.first] = add_vertex(vert, tree);
		}
	}

	Graph::edge_descriptor e;
	Graph::vertex_descriptor u = start;
	Graph::vertex_descriptor v;
	g[u].visited = true;
	int minWeight = LargeValue;
	size_t edgeSource, edgeTarget;
	vector<Graph::vertex_descriptor> visited;
	visited.push_back(u);

	while (visited.size() < num_vertices(tree))
	{
		minWeight = LargeValue;
		for (int i = 0; i < visited.size(); i++)
		{
			u = visited[i];

			pair<Graph::adjacency_iterator, Graph::adjacency_iterator> neighborIter = adjacent_vertices(u, g);
			for (; neighborIter.first != neighborIter.second; ++neighborIter.first)
			{
				v = *neighborIter.first;
				if (!g[v].visited) {
					e = edge(u, v, g).first;
					if (g[e].weight < minWeight)
					{
						minWeight = g[e].weight;
						edgeSource = u;
						edgeTarget = v;
					}
				}
			}
		}
		if (minWeight < LargeValue)
		{
			EdgeProperties newEdgeProp;
			newEdgeProp.weight = minWeight;
			add_edge(treeVertexVal[edgeSource], treeVertexVal[edgeTarget], newEdgeProp, tree);
			g[edgeTarget].visited = true;
			visited.push_back(edgeTarget);
		}
		else if (visited.size() != 1)
			throw new std::exception("Less edges found than expected");
	}
}

void findMinSpanningForest(Graph &g, Graph &sf)
{
	clearVisited(g);
	clearMarked(g);

	Graph::vertex_descriptor start = *vertices(g).first;

	int treeNum = 1;
	bool isUnvisitedNode = true;
	while (isUnvisitedNode)
	{
		g[start].visited = true;
		Graph tree;
		findMinSpanningTree(start, g, tree);

		Graph::vertex_descriptor startSFvertex = *vertices(sf).second;
		pair<Graph::vertex_iterator, Graph::vertex_iterator> vertIter = vertices(tree);
		for (; vertIter.first != vertIter.second; ++vertIter.first)
		{
			tree[*vertIter.first].forestTreeNumber = treeNum;
			add_vertex(tree[*vertIter.first], sf);
		}

		pair<Graph::edge_iterator, Graph::edge_iterator> edgeIter = edges(tree);
		for (; edgeIter.first != edgeIter.second; ++edgeIter.first)
		{
			Graph::edge_descriptor edge = *edgeIter.first;
			EdgeProperties newEdgeProp;
			newEdgeProp.weight = tree[edge].weight;

			Graph::vertex_descriptor v1 = source(edge, tree) + startSFvertex;
			Graph::vertex_descriptor v2 = target(edge, tree) + startSFvertex;
			add_edge(v1, v2, newEdgeProp, sf);
		}

		isUnvisitedNode = false;
		vertIter = vertices(g);
		for (; vertIter.first != vertIter.second; ++vertIter.first)
		{
			if (!g[*vertIter.first].visited)
			{
				isUnvisitedNode = true;
				start = *vertIter.first;
				treeNum++;
				break;
			}
		}
	}
}

bool isConnected(Graph &g)
{
	return Dijkstra(g, vertex(0, g));
}

bool isCyclic(Graph &g)
{
	stack <Graph::vertex_descriptor> pathStack;
	Graph::adjacency_iterator neighborIt, neighborEnd;

	Graph::vertex_descriptor currentVertex = vertex(0, g);
	g[currentVertex].pred = NULL;
	pathStack.push(currentVertex);

	while (!pathStack.empty())
	{
		currentVertex = pathStack.top();
		pathStack.pop();

		if (g[currentVertex].visited)
			return true;

		g[currentVertex].visited = true;

		tie(neighborIt, neighborEnd) = adjacent_vertices(currentVertex, g);
		Graph::vertex_descriptor next = currentVertex;
		for (; neighborIt != neighborEnd; ++neighborIt)
		{
			Graph::vertex_descriptor next = vertex(*neighborIt, g);
			if (!g[next].visited)
			{
				pathStack.push(next);
			}
		}
	}

	return false;
}

bool relax(Graph &g, Graph::vertex_descriptor u, Graph::vertex_descriptor v)
{
	pair<Graph::edge_descriptor, bool> e = edge(u, v, g);
	// there's an edge u->v && the temp distance of v can be decreased by going through u
	if (e.second && g[v].weight > g[u].weight + g[e.first].weight)
	{
		g[v].weight = g[u].weight + g[e.first].weight;
		g[v].pred = u;
		return true;
	}
	return false;
}

bool Dijkstra(Graph &g, Graph::vertex_descriptor start)//, size_t offset)
{
	heapV<Graph::vertex_descriptor, Graph> heap;
	
	pair<Graph::vertex_iterator, Graph::vertex_iterator> vertIter = vertices(g);
	for (; vertIter.first != vertIter.second; ++vertIter.first)
	{
		g[*vertIter.first].weight = LargeValue;
		g[*vertIter.first].pred = NULL;
		heap.minHeapInsert(*vertIter.first, g);
	}

	g[start].weight = 0;
	heap.buildMinHeap(heap.size(), g);
	while (heap.size() != 0)
	{
		Graph::vertex_descriptor u = heap.extractMinHeapMinimum(g);
		pair<Graph::adjacency_iterator, Graph::adjacency_iterator> neighborIter = adjacent_vertices(u, g);
		for (; neighborIter.first != neighborIter.second; ++neighborIter.first)
		{
			Graph::vertex_descriptor v = *neighborIter.first;
			if (relax(g, u, v))
				heap.minHeapDecreaseKey(heap.getIndex(v), g);
		}
	}

	// return true if connected otherwise false
	vertIter = vertices(g);
	for (; vertIter.first != vertIter.second; ++vertIter.first)
	{
		if (g[*vertIter.first].weight == LargeValue)
			return false;
	}
	return true;
}

stack<Graph::vertex_descriptor> getShortestPathFromBForD(Graph &g, Graph::vertex_descriptor target)
{
	stack<Graph::vertex_descriptor> s;
	Graph::vertex_descriptor temp = target;
	while (temp != NULL)
	{
		s.push(temp);
		temp = g[temp].pred;
	}
	return s;
}

void printPaths(Graph &g, pair<Graph::vertex_iterator, Graph::vertex_iterator> vertItrRange, Graph::vertex_descriptor sourceNode)
{
	for (; vertItrRange.first != vertItrRange.second; ++vertItrRange.first)
	{
		stack<Graph::vertex_descriptor> s = getShortestPathFromBForD(g, *vertItrRange.first);
		cout << "Path from " << *vertItrRange.first << " to " << sourceNode << ": " << endl;
		while (!s.empty())
		{
			cout << s.top();
			if (s.top() != sourceNode)
				cout << " --> ";
			s.pop();
		}
		cout << sourceNode << endl << endl;
	}
}

template <typename T>
void reverseStack(stack<T> &s)
{
	stack<T> s2;
	while (!s.empty())
	{
		s2.push(s.top());
		s.pop();
	}
	s = s2;
}

void clearVisited(Graph &g)
// Mark all nodes in g as not visited.
{
	typedef graph_traits<Graph>::vertex_iterator vertex_iter;
	pair<vertex_iter, vertex_iter> vertIter;
	for (vertIter = vertices(g); vertIter.first != vertIter.second; ++vertIter.first)
	{
		g[*vertIter.first].visited = false;
	}

	typedef graph_traits<Graph>::edge_iterator edge_iter;
	pair<edge_iter, edge_iter> edgeIter;
	for (edgeIter = edges(g); edgeIter.first != edgeIter.second; ++edgeIter.first)
	{
		g[*edgeIter.first].visited = false;
	}
}

void setNodeWeights(Graph &g, int w)
// Set all node weights to w.
{
	int num = num_vertices(g);
	for (int i = 0; i < num; i++)
	{
		g[vertex(i, g)].weight = w;
	}
}

void clearMarked(Graph &g)
{
	typedef graph_traits<Graph>::vertex_iterator vertex_iter;
	pair<vertex_iter, vertex_iter> vertIter;
	for (vertIter = vertices(g); vertIter.first != vertIter.second; ++vertIter.first)
	{
		g[*vertIter.first].marked = false;
	}

	typedef graph_traits<Graph>::edge_iterator edge_iter;
	pair<edge_iter, edge_iter> edgeIter;
	for (edgeIter = edges(g); edgeIter.first != edgeIter.second; ++edgeIter.first)
	{
		g[*edgeIter.first].marked = false;
	}
}

void initializeGraph(Graph &g,
	Graph::vertex_descriptor &start,
	Graph::vertex_descriptor &end, ifstream &fin)
	// Initialize g using data from fin.  Set start and end equal
	// to the start and end nodes.
{
	EdgeProperties e;

	int n, i, j;
	int startId, endId;
	fin >> n;
	Graph::vertex_descriptor v;

	// Add nodes.
	for (int i = 0; i < n; i++)
	{
		v = add_vertex(g);
	}

	while (fin.peek() != '.')
	{
		fin >> i >> j >> e.weight;
		add_edge(i, j, e, g);
	}
}

ostream& operator << (ostream &ostr, const VertexProperties &vp) {
	ostr<< "  Tree #" << vp.forestTreeNumber << " of the forest"
		<< "  Marked: " << vp.marked
		<< "  Pred: " << vp.pred
		<< "  Visited: " << vp.visited
		<< "  Weight: " << vp.weight << endl;
	return ostr;
}

ostream& operator << (ostream &ostr, const EdgeProperties&ep) {
	ostr <<"  Marked: " << ep.marked
		<< "  Visited: " << ep.visited
		<< "  Weight: " << ep.weight;
	return ostr;
}
typedef adjacency_list<vecS, vecS, bidirectionalS, VertexProperties, EdgeProperties> Graph;

ostream& operator << (ostream &ostr, const Graph &g) {
	// Iterate through the vertex properties and print them out
	typedef graph_traits<Graph>::vertex_iterator vertex_iter;
	pair<vertex_iter, vertex_iter> vertIter;
	for (vertIter = vertices(g); vertIter.first != vertIter.second; ++vertIter.first)
	{
		ostr << "Vertex (" << *vertIter.first << "): "<< endl;
		ostr << g[*vertIter.first] << endl;
	}

	// Iterate through the edges and print them out
	typedef graph_traits<Graph>::edge_iterator edge_iter;
	pair<edge_iter, edge_iter> edgeIter;
	for (edgeIter = edges(g); edgeIter.first != edgeIter.second; ++edgeIter.first)
	{
		ostr << "Edge (" << source(*edgeIter.first, g) << "," << target(*edgeIter.first, g) << ") :" << endl;
		ostr << g[*edgeIter.first] << endl;
	}
	return ostr;
}