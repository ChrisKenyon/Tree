// Tree.cpp : Defines the entry point for the console application.
//

#include "stdafx.h"
#include "graph.h"
#include "d_except.h"

using namespace boost;
using namespace std;

int main()
{
	try
	{
		ifstream fin;

		string fileName = "graph4.txt";

		fin.open(fileName.c_str());
		if (!fin)
		{
			cerr << "Cannot open " << fileName << endl;
			exit(1);
		}

		Graph g;
		typedef graph_traits<Graph>::vertex_iterator vertex_iter;
		std::pair<vertex_iter, vertex_iter> vertItrRange = vertices(g);
		Graph::vertex_descriptor first = vertex(*vertItrRange.first, g);
		Graph::vertex_descriptor second = vertex(*vertItrRange.second, g);

		initializeGraph(g, first, second, fin);
		clearMarked(g);
		clearVisited(g);
		fin.close();

		Graph spanningForest;
		findMinSpanningForest(g, spanningForest);
		cout << spanningForest;
	}
	catch (std::exception ex)
	{
		cout << ex.what();
	}
	int pause;
	cin >> pause;
    return 0;
}