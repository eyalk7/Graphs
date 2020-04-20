#include <vector>
#include <set>
#include <algorithm>
#include <map>
#include <queue>
using std::vector;
using std::set;
using std::map;
using std::queue;
using std::exception;
using std::pair;
using std::sort;
using std::reverse;
using std::min;

#ifndef GRAPHS_GRAPH_H
#define GRAPHS_GRAPH_H

const int infty = 1000000000;

typedef pair<int,int> Edge;
class UF {
    vector<int> parent;
    vector<int> size;
public:
    UF(int size) : parent(vector<int>(size,-1)), size(vector<int>(size,1)) {};
    void uni(int a, int b) {
        if (size[a] > size[b]) {
            parent[b] = a;
        } else {
            parent[a] = b;
        }
    }
    int find(int a) {
        if (parent[a] == -1) return a;

        int par = a;
        while(parent[par] != -1) par = parent[par];

        int tmp;
        while (parent[a] != par) {
            tmp = parent[a];
            parent[a] = par;
            a = tmp;
        }

        return par;
    }
};

class Graph {
public:
    class DFSresult {
    public:
        explicit DFSresult(int num_of_vertices);
        vector<int> parent;
        vector<int> discovery;
        vector<int> finish;
    };

    explicit Graph(int num_of_vertices, bool directed = true);
    ~Graph() = default;
    bool addEdge(int i, int j);
    bool removeEdge(int i, int j);
    const set<int>& getOutList(int i)const;
    const set<int>& getInList(int i) const;
    Graph getTranspose() const;
    DFSresult DFS(int s) const;
    vector<int> BFS(int s) const;
    vector<int> topologicalSort() const;

    int numOfVertices() const;
    int numOfEdges() const;

    class OUT_OF_BOUND : public exception {};
private:
    // each vertex represented by an integer
    vector<set<int>> out;
    vector<set<int>> in;
    int num_of_edges;
    bool is_directed;

    void DFS_VISIT(int vertex, int& i, vector<char>& color, DFSresult& result) const;
};

//----------------------------ALGORITHMS----------------------
/**
 * @param graph - any graph
 * @return - root vertex if the graph has a root
 *           -1 if it doesn't
 */
int findGraphRoot(const Graph& graph);

/**
 * @param graph - DAG
 * @param weights - weight function E -> int
 * @param s - starting vertex
 * @return - length of shortest paths from s to each vertex v
 */
vector<int> lengthsOfShortestPathsDAG(const Graph& graph, map<Edge,int>& weights, int s);

/**
 * @param graph - DAG
 * @param s - starting vertex
 * @param t - finish vertex
 * @return - a longest path from s to t
 */
vector<int> getLongestPathDAG(const Graph& graph, int s, int t);

/**
 * @param graph - directed graph
 * @param color - color function E -> {Y,G}
 * @param s - starting vertex
 * @return - length of shortest color changing path from s to v for each vertex v
 */
vector<int> shortestLengthsOfTwoColorChangingPath(const Graph& graph, map<Edge,char>& color, int s);

/** todo : not working
 * @param graph - undirected graph
 * @param weight - weight function E -> {0,1}
 * @param s - starting vertex
 * @return - length of shortest path from s to each vertex
 */
vector<int> shortestLengthsOfPathsZeroOneGraph(const Graph& graph, map<Edge,int>& weight, int s);

/**
 * @param graph - undirected graph
 * @return - number of connected component for each vertex
 */
vector<int> getConnectedComponentsUndirectedGraph(const Graph& graph);

/**
 * Kruskal
 */
 set<Edge> kruskal(const Graph& G, const map<Edge,int>& w);

 /**
  * prim
  */
 set<Edge> prim(const Graph& G, const map<Edge,int>& w);
#endif //GRAPHS_GRAPH_H
