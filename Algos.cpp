#include "Graph.h"

int findGraphRoot(const Graph& graph) {
    int num_of_vertices = graph.numOfVertices();
    if (num_of_vertices == 0) return -1;

    // DFS from random vertex
    auto results = graph.DFS(0);

    // find root of last DFS tree
    int candidate = 0, max_discovery_time = 0;
    for (int vertex = 0; vertex < num_of_vertices; vertex++) {
        if (results.parent[vertex] == -1) {   // vertex is some DFS tree root
            if (max_discovery_time < results.discovery[vertex]) {
                candidate = vertex;
                max_discovery_time = results.discovery[vertex];
            }
        }
    }

    // DFS from root of last DFS tree
    results = graph.DFS(candidate);

    // check if there is only one tree in the last DFS
    int num_of_roots = 0;
    for (const auto& parent : results.parent) {
        if (parent == -1) num_of_roots++;
    }

    // if it's true return that root, else return -1 (no root)
    if (num_of_roots == 1) return candidate;
    return -1;
}

vector<int> lengthsOfShortestPathsDAG(const Graph& graph, map<Edge,int>& weights, int s) {
    // get topological sort
    auto top_sort = graph.topologicalSort();

    // init shortest path = infty for all vertices
    // init shortest path to s = 0
    vector<int> shortest(graph.numOfVertices(), infty);
    shortest[s] = 0;

    // for each vertex v after s in the sort
        // for each u in the in list of v
            // if shortest(u)+w(u->v) < shortest(v) update
    int place = 0;
    while (top_sort[place] != s) place++;
    for (place++; place < graph.numOfVertices(); place++) {
        int v = top_sort[place];
        auto in_list = graph.getInList(v);
        for (auto u : in_list) {
            Edge edge = {u,v};
            if (weights.count(edge) == 0) continue;
            if (shortest[u] + weights[edge] < shortest[v]) {
                shortest[v] = shortest[u] + weights[edge];
            }
        }
    }

    return shortest;
}

vector<int> getLongestPathDAG(const Graph& graph, int s, int t) {
    // build weight function, each node weights -1
    map<Edge, int> weights;
    for (int v = 0; v < graph.numOfVertices(); v++) {
        auto out_list = graph.getOutList(v);
        for (int u : out_list) {
            Edge edge = {v,u};
            weights[edge] = -1;
        }
    }

    // run find shortest paths from s, if shortest[t] == infty than no path at all
    auto shortest = lengthsOfShortestPathsDAG(graph, weights, s);
    if (shortest[t] == infty) return {};

    // start from v = t and while v != s
        // find u in in list of v such that shortest(u) = shortest(v) + 1
        // insert u to the path
    vector<int> path(1,t);
    int v = t;
    while (v != s) {
        auto in_list = graph.getInList(v);
        for (auto u : in_list) {
            if (shortest[u] == shortest[v] + 1) {
                path.push_back(u);
                v = u;
                break;
            }
        }
    }

    // reverse the path and return it
    reverse(path.begin(), path.end());
    return path;
}

vector<int> shortestLengthsOfTwoColorChangingPath(const Graph& graph, map<Edge,char>& color, int s) {
    // create new graph with double the num of the vertices
    // each vertex v appears twice, one as v (green) second as v+n (yellow)
    // for each yellow edge uv add an edge from u to v+n
    // for each green edge uv add an edge from u+n to v
    int n = graph.numOfVertices();
    Graph bipartite(2*n);
    for (int u = 0; u < n; u++) {
        auto out_list = graph.getOutList(u);
        for (auto v : out_list) {
            Edge edge = {u,v};
            if (color[edge] == 'Y') {
                bipartite.addEdge(u,v+n);
            } else {
                bipartite.addEdge(u+n, v);
            }
        }
    }

    // run two BFSs from s and from s+n in the new graph
    auto bfs1 = bipartite.BFS(s);
    auto bfs2 = bipartite.BFS(s+n);

    // for each vertex, return minimum of the four different distances
    vector<int> distance(n);
    for (int i = 0; i < n; i++) {
        int min_in_bfs1 = min(bfs1[i], bfs1[i+n]);
        int min_in_bfs2 = min(bfs2[i], bfs2[i+n]);
        distance[i] = min(min_in_bfs1, min_in_bfs2);
    }

    return distance;
}

vector<int> shortestLengthsOfPathsZeroOneGraph(const Graph& graph, map<Edge,int>& weight, int s) {
    // build graph Z with same vertices of graph, but with only zero edges
    Graph Z(graph.numOfVertices(), false);
    for (int u = 0; u < graph.numOfVertices(); u++) {
        auto edge_list = graph.getOutList(u);
        for (int v : edge_list) {
            Edge edge = {u, v};
            if (weight[edge] == 0) Z.addEdge(u, v);
        }
    }

    // find connected components of Z
    auto component = getConnectedComponentsUndirectedGraph(Z);

    // build new graph C where each vertex represents a connected component in Z
    // add edge uv if there is an edge with weight 1 from some vertex in component U to component V
    int num_of_components = 0;
    for (auto comp : component) if (comp > num_of_components) num_of_components = comp;
    num_of_components++;

    Graph C(num_of_components, false);
    for (int u = 0; u < graph.numOfVertices(); u++) {
        auto edge_list = graph.getOutList(u);
        for (int v : edge_list) {
            Edge edge = {u, v};
            if (weight[edge] == 1) C.addEdge(component[u], component[v]);
        }
    }

    // run BFS on C from S (where S is s's connected component)
    auto comp_distance = C.BFS(component[s]);

    // for each node v return distance[V] where V is v's connected component
    vector<int> distances(graph.numOfVertices());
    for (int v = 0; v < graph.numOfVertices(); v++) distances[v] = comp_distance[component[v]];
    return distances;
}

vector<int> getConnectedComponentsUndirectedGraph(const Graph& graph) {
    if (graph.numOfVertices() == 0) return {};

    auto results = graph.DFS(0);

    vector<vector<int>> tmp_by_discovery(graph.numOfVertices()); // <discovery, vertex>
    vector<vector<int>> tmp_by_finish(graph.numOfVertices());    // <finish, vertex>
    for (int v = 0; v < graph.numOfVertices(); v++) {
        tmp_by_discovery[v] =  {results.discovery[v], v};
        tmp_by_finish[v] =     {results.finish[v], v};
    }
    sort(tmp_by_discovery.begin(), tmp_by_discovery.end());
    sort(tmp_by_finish.begin(), tmp_by_finish.end());

    vector<int> vertices_by_discovery(graph.numOfVertices());
    vector<int> vertices_by_finish(graph.numOfVertices());
    for (int i = 0; i < graph.numOfVertices(); i++) {
        vertices_by_discovery[i]    = tmp_by_discovery[i][1];
        vertices_by_finish[i]       = tmp_by_finish[i][1];
    }

    vector<int> component(graph.numOfVertices());
    int comp = 0, iter = 0;
    while (iter < graph.numOfVertices()) {
        int curr_root = vertices_by_discovery[iter];
        component[curr_root] = comp;
        while (vertices_by_finish[iter] != curr_root) {
            component[vertices_by_finish[iter]] = comp;
            iter++;
        }
        comp++;
        iter++;
    }

    return component;
}

set<Edge> kruskal(const Graph& G, const map<Edge,int>& w) {
    UF uf(G.numOfVertices());

    vector<pair<int,Edge>> sorted(w.size());
    int i = 0;
    for (auto edge : w) sorted[i++] = {edge.second, edge.first};
    sort(sorted.begin(), sorted.end());

    set<Edge> res;
    // iterate over the edges from low weight to high weight
    // if both vertices touching the edge not in the same set, put edge in MST
    for (i = 0; i < (int)sorted.size(); i++) {
        auto edge = sorted[i].second;
        if (uf.find(edge.first) != uf.find(edge.second)){
            res.insert(edge);
            uf.uni(uf.find(edge.first), uf.find(edge.second));
        }
    }

    return res;
}

set<Edge> prim(const Graph& G, const map<Edge,int>& w) {
    // min heap of unused vertices sorted by min crossing edge

    // pop min from the heap, insert the edge to the MST

    // update heap for any of the new vertex's edges
}