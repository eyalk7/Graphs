#include "Graph.h"

Graph::Graph(int num_of_vertices, bool directed) :  out(vector<set<int>>(num_of_vertices)),
                                                    in(vector<set<int>>(num_of_vertices)),
                                                    num_of_edges(0),
                                                    is_directed(directed) {

}

bool Graph::addEdge(int i, int j) {
    int num_of_vertices = out.size();
    if (i < 0 || i >= num_of_vertices || j < 0 || j >= num_of_vertices) return false;
    if (out[i].count(j) > 0) return false;

    out[i].insert(j);
    if (!is_directed) out[j].insert(i);
    in[j].insert(i);
    if (!is_directed) in[i].insert(j);
    num_of_edges++;
    return true;
}

bool Graph::removeEdge(int i, int j) {
    int num_of_vertices = out.size();
    if (i < 0 || i >= num_of_vertices || j < 0 || j >= num_of_vertices) return false;
    if (out[i].count(j) == 0) return false;

    out[i].erase(j);
    if (!is_directed) out[j].erase(i);
    in[j].erase(i);
    if (!is_directed) in[i].erase(j);
    num_of_edges--;
    return true;
}

const set<int>& Graph::getOutList(int i) const {
    int num_of_vertices = out.size();
    if (i < 0 || i >= num_of_vertices) throw OUT_OF_BOUND();

    return out[i];
}

const set<int>& Graph::getInList(int i) const {
    int num_of_vertices = out.size();
    if (i < 0 || i >= num_of_vertices) throw OUT_OF_BOUND();

    return in[i];
}

Graph Graph::getTranspose() const {
    Graph transpose(0);
    transpose.in = out;
    transpose.out = in;

    return  transpose;
}

Graph::DFSresult::DFSresult(int num_of_vertices) :  parent(vector<int>(num_of_vertices,-1)),
                                                    discovery(vector<int>(num_of_vertices, -1)),
                                                    finish(vector<int>(num_of_vertices, -1))
                                                    {

}

Graph::DFSresult Graph::DFS(int s) const {
    int num_of_vertices = out.size();

    vector<char> color (num_of_vertices, 'w');
    DFSresult results(num_of_vertices);
    int i = 0;

    for (int vertex = 0; vertex < num_of_vertices; vertex++) {
        if (color[vertex] == 'w') DFS_VISIT(vertex, i, color, results);
    }

    return results;
}

void Graph::DFS_VISIT(int vertex1, int& i, vector<char>& color, DFSresult& results) const {
    color[vertex1] = 'g';
    results.discovery[vertex1] = ++i;
    for (int vertex2 : out[vertex1]) {
        if (color[vertex2] != 'w') continue;

        results.parent[vertex2] = vertex1;
        DFS_VISIT(vertex2, i, color, results);
    }
    results.finish[vertex1] = ++i;
    color[vertex1] = 'b';
}

vector<int> Graph::BFS(int s) const {
    queue<int> Q;
    vector<int> distance(numOfVertices(), infty);

    // init distance[v] = infty for all vertices
    // init distance[s] = 0
    // insert s to Q
    distance[s] = 0;
    Q.push(s);

    // while Q ins't empty
        // pop v from Q
        // for each u in v's out list that doesn't have distance
            // distance[u] = distance[v] + 1
            // push u to Q
    while (!Q.empty()) {
        int v = Q.front();
        Q.pop();

        for (auto u : out[v]) {
            if (distance[u] == infty) {
                distance[u] = distance[v] + 1;
                Q.push(u);
            }
        }
    }

    // return
    return distance;
}

vector<int> Graph::topologicalSort() const {
    if (numOfVertices() == 0) return {};

    // run DFS
    auto results = DFS(0);

    // check if we got back edges, and if we did return {} (no topological sort)
    // back edge is an edge from u to v such that d(v) < d(u) and f(u) < f(v)
    for (int u = 0; u < numOfVertices(); u++) {
        for (int v : out[u]) {
            if (results.discovery[v] < results.discovery[u] && results.finish[u] < results.finish[v]) {
                return {};
            }
        }
    }

    // sort the vertices by finish time in descending order
    vector<pair<int,int>> vertices_by_finish_time(numOfVertices()); // <finish, vertex>
    for (int vertex = 0; vertex < numOfVertices(); vertex++) {
        vertices_by_finish_time[vertex] = {results.finish[vertex], vertex};
    }
    sort(vertices_by_finish_time.begin(), vertices_by_finish_time.end());
    reverse(vertices_by_finish_time.begin(), vertices_by_finish_time.end());

    // return the sort
    vector<int> to_return(numOfVertices());
    for(int numbering = 0; numbering < numOfVertices(); numbering++) {
        int curr_vertex = vertices_by_finish_time[numbering].second;
        to_return[numbering] = curr_vertex;
    }
    return to_return;
}

int Graph::numOfVertices() const {
    return out.size();
}

int Graph::numOfEdges() const {
    return num_of_edges;
}
