//
//  shortestpath.h
//  KinectMusic
//
//  Created by Mikhail Svetlov on 28/08/17.
//  Copyright © 2017 mgsvetlov. All rights reserved.
//

#ifndef shortestpath_hpp
#define shortestpath_hpp
#include <boost/config.hpp>
#include <iostream>
#include <fstream>
#include <vector>
#include <list>
#include <climits>

#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/dijkstra_shortest_paths.hpp>

using namespace boost;

typedef adjacency_list_traits<listS, listS,
directedS>::vertex_descriptor vertex_descriptor;
typedef adjacency_list < listS, listS, directedS,
property<vertex_index_t, int,
property<vertex_distance_t, int,
property<vertex_predecessor_t, vertex_descriptor> > >,
property<edge_weight_t, int> > graph_t;
typedef std::pair<int, int> Edge;

struct Graph{
    size_t verts_count;
    std::vector<Edge> edges;
    std::vector<int> weights;
};

class ShortestPath {
public:
    ShortestPath(const Graph& graph, int src_ind);
    int GetDistance(int dst_ind);
    std::list<int> GetPath(int dst_ind);
private:
    graph_t g;
    property_map<graph_t, vertex_index_t>::type indexmap;
    property_map<graph_t, vertex_distance_t>::type d;
    property_map<graph_t, vertex_predecessor_t>::type p;
    int src_ind;
};

#endif /* shortestpath_hpp */
