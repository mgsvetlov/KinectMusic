//
//  shortestpath.cpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 29/08/17.
//  Copyright © 2017 mgsvetlov. All rights reserved.
//

#include "shortestpath.hpp"

ShortestPath::ShortestPath(const Graph& graph, int src_ind) :
g(&graph.edges[0], &graph.edges[0] + graph.edges.size(), &graph.weights[0], graph.verts_count),
src_ind(src_ind)
{
    indexmap = get(vertex_index, g);
    graph_traits<graph_t>::vertex_iterator i, iend;
    int c = 0;
    for (boost::tie(i, iend) = vertices(g); i != iend; ++i, ++c) {
        indexmap[*i] = c;
    }
    
    vertex_descriptor s = vertex(src_ind, g);
    
    d = get(vertex_distance, g);
    p = get(vertex_predecessor, g);
    dijkstra_shortest_paths(g, s, predecessor_map(p).distance_map(d));
}

int ShortestPath::GetDistance(int dst_ind){
    graph_traits < graph_t >::vertex_iterator vi, vend;
    boost::tie(vi, vend) = vertices(g);
    for(int j = 0; j < dst_ind; ++j, ++vi);
    auto dist = d[*vi];
    if(dist == INT_MAX)
        dist = -1;
    //std::cout << "src_ind " << src_ind << " dst_ind " << dst_ind << " dist " << dist << std::endl;
    return dist;
}

std::list<int> ShortestPath::GetPath(int dst_ind){
    std::list<int> path;
    graph_traits < graph_t >::vertex_iterator vi, vend;
    boost::tie(vi, vend) = vertices(g);
    for(int j = 0; j < dst_ind; ++j, ++vi);
    auto dist = d[*vi];
    if(dist == INT_MAX)
        return path;
    auto pred = *vi;
    auto predInd = indexmap[pred];
    while(predInd != src_ind) {
        path.push_front(predInd);
        pred = p[pred];
        predInd = indexmap[pred];
    }
    path.push_front(src_ind);
    /*std::cout << "path ";
    for(auto ind : path)
        std::cout << ind  << "  ";
    std::cout <<  std::endl;*/
    return path;
}