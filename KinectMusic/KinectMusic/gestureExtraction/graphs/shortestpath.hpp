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
property<vertex_name_t, char,
property<vertex_distance_t, int,
property<vertex_predecessor_t, vertex_descriptor> > > >,
property<edge_weight_t, int> > graph_t;
typedef std::pair<int, int> Edge;

class ShortestPath {
public:
    static void Test() {
        std::vector<int> verts {0,1,2,3,4,5};
        std::vector<Edge> edge_array = { Edge(verts[0], verts[2]), Edge(verts[1], verts[1]), Edge(verts[1], verts[3]), Edge(verts[1], verts[4]),
            Edge(verts[2], verts[1]), Edge(verts[2], verts[3]), Edge(verts[3], verts[4]), Edge(verts[4], verts[0]), Edge(verts[4], verts[1])
        };
        std::vector<int> weights = { 1, 2, 1, 2, 7, 3, 1, 1, 1 };
        int src (0);
        std::list<int> dsts {4};
        
        graph_t g(&edge_array[0], &edge_array[0] + edge_array.size(), &weights[0], verts.size());
        property_map<graph_t, edge_weight_t>::type weightmap = get(edge_weight, g);
        
        // Manually intialize the vertex index and name maps
        property_map<graph_t, vertex_index_t>::type indexmap = get(vertex_index, g);
        property_map<graph_t, vertex_name_t>::type name = get(vertex_name, g);
        graph_traits<graph_t>::vertex_iterator i, iend;
        int c = 0;
        for (boost::tie(i, iend) = vertices(g); i != iend; ++i, ++c) {
            indexmap[*i] = c;
            name[*i] = 'A' + c;
        }
        
        vertex_descriptor s = vertex(0, g);
        
        property_map<graph_t, vertex_distance_t>::type
        d = get(vertex_distance, g);
        property_map<graph_t, vertex_predecessor_t>::type
        p = get(vertex_predecessor, g);
        dijkstra_shortest_paths(g, s, predecessor_map(p).distance_map(d));
        
        std::cout << "distances and parents:" << std::endl;
       
        for(auto ind : dsts){
            graph_traits < graph_t >::vertex_iterator vi, vend;
            boost::tie(vi, vend) = vertices(g);
            for(int j = 0; j < ind; ++j, ++vi);
            auto dist = d[*vi];
            std::cout << "distance(" << verts[ind] << ") = " << dist << ", ";
            if(dist == INT_MAX)
                continue;
            auto pred = *vi;
            auto predInd = indexmap[pred];
            while(predInd != src) {
                std::cout << "  " << verts[predInd];
                pred = p[pred];
                predInd = indexmap[pred];
            }
            std::cout << "  " << src << std::endl;
        }
        std::cout << std::endl;
        
        /*std::ofstream dot_file("dijkstra-eg.dot");
        dot_file << "digraph D {\n"
        << "  rankdir=LR\n"
        << "  size=\"4,3\"\n"
        << "  ratio=\"fill\"\n"
        << "  edge[style=\"bold\"]\n" << "  node[shape=\"circle\"]\n";
        
        graph_traits < graph_t >::edge_iterator ei, ei_end;
        for (boost::tie(ei, ei_end) = edges(g); ei != ei_end; ++ei) {
            graph_traits < graph_t >::edge_descriptor e = *ei;
            graph_traits < graph_t >::vertex_descriptor
            u = source(e, g), v = target(e, g);
            dot_file << name[u] << " -> " << name[v]
            << "[label=\"" << get(weightmap, e) << "\"";
            if (p[v] == u)
                dot_file << ", color=\"black\"";
            else
                dot_file << ", color=\"grey\"";
            dot_file << "]";
        }
        dot_file << "}";*/
    }
    
};

#endif /* shortestpath_hpp */
