//
//  borderclustrow.cpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 21/07/17.
//  Copyright © 2017 mgsvetlov. All rights reserved.
//

#include "borderclustrow.h"

void BorderClustRow::MergeClusts() {
    clusts.sort([](const BorderClust& cl1, const BorderClust& cl2){return cl1.first < cl2.first;});
    auto it = clusts.begin();
    while(it != clusts.end()) {
        auto it1 = it;
        ++it1;
        if(it1 == clusts.end())
            break;
        if(it->last == it1->first - 1){
            it->Merge(*it1);
            clusts.erase(it1);
            continue;
        }
        ++it;
    }
}
std::list<BorderClust*> BorderClustRow::FindAdjacentChildren(const BorderClust& cluster)  {
    std::list<BorderClust*> clustsAdj;
    for(auto& clust : clusts){
        if(cluster.parent == &clust)
            continue;
        if(clust.IsAdjacent(cluster))
            clustsAdj.push_back(&clust);
    }
    return clustsAdj;
}