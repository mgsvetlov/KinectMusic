//
//  borderclust.cpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 21/07/17.
//  Copyright © 2017 mgsvetlov. All rights reserved.
//

#include "borderclust.h"

BorderClust::BorderClust(uint16_t y, uint16_t x, uint16_t val, uint16_t parentVal) :
y(y),
first(x),
last(x),
valAvg(val),
parentValAvg(parentVal) {}

void BorderClust::Merge(BorderClust clust){
    int size = last - first + 1;
    int size1 = clust.last - clust.first;
    valAvg = (valAvg * size + clust.valAvg * size1) / (size + size1);
    parentValAvg = (parentValAvg * size + clust.parentValAvg * size1) / (size + size1);
    last = clust.last;
}

bool BorderClust::IsAdjacent(const BorderClust& cluster) const {
    int min = first < cluster.first ? first : cluster.first;
    int max = last > cluster.last ? last : cluster.last;
    return max - min <= last - first + cluster.last - cluster.first + 1;
}

BorderClust* BorderClust::AdjClustCCW(std::list<BorderClust*>& adjClusts){
    if(adjClusts.empty()){
        return nullptr;
    }
    if(!parent){
        adjClusts.sort([](const BorderClust* cl1, const BorderClust* cl2){return cl1->first < cl2->first;});
        for(auto& clust: adjClusts){
            if(!clust->isInBorder)
                return clust;
        }
        return nullptr;
    }
    std::list<BorderClust*> adjClusts1;
    std::list<BorderClust*> adjClusts2;
    for(auto& clust : adjClusts){
        if(clust->y == parent->y)
            adjClusts1.push_back(clust);
        else
            adjClusts2.push_back(clust);
    }
    adjClusts1.sort([](const BorderClust* cl1, const BorderClust* cl2){return cl1->first < cl2->first;});
    adjClusts2.sort([](const BorderClust* cl1, const BorderClust* cl2){return cl1->first < cl2->first;});
    if(parent->y < y){
        if(!adjClusts1.empty() && adjClusts1.front()->first < parent->first)
            return adjClusts1.front();
        if(!adjClusts2.empty())
            return adjClusts2.front();
        if(!adjClusts1.empty())
            return adjClusts1.back();
    }
    else {
        if(!adjClusts1.empty() && adjClusts1.back()->first > parent->first)
            return adjClusts1.back();
        if(!adjClusts2.empty())
            return adjClusts2.back();
        if(!adjClusts1.empty())
            return adjClusts1.front();
    }
    return nullptr;
}