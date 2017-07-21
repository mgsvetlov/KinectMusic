//
//  borderclust.hpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 21/07/17.
//  Copyright © 2017 mgsvetlov. All rights reserved.
//

#ifndef borderclust_h
#define borderclust_h

#include <cstdint>
#include <list>

struct BorderClust {
    BorderClust(uint16_t y, uint16_t x, uint16_t val, uint16_t parentVal);
    void Merge(BorderClust clust);
    bool IsAdjacent(const BorderClust& cluster) const;
    BorderClust* AdjClustCCW(std::list<BorderClust*>& adjClusts);
    
    uint16_t y;
    uint16_t first;
    uint16_t last;
    uint16_t valAvg;
    uint16_t parentValAvg;
    bool isInBorder = false;
    BorderClust* parent = nullptr;
    BorderClust* child;
};

#endif /* borderclust_h */
