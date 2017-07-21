//
//  borderclustrow.hpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 21/07/17.
//  Copyright © 2017 mgsvetlov. All rights reserved.
//

#ifndef borderclustrow_h
#define borderclustrow_h

#include "borderclust.h"

struct  BorderClustRow {
    void MergeClusts();
    std::list<BorderClust*> FindAdjacentChildren(const BorderClust& cluster);
    
    std::list<BorderClust> clusts;
};

#endif /* borderclustrow_h */
