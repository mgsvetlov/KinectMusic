//
//  limbsextractor.cpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 22/10/16.
//  Copyright © 2016 mgsvetlov. All rights reserved.
//

#include "limb.h"

Limb::Limb(cv::Mat mat, const Blob& blob){
    const auto& blobCells = blob.getLCellsConst();
    for(const auto& cell : blobCells){
        this->lCells.push_back(cell);
    }
}

void Limb::findLimbs(cv::Mat mat, const std::list<Blob>& lvBlobs, std::list<Limb>& lvLimbs) {
    for(auto& blob : lvBlobs){
        lvLimbs.push_back(Limb (mat, blob));
    }
}
