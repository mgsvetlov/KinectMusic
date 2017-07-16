//
//  blobsclust.cpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 16/07/17.
//  Copyright © 2017 mgsvetlov. All rights reserved.
//

#include "blobsclust.hpp"

BlobsClust::BlobsClust( std::list<Blob>& blobs, int xyThresh, int depthThresh) :
blobs (blobs),
xyThresh(xyThresh),
depthThresh(depthThresh)
{
    if(blobs.empty())
        return;
    
    for(auto& blob : blobs) {
        bool isMerged(false);
        for(auto& blobClust : blobsClust){
            if(isBlobNear(blob, blobClust)){
                blobClust.cells.Merge(blob.cells);
                isMerged = true;
                break;
            }
        }
        if(!isMerged){
            blobsClust.push_back(std::move(blob));
        }
    }
    
    bool merge (true);
    while(merge) {
        merge = false;
        auto it = blobsClust.begin();
        while(it != blobsClust.end()){
            auto it1 = it;
            it1++;
            while(it1 != blobsClust.end()){
                if(isBlobNear(*it, *it1 )) {
                    it->cells.Merge(it1->cells);
                    it1 = blobsClust.erase(it1);
                    merge = true;
                    continue;
                }
                it1++;
            }
            it++;
        }
    }
    
    auto it = blobsClust.begin();
    while(it != blobsClust.end()){
        if(it->cells.Size() < 3){
            it = blobsClust.erase(it);
            continue;
        }
        it++;
    }
}

std::list<Blob>& BlobsClust::getBlobsClust(){
    return blobsClust;
}

bool BlobsClust::isBlobNear(const Blob& blob1, const Blob& blob2) {
    if(!blob1.cells.MinValCell()|| !blob2.cells.MinValCell())
        return false;
    if(abs(blob1.cells.MinValCell()->val - blob2.cells.MinValCell()->val) > depthThresh)
        return false;
    int x1 = blob1.cells.MinValCell()->x;
    int y1 = blob1.cells.MinValCell()->y;
    int x2 = blob2.cells.MinValCell()->x;
    int y2 = blob2.cells.MinValCell()->y;
    int dx = x1 - x2;
    int dy = y1 - y2;
    if(dx * dx + dy * dy > xyThresh * xyThresh)
        return false;
    return true;
}