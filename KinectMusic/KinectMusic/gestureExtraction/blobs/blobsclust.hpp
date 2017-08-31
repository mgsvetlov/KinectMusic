//
//  blobsclust.hpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 16/07/17.
//  Copyright © 2017 mgsvetlov. All rights reserved.
//

#ifndef blobsclust_hpp
#define blobsclust_hpp

#include "blobext.hpp"

template<typename T> class BlobsClust {
public:
    BlobsClust( std::list<T>& blobs, int xyThresh, int depthThresh, int blobMinSize = -1, bool isAdjacentTest = false);
    std::list<T>& getBlobsClust();
private:
    bool isBlobNear(const T& blob1, const T& blob2);
private:
    std::list<T>& blobs;
    std::list<T> blobsClust;
    int xyThresh;
    int depthThresh;
    bool isAdjacentTest;
};


template<typename T> BlobsClust<T>::BlobsClust( std::list<T>& blobs, int xyThresh, int depthThresh, int blobMinSize, bool isAdjacentTest) :
blobs (blobs),
xyThresh(xyThresh),
depthThresh(depthThresh),
isAdjacentTest(isAdjacentTest)
{
    if(blobs.empty())
        return;
    
    for(auto& blob : blobs) {
        bool isMerged(false);
        for(auto& blobClust : blobsClust){
            if(isBlobNear(blob, blobClust)){
                blobClust.getCells().Merge(blob.getCells());
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
                    it->getCells().Merge(it1->getCells());
                    it1 = blobsClust.erase(it1);
                    merge = true;
                    continue;
                }
                it1++;
            }
            it++;
        }
    }
    
    //filter small blobs
    if(blobMinSize == -1)
        return;
    
    auto it = blobsClust.begin();
    while(it != blobsClust.end()){
        if(it->getCellsConst().Size() < blobMinSize){
            it = blobsClust.erase(it);
            continue;
        }
        ++it;
    }
}

template<typename T> std::list<T>& BlobsClust<T>::getBlobsClust(){
    return blobsClust;
}

template<typename T> bool BlobsClust<T>::isBlobNear(const T& blob1, const T& blob2) {
    if(!blob1.getCellsConst().MinValCell()|| !blob2.getCellsConst().MinValCell())
        return false;
    if(depthThresh > 0){
        if(std::abs(blob1.getCellsConst().MinValCell()->val - blob2.getCellsConst().MinValCell()->val) > depthThresh)
            return false;
    }
    if(xyThresh > 0){
        if(isAdjacentTest){
            bool isNear(false);
            for(const auto& cell1 : blob1.getCellsConst().AllConst()){
                int x1 = cell1.x;
                int y1 = cell1.y;
                for(const auto& cell2 : blob2.getCellsConst().AllConst()){
                    int x2 = cell2.x;
                    int y2 = cell2.y;
                    int dx = x1 - x2;
                    int dy = y1 - y2;
                    if(dx * dx + dy * dy <= xyThresh * xyThresh){
                        isNear = true;
                        break;
                    }
                }
                if(isNear)
                    break;
            }
            if(!isNear)
                return false;
        }
        else {
            int x1 = blob1.getCellsConst().MinValCell()->x;
            int y1 = blob1.getCellsConst().MinValCell()->y;
            int x2 = blob2.getCellsConst().MinValCell()->x;
            int y2 = blob2.getCellsConst().MinValCell()->y;
            int dx = x1 - x2;
            int dy = y1 - y2;
            if(dx * dx + dy * dy > xyThresh * xyThresh)
                return false;
        }
    }
    if(!isAdjacentTest)
        return true;
    return blob1.IsAdjacent(blob2, depthThresh);
}

#endif /* blobsclust_hpp */
