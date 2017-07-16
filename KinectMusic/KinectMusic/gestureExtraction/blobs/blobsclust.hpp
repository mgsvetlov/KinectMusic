//
//  blobsclust.hpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 16/07/17.
//  Copyright © 2017 mgsvetlov. All rights reserved.
//

#ifndef blobsclust_hpp
#define blobsclust_hpp

#include "blob.h"

class BlobsClust {
public:
    BlobsClust( std::list<Blob>& blobs, int xyThresh, int depthThresh);
    std::list<Blob>& getBlobsClust();
private:
    bool isBlobNear(const Blob& blob1, const Blob& blob2);
private:
    std::list<Blob>& blobs;
    std::list<Blob> blobsClust;
    int xyThresh;
    int depthThresh;
};

#endif /* blobsclust_hpp */
