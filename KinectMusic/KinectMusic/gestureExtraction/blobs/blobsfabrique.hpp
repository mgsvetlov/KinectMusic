//
//  blobsfabrique.hpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 16/07/17.
//  Copyright © 2017 mgsvetlov. All rights reserved.
//

#ifndef blobsfabrique_hpp
#define blobsfabrique_hpp

#include "blob.h"

class BlobsFabrique {
public:
    BlobsFabrique(int mode, cv::Mat mat, const std::list<int>& inds = std::list<int>());
    void constructBlobsExt(cv::Mat origMat);
    std::list<Blob>& getBlobs();
    std::list<Blob>& getBlobsExt();
    int getBodyDepth();
private:
    void blobsFabrique0();
    void blobsFabrique1();
    int minCellIndex();
    void computeBodyDepth();
private:
    cv::Mat mat;
    int mode = 0;
    std::list<Blob> blobs;
    std::list<Blob> blobsExt;
    int bodyDepth = -1;
};

#endif /* blobsfabrique_hpp */
