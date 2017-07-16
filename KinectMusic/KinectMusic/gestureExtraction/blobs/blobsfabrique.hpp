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
    BlobsFabrique(cv::Mat mat, int mode = 0);
    std::list<Blob>& getBlobs();
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
    int bodyDepth = -1;
};

#endif /* blobsfabrique_hpp */
