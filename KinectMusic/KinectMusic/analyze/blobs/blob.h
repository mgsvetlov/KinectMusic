//
//  nearestblob.hpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 01/10/16.
//  Copyright © 2016 mgsvetlov. All rights reserved.
//

#ifndef nearestblob_h
#define nearestblob_h

#include <stdio.h>
#include "../types.h"

class Blob {
private:
    Blob(cv::Mat mat16, std::list<std::vector<int>>& lvBlobs, cv::Mat matBlobsMap, int x, int y, int thresh, int blobNum);
public:
    static cv::Mat findBlobs(cv::Mat mat16, std::list<std::vector<int>>& lvBlobs);

};
#endif /* nearestblob_hpp */
