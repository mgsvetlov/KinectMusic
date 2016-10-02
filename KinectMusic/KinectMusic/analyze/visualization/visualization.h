//
//  visualization.h
//  KinectMusic
//
//  Created by Mikhail Svetlov on 01/10/16.
//  Copyright © 2016 mgsvetlov. All rights reserved.
//

#ifndef visualization_h
#define visualization_h

#include "../types.h"

class Blob;

class Visualization {
private:
    Visualization();
public:
    static void visualize(cv::Mat mat);
    static void visualizeMap(const cv::Size& size, const cv::Size& fullSize, const std::list<Blob>& lBlobs);
private:
    static Visualization* p_vis;
};

#endif /* visualization_h */
