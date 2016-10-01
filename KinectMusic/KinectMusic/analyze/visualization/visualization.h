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

class Visualization {
private:
    Visualization();
public:
    static void visualize(cv::Mat mat);
    static void visualizeMap(cv::Mat mat, const std::list<std::vector<int>>& lvBlobs);
private:
    static cv::Size fullSize;
    static Visualization* p_vis;
};

#endif /* visualization_h */
