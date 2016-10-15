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
    static bool showImage();
    
    static cv::Mat blobs2img_mark(const std::list<Blob>& lBlobs, const cv::Size& size);
    static cv::Mat mat2img(cv::Mat mat);
    
    static void setMatImage(cv::Mat mat){matImage = mat.clone();}
    
    static void setIsNeedRedraw(bool val) {isNeedRedraw = val;}
    static bool getIsNeedRedraw() {return isNeedRedraw;}

    
private:
    static Visualization* p_vis;
    static cv::Mat matImage;
    static bool isNeedRedraw;
};

#endif /* visualization_h */
