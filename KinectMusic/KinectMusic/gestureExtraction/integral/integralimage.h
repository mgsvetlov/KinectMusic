//
//  integralimage.hpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 17/09/2017.
//  Copyright © 2017 mgsvetlov. All rights reserved.
//

#ifndef integralimage_h
#define integralimage_h
#include "types.h"

class IntegralImage {
public:
    IntegralImage(cv::Mat mat, size_t resizePow = 0);
    cv::Mat getMatIntegral() const {return matIntegral;}
    size_t getResizePow() const {return resizePow;}
    
private:
    size_t resizePow;
    cv::Mat mat;
    cv::Mat matIntegral;
    
};


#endif /* integralimage_h */
