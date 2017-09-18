//
//  integralgrid.hpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 17/09/2017.
//  Copyright © 2017 mgsvetlov. All rights reserved.
//

#ifndef integralgrid_hpp
#define integralgrid_hpp

#include "types.h"

class IntegralImage;

class IntegralGrid {
public:
    IntegralGrid(const IntegralImage& integralImage, size_t cellSize, size_t step, cv::Vec2i edge = cv::Vec2i(0,0));
    std::vector<std::vector<float>> getVecResponses(const std::vector<cv::Vec2i>& geometry, std::function<float (float, float)> func = std::minus<float>());
    const cv::Size getSize() const {return matGrid.size();}
private:
    size_t resizePow;
    size_t cellSize;
    size_t step;
    cv::Vec2i edge;
    const cv::Mat matIntegral;
    cv::Mat matGrid;
    
};

#endif /* integralgrid_hpp */
