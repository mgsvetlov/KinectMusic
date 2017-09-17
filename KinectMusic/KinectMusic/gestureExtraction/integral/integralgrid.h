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

class IntegralGrid {
public:
    IntegralGrid(cv::Mat matIntegral, size_t cellSize, size_t step, size_t edge = 0, size_t resizePow = 0);
    std::vector<std::vector<float>> getVecResponses(const std::vector<cv::Vec2i>& geometry);
    const cv::Size getSize() const {return matGrid.size();}
private:
    size_t resizePow;
    size_t cellSize;
    size_t step;
    size_t edge;
    cv::Mat matIntegral;
    cv::Mat matGrid;
    
};

#endif /* integralgrid_hpp */
