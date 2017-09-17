//
//  integralgrid.cpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 17/09/2017.
//  Copyright © 2017 mgsvetlov. All rights reserved.
//

#include "integralgrid.h"

IntegralGrid::IntegralGrid(cv::Mat matIntegral, size_t cellSize, size_t step, size_t edge, size_t resizePow) :
resizePow(resizePow),
cellSize(cellSize >> resizePow),
step(step >> resizePow),
edge((edge / step) >> resizePow),
matIntegral(matIntegral),
matGrid(cv::Size(matIntegral.cols /step, matIntegral.rows / step), cv::DataType<float>::type, NO_DATA_VALUE)
{
    const size_t w = matIntegral.cols;
    const size_t h = matIntegral.rows;
    size_t edgeRight = w - cellSize;
    size_t edgeLow = h - edge * 1.75;
    const size_t vertShift (w * (step - 1));
    float* p0 = (float*)(matIntegral.data);
    float* p1 = p0 + cellSize;
    float* p2 = (float*)(matIntegral.data) + cellSize * w;
    float* p3 = p2 + cellSize;
    float* p = (float*)(matGrid.data);
    float* pEnd = (float*)(matIntegral.data) + matIntegral.total();
    for(int i = 0 ; p3 < pEnd; ++p, i+=step, p0+=step, p1+=step, p2+=step, p3+=step){
        int x = i % w;
        int y = (i - x) / w;
        if(x == 0)
            i+=vertShift, p0+=vertShift, p1+=vertShift, p2+=vertShift, p3+=vertShift;
        if( x >= edgeRight || y >= edgeLow){
            continue;
        }
        *p = *p0 - *p1 - *p2 + *p3;
    }
}

std::vector<std::vector<float>> IntegralGrid::getVecResponses(const std::vector<cv::Vec2i>& geometry){
    std::vector<std::vector<float>> vecResponses(matGrid.total(), std::vector<float>(geometry.size(), FLT_MAX));
    auto itResp = vecResponses.begin();
    float* pBegin = (float*)(matGrid.data);
    float* pEnd = (float*)(matGrid.data) + matGrid.total();
    float* p = (float*)(matGrid.data);
    std::vector<float*> p_geom(geometry.size());
    for(int i = 0; i < geometry.size(); ++i){
        auto& geom = geometry[i];
        p_geom[i] = (float*)(matGrid.data) + geom[1] * matGrid.cols + geom[0];
    }
    
   while(p < pEnd){
       if(*p != NO_DATA_VALUE){
            bool isInside(true);
            for(auto& p_g : p_geom){
                if(p_g < pBegin || p_g >= pEnd || *p_g == NO_DATA_VALUE ){
                    isInside = false;
                    break;
                }
            }
            
            if(isInside) {
                for(int i = 0; i < geometry.size(); ++i)
                    (*itResp)[i] = *p - *p_geom[i];
            }
       }
       ++p, ++itResp;
        for(auto& p_g : p_geom)
            ++p_g;
    }
    return vecResponses;
}
