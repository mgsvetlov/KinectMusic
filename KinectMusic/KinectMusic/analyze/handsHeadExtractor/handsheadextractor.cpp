//
//  handsheadextractor.cpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 22/10/16.
//  Copyright © 2016 mgsvetlov. All rights reserved.
//

#include "handsheadextractor.h"

HandsHeadExtractor::HandsHeadExtractor(cv::Mat mat, int filt_size1, int filt_depth1, int filt_size2, int filt_depth2) :
mat (mat),
filt_size1(filt_size1),
filt_depth1(filt_depth1),
filt_size2(filt_size2),
filt_depth2(filt_depth2)
{}

cv::Mat HandsHeadExtractor::extractHandsHead()
{
    cv::Mat matDst = cv::Mat_<uint16_t>::zeros(mat.size());
    uint16_t* p_mat = (uint16_t*)(mat.data);
    uint16_t* p_matDst = (uint16_t*)(matDst.data);
    
    for(int y = 0; y < mat.rows; y++){
        for(int x = 0; x < mat.cols; ++x, ++p_mat, ++p_matDst){
            uint16_t val = *p_mat;
            if(!val)
                continue;
            bool isValue(true);
            
            for(int i = 1; i <= 2; i++){
                int depth = i == 1 ? filt_depth1 :  filt_depth2;
                int size = i == 1 ? filt_size1 : filt_size2;
                for(int m = -1; m <= 1; m++){
                    for(int n = -1; n <= 1; n++){
                        if(!m && !n)
                            continue;
                        int y_ = y + m * size;
                        int x_ = x + n * size;
                        uint16_t val_ = *((uint16_t*)(mat.data) + y_* mat.cols + x_);
                        if(val_ && val_ - val <= depth){
                            isValue = false;
                            break;
                        }
                         if(!isValue)
                             break;
                    }
                }
              if(!isValue)
                  break;
            }
            if(isValue)
                *p_matDst = val;
        }
    }
    return matDst;
}