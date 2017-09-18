//
//  integralimage.cpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 17/09/2017.
//  Copyright © 2017 mgsvetlov. All rights reserved.
//

#include "integralimage.h"

IntegralImage::IntegralImage(cv::Mat mat_, size_t resizePow):
resizePow(resizePow),
mat(mat_)
{
    if(resizePow){
        mat = mat.clone();
        cv::resize(mat, mat, cv::Size(mat.cols >> resizePow, mat.rows >> resizePow));
    }
    cv::Mat matFloat(mat.rows, mat.cols, cv::DataType<float>::type);
    uint16_t* p_mat = (uint16_t*)(mat.data);
    float* p_matFloat = (float*)(matFloat.data);
    for(int i = 0; i < mat.total(); ++i)
        *p_matFloat++ = static_cast<float>(*p_mat++);
    cv::integral(matFloat, matIntegral, CV_32F);
    cv::resize(matIntegral, matIntegral, mat.size());
    
}
