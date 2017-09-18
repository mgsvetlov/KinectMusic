//
//  visualization.h
//  KinectMusic
//
//  Created by Mikhail Svetlov on 01/10/16.
//  Copyright © 2016 mgsvetlov. All rights reserved.
//

#ifndef visualization_h
#define visualization_h

#include <memory>
#include "../types.h"

#include "../blobs/blobext.hpp"
#include <unistd.h>
#include "../tracking/tracking.h"
#include "../extractframedata.h"
#include "../gesture/gesturefabrique.h"
#include "../../log/logs.h"

class Hand;
struct HandData;
class Gesture;

class Visualization {
private:
    Visualization();
public:
    static bool showImage();
    static void setMatImage(cv::Mat mat){matImage = mat.clone();}
    
    static void setIsNeedRedraw(bool val) {isNeedRedraw = val;}
    static bool getIsNeedRedraw() {return isNeedRedraw;}
    
    static void mat2img(cv::Mat mat, cv::Mat& matImg);

    static void blobs2img(const std::list<BlobFinal>& lBlobs, cv::Mat& matImg, bool drawKeyPoints = true);
    
    static void tracks2img(const std::vector<Track>& tracks, cv::Mat& matImg);
    //static void gestures2img(const std::vector<std::shared_ptr<Gesture>>& handsTrackedStreams, cv::Mat& matImg, size_t length = 0);
     static void drawText(cv::Mat& mat, std::string text, double fontScale, int thickness, cv::Scalar color, cv::Point2f textCenter);
    
    static void vecs2img(cv::Mat& matVecs, cv::Mat& matImg);
    
    static void integralFeatures2img(const std::vector<std::vector<cv::Rect>>& integralFeatures, cv::Mat& matImg);
private:

    static void blob2img(const BlobFinal& blob, cv::Mat& matImg, const cv::Scalar& color, bool colorFromNormal = false);
    //static void gesture2img(const std::shared_ptr<Gesture>& gesture, cv::Mat& matImg, size_t length = 0);
    static void keyPoint2img(const cv::Point3i& keyPoint, cv::Mat& matImg, const cv::Scalar& color, int size);
   
private:
    static Visualization* p_vis;
    static cv::Mat matImage;
    static bool isNeedRedraw;
};







#endif /* visualization_h */
