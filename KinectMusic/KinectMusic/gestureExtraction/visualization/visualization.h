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
class Hand;
class Tracking;
struct HandData;

class Visualization {
private:
    Visualization();
public:
    static bool showImage();
    static void setMatImage(cv::Mat mat){matImage = mat.clone();}
    
    static void setIsNeedRedraw(bool val) {isNeedRedraw = val;}
    static bool getIsNeedRedraw() {return isNeedRedraw;}
    
    static void mat2img(cv::Mat mat, cv::Mat& matImg);
    static void hands2img(const std::list<Hand>& lHands, cv::Mat& matImg, bool drawKeyPoints = true);
    static void tracks2img(const std::vector<Tracking>& tracks, cv::Mat& matImg, bool drawKeyPoints = true);
    static void handsTrackedStreams2img(const std::vector<std::vector<HandData>>& handsTrackedStreams, cv::Mat& matImg, size_t length);
private:
    static void hand2img(const Hand& hand, cv::Mat& matImg, const cv::Scalar& color);
    static void keyPoint2img(const cv::Point3i& keyPoint, cv::Mat& matImg, const cv::Scalar& color, int size);
    
private:
    static Visualization* p_vis;
    static cv::Mat matImage;
    static bool isNeedRedraw;
};

#endif /* visualization_h */
