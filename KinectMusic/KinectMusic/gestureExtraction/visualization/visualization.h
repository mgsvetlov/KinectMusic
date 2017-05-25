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

class Blob;
class Hand;
class Track;
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
    static void hands2img(const std::vector<Track>& tracks, cv::Mat& matImg, bool drawKeyPoints = true);
    static void gestures2img(const std::vector<std::shared_ptr<Gesture>>& handsTrackedStreams, cv::Mat& matImg, size_t length);
private:
    static void hand2img(const Hand& hand, cv::Mat& matImg, const cv::Scalar& color);
    static void gesture2img(const std::shared_ptr<Gesture>& gesture, cv::Mat& matImg);
    static void keyPoint2img(const cv::Point3i& keyPoint, cv::Mat& matImg, const cv::Scalar& color, int size);
    static void drawText(cv::Mat& mat, std::string text, double fontScale, int thickness, cv::Scalar color, cv::Point2f textCenter);
private:
    static Visualization* p_vis;
    static cv::Mat matImage;
    static bool isNeedRedraw;
};

#endif /* visualization_h */
