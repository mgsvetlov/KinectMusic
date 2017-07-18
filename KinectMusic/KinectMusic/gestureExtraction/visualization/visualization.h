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

#include "../blobs/blob.hpp"

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

    template<typename T> static void blobs2img(const std::list<Blob<T>>& lBlobs, cv::Mat& matImg, bool drawKeyPoints = true);

    //static void gestures2img(const std::vector<std::shared_ptr<Gesture>>& handsTrackedStreams, cv::Mat& matImg, size_t length = 0);
private:

    template<typename T> static void blob2img(const Blob<T>& blob, cv::Mat& matImg, const cv::Scalar& color, bool colorFromNormal = false);
    //static void gesture2img(const std::shared_ptr<Gesture>& gesture, cv::Mat& matImg, size_t length = 0);
    static void keyPoint2img(const cv::Point3i& keyPoint, cv::Mat& matImg, const cv::Scalar& color, int size);
    static void drawText(cv::Mat& mat, std::string text, double fontScale, int thickness, cv::Scalar color, cv::Point2f textCenter);
private:
    static Visualization* p_vis;
    static cv::Mat matImage;
    static bool isNeedRedraw;
};

#include <unistd.h>
#include <stdio.h>
//#include "visualization.h"
#include "../blobs/blob.hpp"
#include "../tracking/tracking.h"
#include "../analyze.h"
#include "../hand/hand.h"
#include "../gesture/gesturefabrique.h"
#include "../../log/logs.h"


template<typename T> void Visualization::blobs2img(const std::list<Blob<T>>& lBlobs, cv::Mat& matImg, bool drawKeyPoints ){
    for(const auto& blob : lBlobs){
        cv::Scalar color = blob.angle > 0? cv::Scalar(0,0.5,0) : cv::Scalar(0.5,0,0);
        blob2img(blob, matImg, color, true);
        if(drawKeyPoints) {
            int x = blob.cells.MinValCell()->x;
            int y = blob.cells.MinValCell()->y;
            int z = blob.cells.MinValCell()->val;
            keyPoint2img(cv::Point3i(x, y, z), matImg, cv::Scalar(127, 0, 0), 3);
        }
    }
}


template<typename T> void Visualization::blob2img(const Blob<T>& blob, cv::Mat& matImg, const cv::Scalar& color, bool colorFromNormal){
    int minVal = blob.cells.MinValCell()->val;
    for(auto& cell : blob.cells.AllConst()){
        int col = 255 - (cell.val - minVal);
        if(col < 0)
            col = 0;
        cv::circle(matImg, cv::Point(cell.x, cell.y), 1, cv::Scalar (0.0f, col, col), -1);
    }
    for(auto& cell : blob.border1.AllConst()){
        auto& parentCell = cell.parent;
        cv::circle(matImg, cv::Point(parentCell->x, parentCell->y), 1, cv::Scalar (0.0f, 255.0f, 0), -1);
        cv::circle(matImg, cv::Point(cell.x, cell.y), 1, cv::Scalar (0.0f, 0.0f, 255), -1);
    }
    for(auto& cell : blob.border2.AllConst()){
        auto& parentCell = cell.parent;
        cv::circle(matImg, cv::Point(parentCell->x, parentCell->y), 1, cv::Scalar (255, 255.0f, 0), -1);
        cv::circle(matImg, cv::Point(cell.x, cell.y), 1, cv::Scalar (255, 0.0f, 0), -1);
    }
}


#endif /* visualization_h */
