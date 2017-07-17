//
//  visualization.cpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 01/10/16.
//  Copyright © 2016 mgsvetlov. All rights reserved.
//
#include <unistd.h>
#include <stdio.h>
#include "visualization.h"
#include "../blobs/blob.h"
#include "../tracking/tracking.h"
#include "../analyze.h"
#include "../hand/hand.h"
#include "../gesture/gesturefabrique.h"
#include "../../log/logs.h"

Visualization* Visualization::p_vis = nullptr;
cv::Mat Visualization::matImage;
bool Visualization::isNeedRedraw = false;

Visualization::Visualization() {
    cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );
    cv::moveWindow("Display window", 20, -1000);
    //cv::resizeWindow("Display window", 640, 480);
    p_vis = this;
}

bool Visualization::showImage() {
    pthread_mutex_lock(&visualisation_mutex);
    if(!getIsNeedRedraw()){
        pthread_mutex_unlock(&visualisation_mutex);
        usleep(10);
        return true;
    }
    isNeedRedraw = false;
    if(p_vis == nullptr){
        Visualization();
    }
    cv::flip(matImage, matImage, 1);
    //cv::Mat matImageRes;
    //cv::resize(matImage, matImageRes, cv::Size(matImage.cols << 1, matImage.rows << 1));
    cv::imshow( "Display window", matImage);
    pthread_mutex_unlock(&visualisation_mutex);
    
    if(cv::waitKey(1) == 27) {
        return false;
    }
    return true;
}


void Visualization::mat2img(cv::Mat mat, cv::Mat& matImg) {
    
    int w = mat.cols;
    int h = mat.rows;
    cv::Mat r  = cv::Mat_<unsigned char>::zeros(cv::Size(w, h));
    cv::Mat g  = cv::Mat_<unsigned char>::zeros(cv::Size(w, h));
    cv::Mat b  = cv::Mat_<unsigned char>::zeros(cv::Size(w, h));
    uint16_t* p_mat16 = (uint16_t*)(mat.data);
    unsigned char* p_r = (unsigned char*)(r.data);
    unsigned char* p_g = (unsigned char*)(g.data);
    unsigned char* p_b = (unsigned char*)(b.data);
    for(int i = 0; i < w*h; i++) {
        uint16_t d16 = *p_mat16;
        
        if(d16 && d16 < MAX_KINECT_VALUE) {
            *p_b = *p_g =*p_r = 255 - d16 * 255. / MAX_KINECT_VALUE;
            //*p_b = 0;
        }
        /*else {
            *p_r = 0;
            *p_b = 255;
        }*/
        p_r++, p_g++, p_b++, p_mat16++;
    }
    
    std::vector<cv::Mat> channels;
    channels.push_back(b);
    channels.push_back(g);
    channels.push_back(r);
    
    cv::merge(channels, matImg);
}

void Visualization::blobs2img(const std::list<Blob>& lBlobs, cv::Mat& matImg, bool drawKeyPoints ){
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

void Visualization::gesture2img(const std::shared_ptr<Gesture>& gesture, cv::Mat& matImg, size_t length){
    int pointSize(5);
    cv::Scalar color = gesture->handInd == 0 ? cv::Scalar(0,255,255) : cv::Scalar(255,255, 0);
    auto rit = gesture->handsData.crbegin();
    if(length == 0 || gesture->handsData.size() < length)
        length = gesture->handsData.size();
    for(int i = 0; i < length; ++i, ++rit){
        const auto& handData = *rit;
        const auto& point = handData.point;
        if(point.x != NO_DATA_VALUE && handData.phase != NO_DATA_VALUE){
            keyPoint2img(point, matImg, color, pointSize);
        }
    }
}

void Visualization::gestures2img(const std::vector<std::shared_ptr<Gesture>>& gestures, cv::Mat& matImg, size_t length){
    for(auto& gesture : gestures) {
        gesture2img(gesture, matImg, length);
    }
}


void Visualization::blob2img(const Blob& blob, cv::Mat& matImg, const cv::Scalar& color, bool colorFromNormal){
    int minVal = blob.cells.MinValCell()->val;
    for(auto& cell : blob.cells.AllConst()){
        int col = 255 - (cell.val - minVal);
        if(col < 0)
            col = 0;
        cv::circle(matImg, cv::Point(cell.x, cell.y), 1, cv::Scalar (0.0f, col, col), -1);
    }
    for(auto& cell : blob.border1.AllConst()){
        cv::circle(matImg, cv::Point(cell.x, cell.y), 1, cv::Scalar (0.0f, 0.0f, 255), -1);
    }
    for(auto& cell : blob.border2.AllConst()){
        cv::circle(matImg, cv::Point(cell.x, cell.y), 1, cv::Scalar (255, 0.0f, 0), -1);
    }
    /*int j(0);
    for(auto& item : blob.subBlobs){
        int c = j % 3;
        cv::Scalar colorPoint = c == 0 ? cv::Scalar(255,0.0f, 0.0f) : c == 1 ? cv::Scalar(0.0f,255, 0.0f) : cv::Scalar(0.0f,0.0f, 255);
        ++j;
        for(auto& cell : item.vpCells){
            cv::circle(matImg, cv::Point(cell->x, cell->y), 1, colorPoint, -1);
        }
    }*/
}

void Visualization::keyPoint2img(const cv::Point3i& keyPoint, cv::Mat& matImg, const cv::Scalar& color, int size) {
    int x = keyPoint.x;
    int y = keyPoint.y;
    if(x < 0 || y < 0)
        return;
    cv::circle(matImg, cv::Point(x, y), size,  color, -1);
}

void Visualization::drawText(cv::Mat& mat, std::string text, double fontScale, int thickness, cv::Scalar color, cv::Point2f textCenter)
{
    int fontFace = cv::FONT_HERSHEY_SIMPLEX;
    
    int baseline = 0;
    cv::Size textSize = cv::getTextSize(text, fontFace,
                                fontScale, thickness, &baseline);
    baseline += thickness;
    
    // center the text
    cv::Point textOrg((mat.cols - textSize.width) * textCenter.x,
                  (mat.rows + textSize.height) * textCenter.y);
    
    cv::putText(mat, text, textOrg, fontFace, fontScale,
            color, thickness, 8);
}
