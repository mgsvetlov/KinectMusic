//
//  visualization.cpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 01/10/16.
//  Copyright © 2016 mgsvetlov. All rights reserved.
//
#include <ctime>
#include <stdio.h>
#include "visualization.h"
#include "../blobs/blob.h"
#include "../gesture/gesture.h"
#include "../analyze.h"

Visualization* Visualization::p_vis = nullptr;
cv::Mat Visualization::matImage;
bool Visualization::isNeedRedraw = false;

Visualization::Visualization() {
    cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );
    cv::moveWindow("Display window", 320, 10);
    p_vis = this;
}

bool Visualization::showImage() {
    pthread_mutex_lock(&visualisation_mutex);
    isNeedRedraw = false;
    pthread_mutex_unlock(&visualisation_mutex);
    
    if(p_vis == nullptr){
        Visualization();
    }
    //cv::Mat matImageRes;
    //cv::resize(matImage, matImageRes, cv::Size(matImage.cols << BLOBS_RESIZE_POW, matImage.rows << BLOBS_RESIZE_POW));
    cv::imshow( "Display window", matImage);
    if(cv::waitKey(1) == 27) {
        return false;
    }

#ifdef USE_CSOUND
    if(!CSOUND_START)
        CSOUND_START = true;
#endif //USE_CSOUND
    
    static int frameCount(0);
    static std::clock_t t = clock();
    if(frameNum % 30 == 0){
        std::clock_t next_t = clock();
        if(frameCount) {
            double elapsed_sec = double(next_t - t)/CLOCKS_PER_SEC;
            std::cout << "output fps " << 30./elapsed_sec  << std::endl;
        }
        t = next_t;
    }
    ++frameCount;
    return true;
}

#ifdef USE_CSOUND
cv::Mat Visualization::gestures2img_mark(const std::vector<Gesture>& gestures, const cv::Size& size){
    cv::Mat img  = cv::Mat::zeros(size, CV_8UC3);
  
    int gestureCount(0);
    for(auto& gesture : gestures){
        const std::list<HandData> lHandData = gesture.getLHandData();
        int frameCount(0);

        for(auto& handData : lHandData) {
            //std::cout<< "Hand data" << handData.x << " " << handData.y << " " << handData.z << "\n";
            int x = handData.x * size.width;
            int y = handData.y * size.height;
            double radius = size.width * 0.04;
            if(frameCount !=  lHandData.size() -1)
                radius *= 0.5;
            int num = frameCount == lHandData.size() -1 ? gestureCount :  gestureCount  + 2;
            cv::Scalar color( 255, 255, 255 );
            unsigned char col = handData.z * 255;
            switch(num){
                case 0:
                    color = cv::Scalar( 0, 0, col );
                    break;
                case 1:
                    color = cv::Scalar( 0, col, 0);
                    break;
                case 2:
                    color = cv::Scalar( col, col, 0 );
                    break;
                case 3:
                    color = cv::Scalar(0, col, col );
                    break;
                case 4:
                    color = cv::Scalar( col, 0, col );
                    break;
                case 5:
                    color = cv::Scalar( 0, 0, col );
                    break;
            }
            cv::circle( img, cv::Point( x, y), radius, color, -1, 8 );
            frameCount++;
        }
        gestureCount++;
    }
    
    cv::flip(img, img, 1);
    
    return img;
}
#endif //USE_CSOUND

cv::Mat Visualization::centralCells2img_mark(const std::list<Blob>& lBlobs, const cv::Size& size){
    cv::Mat img  = cv::Mat::zeros(size, CV_8UC3);

    int blobCount(0);
    for(auto& blob : lBlobs) {
        auto& centralCell = blob.getCentralCell();
        int x = centralCell.ind % size.width;
        int y = (centralCell.ind-x) /size.width;
        double radius = size.width * 0.04;
        int num = blob.getIsHandOpened()? blobCount + 2 :  blobCount;
        cv::Scalar color( 255, 255, 255 );
        unsigned char col = (MAX_KINECT_DEPTH -centralCell.val) * 255. / MAX_KINECT_DEPTH * 2;
        switch(num){
            case 0:
                color = cv::Scalar( 0, 0, col );
                break;
            case 1:
                color = cv::Scalar( 0, col, 0);
                break;
            case 2:
                color = cv::Scalar( col, 0, 0 );
                break;
            case 3:
                color = cv::Scalar( col, col, 0 );
                break;
            case 4:
                color = cv::Scalar( col, 0, col );
                break;
            case 5:
                color = cv::Scalar( 0, col, col );
                break;
        }
        cv::circle( img, cv::Point( x, y), radius, color, -1, 8 );
        blobCount++;
    }

    cv::flip(img, img, 1);
    
    return img;
}

cv::Mat Visualization::blobs2img_mark(const std::list<Blob>& lBlobs, const cv::Size& size) {
    cv::Mat r  = cv::Mat_<unsigned char>::zeros(size);
    cv::Mat g  = cv::Mat_<unsigned char>::zeros(size);
    cv::Mat b  = cv::Mat_<unsigned char>::zeros(size);
    int blobCount(0);
    for(auto& blob : lBlobs) {
        auto& lCells = blob.getLCellsConst();
        for(auto& cell : lCells) {
            int ind = cell.ind;
            unsigned char col = cell.val ? 255 - cell.val * 255. / MAX_KINECT_VALUE : 0;
            unsigned char* p_r = (unsigned char*)(r.data) + ind;
            unsigned char* p_g = (unsigned char*)(g.data) + ind;
            unsigned char* p_b = (unsigned char*)(b.data) + ind;
            
            int num = blob.getIsHandOpened()? blobCount + 2 :  blobCount;

            switch(num){
                case 0:
                    *p_r = 0;
                    *p_g = 0;
                    *p_b = col;
                    break;
                case 1:
                    *p_r = 0;
                    *p_g = col;
                    *p_b = 0;
                    break;
                case 2:
                    *p_r = col;
                    *p_g = 0;
                    *p_b = 0;
                    break;
                case 3:
                    *p_r = col;
                    *p_g = col;
                    *p_b = 0;
                    break;
                case 4:
                    *p_r = col;
                    *p_g = 0;
                    *p_b = col;
                    break;
                case 5:
                    *p_r = 0;
                    *p_g = 255;
                    *p_b = 255;
                    break;
            }
        }
        
        blobCount++;
    }
    
    std::vector<cv::Mat> channels;
    channels.push_back(b);
    channels.push_back(g);
    channels.push_back(r);
    
    cv::Mat img;
    cv::merge(channels, img);
    
    for(auto& blob : lBlobs) {
        Cell centralCell = blob.getCentralCell();
        int ind = centralCell.ind;
        int x = ind % size.width;
        int y = (ind - x) /size.width;
        double radius = size.width * 0.02;
        cv::circle( img, cv::Point( x, y), radius, cv::Scalar(0,0,255), -1, 8 );
    }
    
    cv::flip(img, img, 1);
    
    return img;
}

cv::Mat Visualization::mat2img(cv::Mat mat) {
    
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
            *p_r = 255 - d16 * 255. / MAX_KINECT_VALUE;
            *p_b = 0;
        }
        else {
            *p_r = 0;
            *p_b = 255;
        }
        p_r++, p_g++, p_b++, p_mat16++;
    }
    
    std::vector<cv::Mat> channels;
    channels.push_back(b);
    channels.push_back(g);
    channels.push_back(r);
    
    cv::Mat img;
    cv::merge(channels, img);
    cv::flip(img, img, 1);
    
    return img;
}

cv::Mat Visualization::matAndBlobs2img(cv::Mat mat, const std::list<Blob>& lBlobs){
    
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
            *p_r = 255 - d16 * 255. / MAX_KINECT_VALUE;
            *p_b = 0;
        }
        else {
            *p_r = 0;
            *p_b = 255;
        }
        p_r++, p_g++, p_b++, p_mat16++;
    }
    
    p_r = (unsigned char*)(r.data);
    p_g = (unsigned char*)(g.data);
    p_b = (unsigned char*)(b.data);
    int count(0);
    for(auto& blob : lBlobs) {
        auto lCells = blob.getLCellsConst();
        
        for(auto& cell : lCells){
            int ind = cell.ind;
            unsigned int color = 255 - cell.val * 255. / MAX_KINECT_VALUE;
            if(cell.border){
                *(p_r + ind) = 255;
                *(p_g + ind) = 255;
                *(p_b + ind) = 0;
            }
            else if(count == 0){
                *(p_r + ind) = 0;
                *(p_g + ind) = color;
                *(p_b + ind) = 0;
            }
            else {
                *(p_r + ind) = 0;
                *(p_g + ind) = color;
                *(p_b + ind) = color;
            }
        }
        count++;
    }
    
    std::vector<cv::Mat> channels;
    channels.push_back(b);
    channels.push_back(g);
    channels.push_back(r);
    
    cv::Mat img;
    cv::merge(channels, img);
    cv::flip(img, img, 1);
    
    return img;
}



