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
    /*cv::Mat matImageRes;
    cv::resize(matImage, matImageRes, cv::Size(matImage.cols << BLOBS_RESIZE_POW, matImage.rows << BLOBS_RESIZE_POW));*/
    cv::imshow( "Display window", matImage);
    if(cv::waitKey(1) == 27) {
        return false;
    }
    
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




