//
//  visualization.cpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 01/10/16.
//  Copyright © 2016 mgsvetlov. All rights reserved.
//

#include <stdio.h>
#include "visualization.h"
#include "../blobs/blob.h"

Visualization* Visualization::p_vis = nullptr;

Visualization::Visualization() {
    cv::namedWindow( "Display window", cv::WINDOW_AUTOSIZE );
    cv::moveWindow("Display window", 300, 10);
    p_vis = this;
}

void Visualization::visualize(cv::Mat mat) {
    
    if(p_vis == nullptr){
        Visualization();
    }
    
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
        if(d16 < MAX_KINECT_VALUE) {
            *p_r = d16 >> 2;
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
    cv::imshow( "Display window", img);
    
}

void Visualization::visualizeMap(const cv::Size& size, const cv::Size& fullSize, const std::list<Blob>& lBlobs){
    if(p_vis == nullptr) {
        Visualization();
    }

    cv::Mat r  = cv::Mat_<unsigned char>::zeros(size);
    cv::Mat g  = cv::Mat_<unsigned char>::zeros(size);
    cv::Mat b  = cv::Mat_<unsigned char>::zeros(size);
    int blobCount(0);
    for(auto& blob : lBlobs) {
        auto& lCells = blob.getLCellsConst();
        for(auto& cell : lCells) {
            int ind = cell.ind;
            int col = cell.val >> 2;
            unsigned char* p_r = (unsigned char*)(r.data) + ind;
            unsigned char* p_g = (unsigned char*)(g.data) + ind;
            unsigned char* p_b = (unsigned char*)(b.data) + ind;
            int num = blobCount < 3 ? blobCount : (blobCount % 3) + 3;
            switch(num){
                case 0:
                    *p_r = col;
                    *p_g = 0;
                    *p_b = 0;
                    break;
                case 1:
                    *p_r = 0;
                    *p_g = col;
                    *p_b = 0;
                    break;
                case 2:
                    *p_r = 0;
                    *p_g = 0;
                    *p_b = col;
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
    
    cv::resize(r, r, fullSize);
    cv::resize(g, g, fullSize);
    cv::resize(b, b, fullSize);
    
    std::vector<cv::Mat> channels;
    channels.push_back(b);
    channels.push_back(g);
    channels.push_back(r);
    
    cv::Mat img;
    cv::merge(channels, img);
    cv::flip(img, img, 1);
    
    cv::imshow( "Display window", img);
    
}

