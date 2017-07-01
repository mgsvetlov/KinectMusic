//
//  hand.cpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 26/11/16.
//  Copyright © 2016 mgsvetlov. All rights reserved.
//

#include "hand.h"

Hand::Hand(const Blob& blob, const cv::Size& matSize){
    auto centralCell = blob.getCentralCell();
    int ind = centralCell.ind;
    int w = matSize.width;
    int x = ind % w;
    int y = (ind-x) / w;
    int z = centralCell.val;
    keyPoint = cv::Point3i(x<<BLOBS_RESIZE_POW,y<<BLOBS_RESIZE_POW,z);
    angle = blob.angle;
}

double Hand::dist2hand(const Hand& hand) const{
    int dx = keyPoint.x - hand.keyPoint.x;
    int dy = keyPoint.y - hand.keyPoint.y;
    return sqrt(dx*dx+dy*dy);
}