//
//  hand.cpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 26/11/16.
//  Copyright © 2016 mgsvetlov. All rights reserved.
//
/*
#include "hand.h"

Hand::Hand(Blob& blob){
    int x = blob.cells.MinValCell()->x;
    int y = blob.cells.MinValCell()->y;
    int z = blob.cells.MinValCell()->val;
    keyPoint = cv::Point3i(x,y,z);
    angle = blob.angle;
    p_blob = &blob;
}

double Hand::dist2hand(const Hand& hand) const{
    int dx = keyPoint.x - hand.keyPoint.x;
    int dy = keyPoint.y - hand.keyPoint.y;
    return sqrt(dx*dx+dy*dy);
}
*/