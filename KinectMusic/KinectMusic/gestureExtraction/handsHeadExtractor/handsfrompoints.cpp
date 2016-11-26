//
//  handsfrompoints.cpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 12/11/16.
//  Copyright © 2016 mgsvetlov. All rights reserved.
//

#include "handsfrompoints.h"
#include <queue>
#include "../hand/hand.h"

HandsFromPoints::HandsFromPoints(cv::Mat mat, const std::list<Blob>& lBlobs, int bbXY, int bbZ) :
mat(mat),
lBlobs(lBlobs),
bbXY(bbXY),
bbZ(bbZ)
{
    
}

std::list<Hand> HandsFromPoints::extractHandBlobs()
{
    std::list<Hand> lHands;
    for(auto& blob : lBlobs) {
        Blob handBlob;
        handBlob.setMatSize(mat.size());
        Cell keyCell = blob.centralCell;
        int ind = keyCell.ind;
        int x = ind % blob.getMatSize().width;
        int y = (ind - x) /blob.getMatSize().width;
        x <<= BLOBS_RESIZE_POW;
        y <<= BLOBS_RESIZE_POW;
        int z = keyCell.val;
        Hand hand(mat, bbXY, cv::Point3i(x,y,z));
        Hand handExtracted = hand.extractHand();
        if(!handExtracted.lPoints.empty())
            lHands.push_back(handExtracted);
    }
    
    return lHands;
}
