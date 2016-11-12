//
//  handsfrompoints.cpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 12/11/16.
//  Copyright © 2016 mgsvetlov. All rights reserved.
//

#include "handsfrompoints.h"

HandsFromPoints::HandsFromPoints(cv::Mat mat, const std::list<Blob>& lBlobs, int bbXY, int bbZ) :
mat(mat),
lBlobs(lBlobs),
bbXY(bbXY),
bbZ(bbZ)
{
    
}

std::list<Blob> HandsFromPoints::extractHandBlobs()
{
    std::list<Blob> lHandBlobs;
    for(auto& blob : lBlobs) {
        lHandBlobs.push_back(Blob());
        lHandBlobs.back().setMatSize(mat.size());
        Cell centralCell = blob.getCentralCell();
        int ind = centralCell.ind;
        int x = ind % blob.getMatSize().width;
        int y = (ind - x) /blob.getMatSize().width;
        x <<= BLOBS_RESIZE_POW;
        y <<= BLOBS_RESIZE_POW;
        int z = centralCell.val;
        uint16_t* p_mat = (uint16_t*)(mat.data);
        for(int i = 0; i < mat.total(); i++, p_mat++)
        {
            int x_ = i % mat.cols;
            int y_ = (i - x_)/mat.cols;
            int z_ = *p_mat;
            if(abs(x - x_)<= bbXY &&
               abs(y - y_)<= bbXY &&
               abs(z - z_)<= bbZ){
                lHandBlobs.back().lCells.push_back(Cell(i, z_));
            }
        }
    }
    return lHandBlobs;
}