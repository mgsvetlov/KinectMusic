//
//  handsfrompoints.cpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 12/11/16.
//  Copyright © 2016 mgsvetlov. All rights reserved.
//

#include "handsfrompoints.h"
#include <queue>

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
        Blob handBlob;
        handBlob.setMatSize(mat.size());
        Cell keyCell = blob.centralCell;
        int ind = keyCell.ind;
        int x = ind % blob.getMatSize().width;
        int y = (ind - x) /blob.getMatSize().width;
        x <<= BLOBS_RESIZE_POW;
        y <<= BLOBS_RESIZE_POW;
        int z = keyCell.val;
        cv::Mat matClone = mat.clone();
        uint16_t* p_mat = (uint16_t*)(matClone.data);
        
        for(int i = 0; i < mat.total(); i++, p_mat++)
        {
            int x_ = i % mat.cols;
            int y_ = (i - x_)/mat.cols;
            int z_ = *p_mat;
            if(abs(x - x_)<= bbXY &&
               abs(y - y_)<= bbXY &&
               abs(z - z_)<= MAX_NEIGHB_DIFF_COARSE * 2){
                handBlob.lCells.push_back(Cell(i, z_));
            }
        }
        int keyCellInd = y*mat.cols + x;
        p_mat = (uint16_t*)(matClone.data);
        if(checkIsHand(handBlob, Cell(keyCellInd, *(p_mat+keyCellInd)))&& !handBlob.lCells.empty())
            lHandBlobs.push_back(handBlob);
    }

    return lHandBlobs;
}

bool HandsFromPoints::checkIsHand(Blob& blob, Cell keyCell){
    std::list<Blob> lBlobsSegm = blob.segmentation();
    
    Blob blobUnited;
    for(auto& blobSegm : lBlobsSegm){
        findBorderPoints(blobSegm, keyCell);
        int counts[] = {0,0,0,0};
        for(auto& cell : blobSegm.lCells){
            if(cell.border)
                counts[cell.border-1]++;
        }
        int sum(0);
        for(int i = 0; i < 4; i++){
            sum += counts[i];
        }
        if(blobSegm.centralCell.ind == -1){
            if(sum)
                continue;
        }
        else if (sum > bbXY * 2){
            return false;
        }
        
        for(auto& cell : blobSegm.lCells)
            blobUnited.lCells.push_back(cell);
    }
    blob = blobUnited;
    return true;

}

void HandsFromPoints::findBorderPoints(Blob& blob, Cell keyCell){
    blob.centralCell = Cell(-1, -1);
    int x = keyCell.ind % mat.cols;
    int y = (keyCell.ind - x)/mat.cols;
    for(auto& cell : blob.lCells){
        int ind = cell.ind;
        int x_ = ind % mat.cols;
        int y_ = (ind - x_)/mat.cols;
        if(x - x_ == bbXY)
            cell.border = 1;
        else if(-x + x_ == bbXY)
            cell.border = 2;
        else if(y - y_ == bbXY)
            cell.border = 3;
        else if(-y + y_ == bbXY)
            cell.border = 4;
        if(ind == keyCell.ind)
            blob.centralCell = keyCell;
    }
}
