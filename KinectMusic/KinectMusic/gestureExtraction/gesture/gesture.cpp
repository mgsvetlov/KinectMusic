//
//  Gesture.cpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 29/10/16.
//  Copyright © 2016 mgsvetlov. All rights reserved.
//

#include "gesture.h"
#include <cfloat>
#include <limits>

std::vector<Gesture> Gesture::gestures(2);

/*static*/
void Gesture::analyzeFrame(const std::list<Blob>& lBlobs){
    for(auto& gesture : gestures){
        gesture.isBlobFound = false;
    }
    std::vector<bool> vBlobsMatched(lBlobs.size(), false);
    std::vector<std::vector<double>> vvDists(gestures.size(), std::vector<double>(lBlobs.size()));
    for(int i = 0; i < gestures.size(); i++){
        int j = 0;
        for(auto it  = lBlobs.begin(); it != lBlobs.end(); it++, j++){
            vvDists[i][j] = gestures[i].dist2blob(*it);
        }
    }
    
    for(int i = 0; i < vvDists.size(); i++){
        double minDist (DBL_MAX);
        int ind (-1);
        auto it  = lBlobs.begin();
        auto itNearest  = lBlobs.begin();
        for(int j = 0; j < vvDists[i].size(); j++, it++){
            if(vvDists[i][j]< minDist){
                minDist = vvDists[i][j];
                itNearest = it;
                ind = j;
            }
        }
        if(ind != -1){
            gestures[i].addHandData(*itNearest);
            vBlobsMatched[ind] = true;
            //жадный алгоритм - для первого ближайшего
            for(int i1 = 0; i1 < gestures.size(); i1++){
                if(i1 == i)
                    continue;
                vvDists[i1][ind] = DBL_MAX;
            }
        }
    }
    int i (-1);
    for(auto& blob : lBlobs){
        if(vBlobsMatched[++i])
            continue;
        for(auto& gesture : gestures){
            if(!gesture.isBlobFound){
                gesture.addHandData(blob);
                break;
            }
        }
        
    }
    for(auto& gesture : gestures){
        if(!gesture.isBlobFound){
            gesture.lHandData.clear();
        }
    }
}


double Gesture::dist2blob(const Blob& blob){
    if(lHandData.empty())
        return DBL_MAX;
    auto& prevBlob = lHandData.back().blob;
    double dist = prevBlob.dist2blob(blob);
    return dist < 10 ? dist : DBL_MAX;
}

void Gesture::addHandData(const Blob& blob){
    int ind = blob.getCentralCell().ind;
    cv::Size size = blob.getMatSize();
    int x = ind % size.width;
    int y = (ind-x) /size.width;
    float z = 2. * (MAX_KINECT_DEPTH - blob.getCentralCell().val) / MAX_KINECT_DEPTH;
    lHandData.push_back(HandData(
                        static_cast<float>(x)/size.width,
                        static_cast<float>(y)/size.height,
                        z,
                        blob));
    isBlobFound = true;
}

