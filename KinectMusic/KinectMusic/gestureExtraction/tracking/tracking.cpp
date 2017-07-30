//
//  Gesture.cpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 29/10/16.
//  Copyright © 2016 mgsvetlov. All rights reserved.
//

#include "tracking.h"
#include "../params.h"

#include <cfloat>
#include <limits>

std::vector<Track> Track::tracks;


void Track::analyzeFrame(std::list<BlobFinal>& blobs){
    if(tracks.empty())
        tracks = std::vector<Track>(Params::getTracksCount());
    for(auto& track : tracks){
        if(track.handHistory.size()  > 2)
            track.handHistory.pop_front();
        track.isHandFound = false;
    }
    std::vector<bool> vHandsMatched(blobs.size(), false);
    std::vector<std::vector<double>> vvDists(tracks.size(), std::vector<double>(blobs.size()));
    for(int i = 0; i < tracks.size(); i++){
        int j = 0;
        for(auto it  = blobs.begin(); it != blobs.end(); it++, j++){
            vvDists[i][j] = tracks[i].dist2blob(*it);
        }
    }
    //nearest neighbour search
    for(int i = 0; i < vvDists.size(); i++){
        double minDist (DBL_MAX);
        int ind (-1);
        auto it  = blobs.begin();
        auto itNearest  = blobs.begin();
        for(int j = 0; j < vvDists[i].size(); j++, it++){
            if(vvDists[i][j]< minDist){
                minDist = vvDists[i][j];
                itNearest = it;
                ind = j;
            }
        }
        if(ind != -1){
            tracks[i].addHandData(*itNearest);
            vHandsMatched[ind] = true;
            //greedy algorithm - for the first nearest neighbour
            for(int i1 = 0; i1 < tracks.size(); i1++){
                if(i1 == i)
                    continue;
                vvDists[i1][ind] = DBL_MAX;
            }
        }
    }
    //if neighbour isn't found
    int i (-1);
    for(auto& blob : blobs){
        if(vHandsMatched[++i])
            continue;
        for(auto& track : tracks){
            if(!track.isHandFound){
                track.addHandData(blob);
                break;
            }
        }
    }
    //data cleaning 
    for(auto& track : tracks){
        if(!track.isHandFound){
            track.handHistory.clear();
        }
    }
}

double Track::dist2blob(const BlobFinal& blob){
    if(handHistory.empty())
        return DBL_MAX;
    auto& prevHand = handHistory.back();
    const auto& minValCell = blob.getCellsConst().MinValCell();
    int dx = prevHand.keyPoint.x - minValCell->x;
    int dy = prevHand.keyPoint.y - minValCell->y;
    double dist = sqrt(dx*dx+dy*dy);
    return dist < Params::getTrackingDistThresh() ? dist : DBL_MAX;
}

void Track::addHandData(BlobFinal& blob){
    cv::Point3i averagedMinPoint = blob.getCells().AveragedMinPoint(1000);
    handHistory.push_back(HandData(averagedMinPoint));
    //const auto& minValCell = blob.getCellsConst().MinValCell();
    //handHistory.emplace_back(HandData(cv::Point3i(minValCell->x, minValCell->y, minValCell->val)));
    isHandFound = true;
}

