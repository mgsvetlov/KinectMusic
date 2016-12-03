//
//  Gesture.cpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 29/10/16.
//  Copyright © 2016 mgsvetlov. All rights reserved.
//

#include "tracking.h"

#include <cfloat>
#include <limits>

std::vector<Tracking> Tracking::tracks(2);

/*static*/
void Tracking::analyzeFrame(const std::list<Hand>& lHands){
    for(auto& gesture : tracks){
        if(gesture.lHandData.size()  > 2)
            gesture.lHandData.pop_front();
        gesture.isHandFound = false;
    }
    std::vector<bool> vHandsMatched(lHands.size(), false);
    std::vector<std::vector<double>> vvDists(tracks.size(), std::vector<double>(lHands.size()));
    for(int i = 0; i < tracks.size(); i++){
        int j = 0;
        for(auto it  = lHands.begin(); it != lHands.end(); it++, j++){
            vvDists[i][j] = tracks[i].dist2hand(*it);
        }
    }
    
    for(int i = 0; i < vvDists.size(); i++){
        double minDist (DBL_MAX);
        int ind (-1);
        auto it  = lHands.begin();
        auto itNearest  = lHands.begin();
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
            //жадный алгоритм - для первого ближайшего
            for(int i1 = 0; i1 < tracks.size(); i1++){
                if(i1 == i)
                    continue;
                vvDists[i1][ind] = DBL_MAX;
            }
        }
    }
    int i (-1);
    for(auto& hand : lHands){
        if(vHandsMatched[++i])
            continue;
        for(auto& gesture : tracks){
            if(!gesture.isHandFound){
                gesture.addHandData(hand);
                break;
            }
        }
        
    }
    for(auto& gesture : tracks){
        if(!gesture.isHandFound){
            gesture.lHandData.clear();
        }
    }
}

double Tracking::dist2hand(const Hand& hand){
    if(lHandData.empty())
        return DBL_MAX;
    auto& prevHand = lHandData.back().hand;
    double dist = prevHand.dist2hand(hand);
    return dist < 80 ? dist : DBL_MAX;
}

void Tracking::addHandData(const Hand& hand){
    int x = hand.keyPoint.x;
    int y = hand.keyPoint.y;
    cv::Size size = hand.mat.size();
    float z = 2. * (MAX_KINECT_DEPTH - hand.keyPoint.z) / MAX_KINECT_DEPTH;
    lHandData.push_back(HandData(
                        static_cast<float>(x)/size.width,
                        static_cast<float>(y)/size.height,
                        z,
                        hand));
    isHandFound = true;
}

