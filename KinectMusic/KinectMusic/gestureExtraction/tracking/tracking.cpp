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
    for(auto& track : tracks){
        if(track.lHands.size()  > 2)
            track.lHands.pop_front();
        track.isHandFound = false;
    }
    std::vector<bool> vHandsMatched(lHands.size(), false);
    std::vector<std::vector<double>> vvDists(tracks.size(), std::vector<double>(lHands.size()));
    for(int i = 0; i < tracks.size(); i++){
        int j = 0;
        for(auto it  = lHands.begin(); it != lHands.end(); it++, j++){
            vvDists[i][j] = tracks[i].dist2hand(*it);
        }
    }
    //поиск ближайшего соседа и добавление к нему
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
    //добавляются те, для которых не найдено ближайшего
    int i (-1);
    for(auto& hand : lHands){
        if(vHandsMatched[++i])
            continue;
        for(auto& track : tracks){
            if(!track.isHandFound){
                track.addHandData(hand);
                break;
            }
        }
        
    }
    //очистка данных, которым не найдено продолжения
    for(auto& track : tracks){
        if(!track.isHandFound){
            track.lHands.clear();
        }
    }
}

double Tracking::dist2hand(const Hand& hand){
    if(lHands.empty())
        return DBL_MAX;
    auto& prevHand = lHands.back();
    double dist = prevHand.dist2hand(hand);
    return dist < 80 ? dist : DBL_MAX;
}

void Tracking::addHandData(const Hand& hand){
    lHands.push_back(hand);
    isHandFound = true;
}

