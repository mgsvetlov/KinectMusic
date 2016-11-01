//
//  mapping.cpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 30/10/16.
//  Copyright © 2016 mgsvetlov. All rights reserved.
//

#include "mapping.h"
#include "../sound/csound_.h"
#include "../gestureExtraction/gesture/gesture.h"

void Mapping::MapDirect(const std::vector<Gesture>& gestures){
    std::vector<std::vector<double>> data = { {0, 0}, {0, 0}};
    for(int i = 0; i < gestures.size(); i++){
        const std::list<HandData> lHandData = gestures[i].getLHandData();
        if(lHandData.empty()){
            data[i][0] = data[i][1] = 0;
            continue;
        }
        const HandData& handData = lHandData.back();
        data[i][0] = 110 + 440 * (1.0 - handData.y);
        data[i][1] = handData.z;
    }
    setPitchVol(data);
}

bool Mapping::setPitchVol (const std::vector<std::vector<double>>& data) {
    if(csound == nullptr)
        return false;
    for(int i = 0; i < 2; i++){
        for(int j = 0; j < 2; j++){
            if(j == 0 && data[i][j] == 0.)
                continue;
            csound_dataDst[i][j] = data[i][j];
        }
    }
    return true;
}
