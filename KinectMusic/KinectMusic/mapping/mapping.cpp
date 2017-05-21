//
//  mapping.cpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 30/10/16.
//  Copyright © 2016 mgsvetlov. All rights reserved.
//

#include "mapping.h"

#ifdef USE_CSOUND

#include "../sound/csound_.h"
#include "../gestureExtraction/gesture/gesture.h"

/*void Mapping::MapDirect(const std::vector<Gesture>& gestures){
    std::vector<std::vector<double>> data = { {0, 0}, {0, 0}};
    for(int i = 0; i < gestures.size(); i++){
        const std::list<HandData> lHandData = gestures[i].getLHandData();
        if(lHandData.empty()){
            data[i][0] = data[i][1] = 0;
            csound_dataDst[i][1] = 0;
            continue;
        }
        const HandData& handData = lHandData.back();
        if(lHandData.size() > 2 ){
            auto it = lHandData.rend();
            for(int j = 0; j < 2; j++)
                ++it;
            const HandData& handDataPrev = *it;
            if(handData.z - handDataPrev.z < 4e-2  ) {
               csound_dataDst[i][1] = 0;
                continue;
            }
            csound_dataDst[i][0] = 110 + 440 * (1.0 - handData.y);
            csound_dataDst[i][1] = 0.5;//handData.z;
        }
    
    }
    
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
}*/

#endif //USE_CSOUND
