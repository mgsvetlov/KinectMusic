//
//  Gesture.hpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 29/10/16.
//  Copyright © 2016 mgsvetlov. All rights reserved.
//

#ifndef tracking_h
#define tracking_h

#include <stdio.h>
#include "../types.h"
#include "../blobs/blobext.hpp"
#include "../processframedata.h"


class Track {
public:
    Track(){}
    static void analyzeFrame(std::list<BlobFinal>& lBlobs);
    static const std::vector<Track>& getTracksConst() {return tracks;}
    std::list<HandData>& getHandHistory()  { return handHistory;}
    const std::list<HandData>& getHandHistoryConst() const { return handHistory;}
    const bool getIsTrackFound() const  {return isHandFound;}
private:
    double dist2blob(const BlobFinal& blob);
    void addHandData(BlobFinal& blob);
private:
    static std::vector<Track> tracks;
    bool isHandFound = false;
    std::list<HandData> handHistory; //lHands
    
    friend class Visualization;
};

#endif /* tracking_h */
