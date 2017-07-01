//
//  hand.h
//  KinectMusic
//
//  Created by Mikhail Svetlov on 26/11/16.
//  Copyright © 2016 mgsvetlov. All rights reserved.
//

#ifndef hand_h
#define hand_h
#include "../blobs/blob.h"

class Hand {
public:
    Hand(){}
    Hand(const Blob& blob, const cv::Size& matSize);

    const cv::Point3i& getKeyPoint() const {return keyPoint;}
private:
    double dist2hand(const Hand& hand) const;
private:
    cv::Point3i keyPoint;

    
    friend class Visualization;
    friend class Track;
};

#endif /* hand_h */
