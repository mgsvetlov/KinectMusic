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
    Hand(cv::Mat mat, int bbXY, const cv::Point3i& keyPoint);
    Hand extractHand() const;
private:
    void findBorderPoints();
    bool checkIsHand();
private:
    cv::Point2i refPoint;
    cv::Size size;
    std::list<cv::Point3i> lPoints;
    std::vector<std::vector<cv::Point3i>> vvPointsBorder;
    
    friend class Visualization;
    friend class HandsFromPoints;
};

#endif /* hand_h */
