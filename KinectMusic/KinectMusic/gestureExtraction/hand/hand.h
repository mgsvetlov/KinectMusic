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
    const cv::Point3i& getKeyPoint() const {return keyPoint;}
private:
    void findBorderPoints();
    bool checkIsHand();
    double dist2hand(const Hand& hand) const;
private:
    cv::Point3i keyPoint;
    cv::Mat mat;
public:
    cv::Point2i refPoint;
private:
    cv::Size size;
public:
    std::list<cv::Point3i> lPoints;
private:
    std::vector<std::vector<cv::Point3i>> vvPointsBorder;
    
    static cv::Mat matHand, matHandOrig;
    
    friend class Visualization;
    friend class HandsFromPoints;
    friend class Track;
};

#endif /* hand_h */
