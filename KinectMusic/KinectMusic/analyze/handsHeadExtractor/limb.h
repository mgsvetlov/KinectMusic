//
//  limbsextractor.hpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 22/10/16.
//  Copyright © 2016 mgsvetlov. All rights reserved.
//

#ifndef limb_h
#define limb_h

#include <stdio.h>
#include "../types.h"
#include "../blobs/blob.h"

class Limb : public Blob{
    Limb(cv::Mat mat, const Blob&);
public:
    static void findLimbs(cv::Mat mat, const std::list<Blob>& lvBlobs, std::list<Limb>& lvLimbs);

    
};

#endif /* limb_h */
