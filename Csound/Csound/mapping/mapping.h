//
//  mapping.hpp
//  Csound
//
//  Created by Mikhail Svetlov on 28/05/17.
//  Copyright © 2017 mgsvetlov. All rights reserved.
//

#ifndef mapping_h
#define mapping_h
#include <vector>

#define NO_DATA_VALUE -1
#define START_GESTURE_VALUE -10
#define INSIDE_GESTURE_VALUE -100
#define END_GESTURE_VALUE -1000

struct HandData{
    int phase;
    double x;
    double y;
    int z;
    HandData(){}
    HandData(int phase, double x, double y, int z) :
    phase(phase), x(x), y(y), z(z){}
};

struct FrameData{
    int frameNum;
    int bodyDepth;
    std::vector<HandData> hands = std::vector<HandData>(2);
};

struct ParamData {
    double param;
    double rampCoeff;
    ParamData(double param, double rampCoeff) :
    param(param), rampCoeff(rampCoeff) {}
};

class Mapping{
public:
    FrameData& getData() { return frameData; }
    const std::vector<std::vector<ParamData>>& getCsound_data() const { return  csound_data;}
    const std::string& getCsdName() const {return csdName;}
    virtual ~Mapping(){};
    virtual void mappingData() = 0;
    
protected:
    int frameNum = 0;
    FrameData frameData;
    std::vector<std::vector<ParamData>> csound_data;
    std::string csdName;
    std::vector<HandData> startGestureData;
};
#endif /* mapping_hpp */
