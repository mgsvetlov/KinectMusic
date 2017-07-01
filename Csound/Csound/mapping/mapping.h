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
#include <string>
#include <map>

#define NO_DATA_VALUE -1
#define START_GESTURE_VALUE -10
#define INSIDE_GESTURE_VALUE -100
#define END_GESTURE_VALUE -1000

struct HandData{
    int phase;
    double x;
    double y;
    int z;
    int angle = -1;
    HandData() : z(-1){}
    HandData(int phase, double x, double y, int z) :
    phase(phase), x(x), y(y), z(z){}
};

struct FrameData{
    int frameNum;
    int bodyDepth;
    std::vector<HandData> hands = std::vector<HandData>(2);
};

struct ParamData {
    //std::string name;
    double param;
    double ramp;
    ParamData(){}
    ParamData(double param, double ramp) :
                param(param), ramp(ramp) {}
};

class Mapping{
public:
    FrameData& getData() { return frameData; }
    const std::map<std::string,ParamData>& getCsound_data() const { return  csound_data;}
    const std::string& getCsdName() const {return csdName;}
    virtual ~Mapping(){};
    virtual void initialScoreEvents() = 0;
    virtual void mappingData() = 0;

protected:
    std::string csdName;
    int frameNum = 0;
    FrameData frameData;
    std::map<std::string,ParamData> csound_data;
    
};
#endif /* mapping_hpp */
