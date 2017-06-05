//
//  config.hpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 05/06/17.
//  Copyright © 2017 mgsvetlov. All rights reserved.
//

#ifndef config_h
#define config_h
#include <string>
#include <map>

class Config
{
public:
    static Config* instance()
    {
        if(!Config::_self){
            if(!Config::parse())
                return nullptr;
            Config::_self = new Config();
        }
        return Config::_self;
    }
    static bool destroy()
    {
        if(Config::_self){
            delete Config::_self;
            Config::_self = nullptr;
            return true;
        }
        return false;
    }
    
protected:
    Config();
    virtual ~Config(){ };
public:
    static void setFileName(std::string value) {fileName = value;}
    
    double getThereminMidiMin() const {return thereminMidiMin; }
    double getThereminMidiMax() const {return thereminMidiMax; }
    
private:
    static bool parse();
    
private:
    static std::string fileName;
    static Config* _self;
    static std::map<std::string, double> varmap;
    
    double thereminMidiMin;
    double thereminMidiMax;
};

#endif /* config_h */
