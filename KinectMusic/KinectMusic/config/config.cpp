//
//  config.cpp
//  KinectMusic
//
//  Created by Mikhail Svetlov on 05/06/17.
//  Copyright © 2017 mgsvetlov. All rights reserved.
//

#include "config.h"
#include "../log/logs.h"
#include <iostream>
#include <fstream>
#include <sstream>

std::string Config::fileName = "../../../config.ini";
Config* Config::_self = nullptr;

std::map<std::string, double> Config::varmap =
{
    { "depthFormat", 0 },
    { "matrixWidth", 640 },
    { "isVisualisation", 0. },
    { "isCsound", 1. },
    { "gestureType", 0. }
};

Config::Config() :
depthFormat(static_cast<int>(varmap["depthFormat"])),
matrixWidth(static_cast<int>(varmap["matrixWidth"])),
isVisualisation(static_cast<bool>(varmap["isVisualisation"])),
isCsound(static_cast<bool>(varmap["isCsound"])),
gestureType(static_cast<int>(varmap["gestureType"]))
{}

bool Config::parse()
{
    std::ifstream fconf;
    fconf.open(fileName);
    if (fconf.fail()) {
        std::stringstream ss;
        ss <<"Can't find configuration file " << fileName << "\nDefault values will be set.\n";
        Logs::writeLog("gestures", ss.str());
   
        varmap["matrixWidth"] = 640;
        varmap["isVisualisation"] = 0.;
        varmap["isCsound"] = 1.;
        varmap["gestureType"] = 0.;
    }
    else {
        std::map<std::string, int> confmap;
        while (!fconf.eof())
        {
            std::string k; //varname as key
            std::string e; //=
            double v; //value
            std::string c; //comment
            
            fconf >> k;
            while (k[0] == '[' && !fconf.eof()) fconf >> k;
            if (fconf.eof()) break;
            if (++confmap[k] > 1)
            {
                std::stringstream ss;
                ss << "CONFIG PARSE ERROR\nParameter " << k << " is found twice.\n";
                Logs::writeLog("gestures", ss.str());
                fconf.close();
                return false;
            }
            
            fconf >> e;
            if (e != "=")
            {
                std::stringstream ss;
                ss << "CONFIG PARSE ERROR\nSyntax error, '=' expected for " << k << ".\n";
                Logs::writeLog("gestures", ss.str());
                fconf.close();
                return false;
            }
            
            fconf >> v;
            if (fconf.fail())
            {
                std::stringstream ss;
                ss << "CONFIG PARSE ERROR\nSyntax error, value expected for " << k << ".\n";
                Logs::writeLog("gestures", ss.str());
                fconf.close();
                return false;
            }
            
            auto it = varmap.find(k);
            if (it != varmap.end()) it->second = v;
            
            char x = fconf.get();
            while ((x == ' ' || x == '\t') && !fconf.eof()) x = fconf.get();
            if (x == '\n') continue;
            else if (x == '/') getline(fconf, c);
            else
            {
                std::stringstream ss;
                ss << "CONFIG PARSE ERROR\nSyntax error, only //comments allowed after value for " << k << ".\n";
                Logs::writeLog("gestures", ss.str());
                fconf.close();
                return false;
            }
        }
        fconf.close();
    }
    return true;
}


