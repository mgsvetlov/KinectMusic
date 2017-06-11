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
    { "isLogCsound", 0. },
    {"mappingType", 0. },
    { "thereminMidiMin", 60. },
    { "thereminMidiMax", 90. }
};

Config::Config() :
isLogCsound(varmap["isLogCsound"]),
mappingType(varmap["mappingType"]),
thereminMidiMin(varmap["thereminMidiMin"]),
thereminMidiMax(varmap["thereminMidiMax"])
{
}

bool Config::parse()
{
    std::ifstream fconf;
    fconf.open(fileName);
    if (fconf.fail()) {
        std::stringstream ss;
        ss <<"Can't find configuration file " << fileName << "\nDefault values will be set.\n";
        Logs::writeLog("csound", ss.str());
        
        varmap["thereminMidiMin"] = 60.;
        varmap["thereminMidiMax"] = 90.;
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
                Logs::writeLog("csound", ss.str());
                fconf.close();
                return false;
            }
            
            fconf >> e;
            if (e != "=")
            {
                std::stringstream ss;
                ss << "CONFIG PARSE ERROR\nSyntax error, '=' expected for " << k << ".\n";
                Logs::writeLog("csound", ss.str());
                fconf.close();
                return false;
            }
            
            fconf >> v;
            if (fconf.fail())
            {
                std::stringstream ss;
                ss << "CONFIG PARSE ERROR\nSyntax error, value expected for " << k << ".\n";
                Logs::writeLog("csound", ss.str());
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
                Logs::writeLog("csound", ss.str());
                fconf.close();
                return false;
            }
        }
        fconf.close();
    }
    return true;
}


