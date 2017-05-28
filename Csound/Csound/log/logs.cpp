//
//  log.cpp
//  Csound
//
//  Created by Mikhail Svetlov on 28/05/17.
//  Copyright © 2017 mgsvetlov. All rights reserved.
//

#include <sstream>
#include <ctime>
#include <fstream>
#include "logs.h"

std::map<std::string, std::ofstream*> Logs::logs;

void Logs::writeLog(const std::string& name, const std::string& data){
    std::ofstream* logPtr = nullptr;
    auto it = logs.find(name);
    if(it != logs.end()){
        logPtr = it->second;
    }
    else {
        std::ofstream* log = new std::ofstream();
        log->open(name + getCurrentTime() + ".log");
        std::pair<std::map<std::string, std::ofstream*>::iterator, bool> ret;
        ret = logs.insert ( std::pair<std::string, std::ofstream*>(name,log) );
        logPtr = ret.first->second;
    }
    (*logPtr) << data << std::endl;
}

void Logs::closeLogs(){
    for(auto it = logs.begin(); it != logs.end(); ++it){
        it->second->close();
        delete it->second;
    }
    logs.clear();
}

std::string Logs::getCurrentTime(){
    time_t t = time(0);   // get time now
    struct tm * now = localtime( & t );
    std::stringstream ss;
    ss << '_' << (now->tm_year + 1900) << '_' << (now->tm_mon + 1) << '_'
    <<  now->tm_mday << '_'<< now->tm_hour << '_'<< now->tm_min;
    return ss.str();
}