//
//  log.hpp
//  Csound
//
//  Created by Mikhail Svetlov on 28/05/17.
//  Copyright © 2017 mgsvetlov. All rights reserved.
//

#ifndef log_h
#define log_h

#include <string>
#include <map>

class Logs {
public:
    static void writeLog(const std::string& name, const std::string& data);
    static void closeLogs();
    static std::string getCurrentTime();
private:
    static std::map<std::string, std::ofstream*> logs;
};

#endif /* log_hpp */
