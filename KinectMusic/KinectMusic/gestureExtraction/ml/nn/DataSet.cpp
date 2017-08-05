#include <cmath>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <sstream>
#include "DataSet.hpp"

Epoch GenFunc1(int size){
    Epoch epoch;
    double sum (0.0);
    for(int i = 0; i < size; i++){
        double r = (std::rand() % static_cast<int>(1e+6)) * 1e-6;
        epoch._data.push_back(r);
        sum += r;
    }
    if(sum < size * 0.5) {
        epoch._res.push_back(1);
        epoch._res.push_back(0);
    }
    else {
        epoch._res.push_back(0);
        epoch._res.push_back(1);
    }
    return epoch;
}

Epoch GenFunc2(int size){
    Epoch epoch;
    double sum (0.0);
    for(int i = 0; i < size; i++){
        double r = (rand() % static_cast<int>(1e+6)) * 1e-6;
        epoch._data.push_back(r);
        sum += r;
    }
    int cl = (rand() % 2);
    double diff (0.0);
    if(cl == 0){
        diff = (-1.0 - sum) / size;
    }
    else {
        diff = (1.0 - sum) / size;
    }
    for(int i = 0; i < size; i++){
        epoch._data[i] += diff;
    }
    if(cl == 0) {
        epoch._res.push_back(1);
        epoch._res.push_back(0);
    }
    else {
        epoch._res.push_back(0);
        epoch._res.push_back(1);
    }
    std::cout << epoch;
    return epoch;
}

std::ostream& operator << (std::ostream& os, const Epoch& epoch){
    for(auto d : epoch._data){
        std::cout << d << " ";
    }
    std::cout << " res ";
    for(auto r : epoch._res){
        std::cout << r << " ";
    }
    std::cout << "\n";
    return os;
}

DataSet::DataSet(Epoch func(int), int rows, int inputs, int outputs) :
_inputs(inputs),
_rows(rows),
_outputs(outputs)
{
    for(int i = 0; i < _rows; i++){
        _epochs.push_back(func(inputs));
    }
}

DataSet::DataSet(std::string fileName, bool isTest) :
_inputs (-1),
_rows (0),
_outputs (2)
{
    std::ifstream fs;
    fs.open(fileName);
    if (fs.fail()) {
        std::cout << "Can't find file " << fileName << "\n";
    }
    else {
        while (!fs.eof()){
            std::string str;
            getline(fs, str);
            if(str.empty())
                continue;
            _epochs.push_back(Epoch());
            Epoch& epoch = _epochs.back();
            size_t pos = str.find(':');
            if (pos == std::string::npos){
                std::cout << "Error str in params file, no ':' found: " << str << "\n";
                return;
            }
            str.erase(str.begin(), str.begin() + pos + 1);
            std::stringstream ss(str);
            double i, count(0);
            while( ss >> i) {
                if((isTest && count > 3) || count > 4) {
                    epoch._data.push_back(i);
                }
                else {
                    if(count == 0){
                        if(isTest) {
                            epoch._res.push_back(0);
                            epoch._res.push_back(0);
                        }
                        else {
                            if(i != 0) {
                                epoch._res.push_back(1);
                                epoch._res.push_back(0);
                                ++count;
                            }
                            else {
                                epoch._res.push_back(0);
                                epoch._res.push_back(1);
                            }
                        }
                    }
                    ++count;
                }
            }
            if(count == 0){
                _epochs.pop_back();
                continue;
            }
            else {
                if(_inputs == -1){
                    _inputs = static_cast<int>(epoch._data.size());
                }
                else if (_inputs != epoch._data.size()){
                    std::cout << "Data size error: " << epoch._data.size() << " instead of " << _inputs << "\n";
                    _epochs.pop_back();
                    continue;
                }
                //std::cout << "epoch data size " << epoch._data.size() << "\n";
                //std::cout << epoch;
            }
        }
        _rows = static_cast<int>(_epochs.size());
        std::cout << "_rows " << _rows << " _inputs " << _inputs << "\n";
    }
    fs.close();
}

std::ostream& operator << (std::ostream& os, const DataSet& dataSet){
    for( auto e : dataSet._epochs)
        std::cout << e;
    std::cout << "\n";
   return os;
}
