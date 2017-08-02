#include <cmath>
#include <cstdlib>
#include <iostream>
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
    return epoch;
}

void Epoch::Out(){
    for(auto d : _data){
       std::cout << d << " ";
    }
    std::cout << " res ";
    for(auto r : _res){
        std::cout << r << " ";
    }
    std::cout << "\n";
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

void DataSet::Out(){
    for( auto c : _epochs)
        c.Out();
    std::cout << "\n";
}