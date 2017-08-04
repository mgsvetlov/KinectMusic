#include <cmath>
#include <limits>
#include <iostream>
#include <fstream>
#include <sstream>
#include "DataSet.hpp"
#include "NN.h"


NN::NN(int inputs, int outputs, std::vector<int> hiddenLayers) :
_inputs(inputs),
_outputs(outputs),
_hiddenLayers(hiddenLayers)
{
    if(!_neurons.empty())
        _neurons.clear();
    _neurons.resize(_hiddenLayers.size() + 2);
    for(int i = 0; i < _neurons.size(); i++){
        NeuronType type = i == 0 ? INPUT : i == _neurons.size() - 1 ? OUTPUT : HIDDEN;
        int count = i == _neurons.size() - 1 ? _outputs : i == 0 ? _inputs : _hiddenLayers[i-1];
        for(int j = 0 ; j < count; j++) {
            _neurons[i].push_back(std::shared_ptr<Neuron> (new Neuron(this, i, j, type)));
            _neurons[i].back()->Init();
        }
    }
}

NN::NN(std::string paramsFileName){
    std::ifstream fs;
    fs.open(paramsFileName);
    if (fs.fail()) {
        std::cout << "Can't find params file " << paramsFileName << "\n";
    }
    else {
        if(!_neurons.empty())
            _neurons.clear();
        NeuronType type = INPUT;
        int i = -1;
        int j = 0;
        while (!fs.eof()){
            std::string str;
            getline(fs, str);
            if(str.empty())
                continue;
            if(str == "layer INPUT"){
                type = INPUT;
                _neurons.push_back(std::vector<std::shared_ptr<Neuron>>());
                ++i;
                j = 0;
            }
            else if (str == "layer HIDDEN"){
                type = HIDDEN;
                _neurons.push_back(std::vector<std::shared_ptr<Neuron>>());
                ++i;
                j = 0;
            }
            else if (str == "layer OUTPUT"){
                type = OUTPUT;
                _neurons.push_back(std::vector<std::shared_ptr<Neuron>>());
                ++i;
                j = 0;
            }
            else if(str[0] == 'i'){
                _neurons[i].push_back(std::shared_ptr<Neuron> (new Neuron(this, i, j++, type)));
                _neurons[i].back()->Init(str);
            }
            else {
                std::cout << "Error str in params file: " << str << "\n";
            }
        }
        _inputs = static_cast<int>(_neurons.front().size());
        _outputs = static_cast<int>(_neurons.back().size());
        for(int i = 0; i < _neurons.size(); ++i){
            if(i != 0 && i != _neurons.size()-1)
            _hiddenLayers.push_back(static_cast<int>(_neurons[i].size()));
        }
    }
    
}

NN::~NN() {
    
}

void NN::Connect(){
    for(int i = 0; i < _neurons.size(); i++){
        for(int j = 0 ; j < _neurons[i].size(); j++) {
            auto neuron = _neurons[i][j];
            if(i != 0){
                for(auto n : _neurons[i - 1])
                    neuron->AddPrev(n);
            }
            if(i != _neurons.size() - 1){
                for(auto n : _neurons[i + 1])
                    neuron->AddNext(n);
            }
        }
    }

}

bool NN::Train(DataSet* dataSet) {
    if(_inputs != dataSet->GetInputs()){
        std::cout << "Error: NN::Train _inputs != dataSet->GetInputs()\n";
        return false;
    }
    if(_outputs != dataSet->GetOutputs()){
        std::cout << "Error: NN::Train _outputs != dataSet->GetOutputs()\n";
        return false;
    }
    for(int i = 0; i < dataSet->GetRows(); i++)
        _Train(dataSet->GetEpoch(i));
    
    _trained = true;
    return true;
}

void NN::_Train(Epoch* epoch){
    _trainEpoch = epoch;
    for(auto l : _neurons) {
        for(auto n : l) {
            n->ForwardComputaion();
        }
    }
    for(int i = static_cast<int>(_neurons.size()) - 1; i >= 0; i--){
        for(auto n : _neurons[i]) {
            n->BackwardComputaion();
        }
    }
}

bool NN::Test(DataSet* dataSet, int numOut) {
    if(_inputs != dataSet->GetInputs()){
        std::cout << "Error: NN::Test _inputs != dataSet->GetInputs()\n";
        return false;
    }
    if(_outputs != dataSet->GetOutputs()){
        std::cout << "Error: NN::Test _outputs != dataSet->GetOutputs()\n";
        return false;
    }
    
    //int counterRight (0);
    for(int i = 0; i < dataSet->GetRows(); i++){
        //counterRight += _Test(dataSet->GetEpoch(i));
        bool res = _Test(dataSet->GetEpoch(i));
        if( numOut == -1 || i < numOut){
            std::cout << "\nres: " << res << " ";
            //OutTestEpoch();
        }
    }
    //std::cout << "Total: " << dataSet->GetRows()
    //<< "\nCorrect:" << counterRight << "\n";
    return true;
}

bool NN::_Test(Epoch* epoch){
    _trainEpoch = epoch;
    for(auto l : _neurons) {
        for(auto n : l) {
            n->ForwardComputaion();
        }
    }
    double minError (std::numeric_limits<double>::max());
    int ind (-1);
    for(int i = 0; i < _outputs; i++ ){
        auto error = fabs(1. - _neurons[_neurons.size() - 1][i]->GetY());
        if( error < minError) {
            ind = i;
            minError = error;
        }
    }
    return ind == 0;//_trainEpoch->_res[ind] != 0.;
}


void NN::OutTestEpoch() {
    /*std::cout << "data: ";
    for(auto d :_trainEpoch->_data)
        std::cout << d << " ";*/
    std::cout << "\nres: ";
    for(auto r :_trainEpoch->_res)
        std::cout << r << " ";
    std::cout << "\nresponse: ";
    for(int i = 0; i < _outputs; i++ )
        std::cout << _neurons[_neurons.size() - 1][i]->GetY() << " ";
    std::cout << "\n";
}

std::ostream& operator << (std::ostream& os, const NN& nn){
    int lNum(0);
    for(const auto& l : nn._neurons) {
        os << "layer " << (lNum == 0 ? "INPUT":  lNum == nn._neurons.size() -1 ? "OUTPUT" : "HIDDEN") << "\n";
        ++lNum;
        for(const auto& n : l) {
            os << *n;
        }
    }
    return os;
}
