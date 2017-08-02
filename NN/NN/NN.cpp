#include <stdio.h>
#include <cmath>
#include <limits>
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
        }
    }
}

NN::~NN() {
    
}

void NN::Init(){
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
            neuron->Init();
        }
    }

}

bool NN::Train(DataSet* dataSet) {
    if(_inputs != dataSet->GetInputs() || _outputs != dataSet->GetOutputs())
        return false;
    
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
    if(_inputs != dataSet->GetInputs() || _outputs != dataSet->GetOutputs())
        return false;
    
    int counterRight (0);
    for(int i = 0; i < dataSet->GetRows(); i++){
        counterRight += _Test(dataSet->GetEpoch(i));
        if( i < numOut)
            OutTestEpoch();
    }
    std::cout << "Total: " << dataSet->GetRows()
    << "\nCorrect:" << counterRight << "\n";
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
    return _trainEpoch->_res[ind] != 0.;
}


void NN::OutTestEpoch() {
    std::cout << "data: ";
    for(auto d :_trainEpoch->_data)
        std::cout << d << " ";
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
        os << "layer " << lNum++ << "\n";
        for(const auto& n : l) {
            os << *n;
        }
    }
    return os;
}
