#include <cmath>
#include <sstream>
#include "Neuron.hpp"
#include "NN.h"
#include "DataSet.hpp"

double Neuron::_a(1.7159), Neuron::_b(0.66666667);
double Neuron::_etaSeed(1e-2), Neuron::_etaBias(1.);

void Neuron::SetParams(double a, double b, double etaSeed, double etaBias){
    _a = a;
    _b = b;
    _etaSeed = etaSeed;
    _etaBias = etaBias;
}

Neuron::Neuron(NN* nn, int layer, int index, NeuronType type) :
_nn(nn),
_L(layer),
_i(index),
_type(type){
    
}

void  Neuron::Init() {
    _eta = Neuron::_etaSeed * pow(_etaBias, _L);
    if(_type == INPUT)
        return;
    int inputs = _type == INPUT ? 0 :
    _L == 1 ? _nn->GetInputs() :
    _type == OUTPUT ? _nn->GetHiddenLayers().back() :
    _nn->GetHiddenLayers()[_L-2];
    inputs ++; 
    for(int i = 0; i < inputs; i++)
        _w.push_back((std::rand() % 2000) * 1e-3 - 1);
    
}

void Neuron::Init(std::string& str){
    _eta = Neuron::_etaSeed * pow(_etaBias, _L);
    if(_type == INPUT)
        return;
    size_t pos = str.find(':');
    if (pos == std::string::npos){
        std::cout << "Error str in params file, no ':' found: " << str << "\n";
        return;
    }
    str.erase(str.begin(), str.begin() + pos + 1);
    std::stringstream ss(str);
    double d;
    while( ss >> d) {
        _w.push_back(d);
    }
    
}

void Neuron::ForwardComputaion() {
    switch(_type){
        case INPUT:
            _y = _nn->GetTrainEpoch()->_data[_i];
            break;
        case HIDDEN:
        case OUTPUT:
            _v = _w[0];
            for(int i = 0; i < _prevs.size(); i++)
                _v += _prevs[i]->GetY() * _w[i+1];
            _y = Neuron::ActivateFunction(_v);
            break;
        default:
            break;
    }
    if(_type == OUTPUT)
        _e = _nn->GetTrainEpoch()->_res[_i] - _y;
}

void Neuron::BackwardComputaion() {
    switch(_type){
        case OUTPUT:
            _delta = _e;
            break;
        case HIDDEN:
            _delta = 0.0;
            for(int i = 0; i < _nexts.size(); i++)
                _delta += _nexts[i]->GetDelta() * _nexts[i]->GetW(i+1);
            break;
        case INPUT:
            return;
        default:
            break;
    }
    _delta *= Neuron::ActivateFunctionDerivative(_v);
    for(int i = 0; i < _w.size(); i++){
        double yPrev = i == 0 ? 1. : _prevs[i-1]->GetY();
        _w[i] = _w[i] + _eta * _delta * yPrev;
    }
}

double Neuron::ActivateFunction(double v){
    return _a * tanh(_b * v);
}

double Neuron::ActivateFunctionDerivative(double v){
    return _a * _b * (1.0 - tanh(_b * v) * tanh(_b * v));
}

std::ostream& operator << (std::ostream& os, const Neuron& neuron){
    os << "i " << neuron._i << " type " << neuron._type << " w: ";
    for( auto w : neuron._w )
        os << w << " ";
    /*os << " eta " << neuron._eta;
    if(neuron._type == OUTPUT)
        os << " e " << neuron._e;*/
    os << "\n";
    return os;
}

