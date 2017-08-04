#ifndef Neuron_hpp
#define Neuron_hpp

#include <iostream>
#include <vector>

class NN;

enum NeuronType { INPUT, HIDDEN, OUTPUT };

class Neuron {
public:
    static void SetParams(double a, double b, double etaSeed, double etaBias);
    Neuron(NN* nn, int layer, int index, NeuronType type);
    double GetY() { return _y;}
    double GetDelta() { return _delta;}
    double GetW(int i) { return _w[i];}
    double GetE() { return _e;}
    void AddPrev(std::shared_ptr<Neuron> prev) {_prevs.push_back(prev);}
    void AddNext(std::shared_ptr<Neuron> next) {_nexts.push_back(next);}
    void ForwardComputaion();
    void BackwardComputaion();
    static double ActivateFunction(double v);
    static double ActivateFunctionDerivative(double v);
    void Init();
    void Init(std::string& coeffStr);
private:

    int _L;
    int _i;
    std::vector<std::shared_ptr<Neuron>> _prevs;
    std::vector<std::shared_ptr<Neuron>> _nexts;
    NeuronType _type;
    
    std::vector<double> _w;
    double _v, _y, _e, _delta;
    double _eta;
    
    static double _a, _b, _etaSeed, _etaBias;
    
    NN* _nn;
    
    friend std::ostream& operator << (std::ostream& os, const Neuron& neuron);
};

#endif /* Neuron_hpp */
