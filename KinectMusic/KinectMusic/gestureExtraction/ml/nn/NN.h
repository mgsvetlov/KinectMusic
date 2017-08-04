#ifndef NN_h
#define NN_h

#include "Neuron.hpp"
#include <vector>
#include <string>

struct Epoch;
class DataSet;

class NN {
public:
    NN(int inputs, int outputs, std::vector<int> layers);
    NN(std::string paramsFileName);
    ~NN();
    void Connect();
    bool Train(DataSet* dataSet);
    bool Test(DataSet* dataSet, int numOut = -1);
    std::vector<int> GetHiddenLayers() { return _hiddenLayers;}
    int GetInputs() { return _inputs;}
    int GetOutputs() { return _outputs;}
    Epoch* GetTrainEpoch() {return _trainEpoch;}
    void OutTestEpoch();
private:
    void _Train(Epoch* epoch);
    bool _Test(Epoch* epoch);
    
private:
    std::vector<std::vector<std::shared_ptr<Neuron>>> _neurons;
    int _inputs;
    int _outputs;
    std::vector<int> _hiddenLayers;
    Epoch* _trainEpoch;
    bool _trained = false;

    friend std::ostream& operator << (std::ostream& os, const NN& nn);
};
#endif /* NN_h */
