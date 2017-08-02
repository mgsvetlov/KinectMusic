#ifndef DataSet_hpp
#define DataSet_hpp

#include <vector>
#include <stdio.h>

struct Epoch{
    std::vector<double> _res;
    std::vector<double> _data;
    void Out();
};

Epoch GenFunc1(int size);
Epoch GenFunc2(int size);

class DataSet {
public:
    DataSet(Epoch func(int), int rows, int inputs, int outputs);
    int GetInputs() {return _inputs;}
    int GetRows() {return _rows;}
    int GetOutputs() {return _outputs;}
    Epoch* GetEpoch(int i) {return &_epochs[i];}
    void Out();
private:
    std::vector<Epoch> _epochs;
    int _inputs, _rows;
    int _outputs;

};

#endif /* DataSet_hpp */
