#ifndef DataSet_hpp
#define DataSet_hpp

#include <vector>
#include <string>

struct Epoch{
    std::vector<double> _res;
    std::vector<double> _data;
    
    friend std::ostream& operator << (std::ostream& os, const Epoch& epoch);
};

std::ostream& operator << (std::ostream& os, const Epoch& epoch);

Epoch GenFunc1(int size);
Epoch GenFunc2(int size);

class DataSet {
public:
    DataSet(Epoch func(int), int rows, int inputs, int outputs);
    DataSet(std::string fileName, bool isTest = false);
    int GetInputs() {return _inputs;}
    int GetRows() {return _rows;}
    int GetOutputs() {return _outputs;}
    Epoch* GetEpoch(int i) {return &_epochs[i];}
private:
    std::vector<Epoch> _epochs;
    int _inputs, _rows;
    int _outputs;

    friend std::ostream& operator << (std::ostream& os, const DataSet& dataSet);
};

std::ostream& operator << (std::ostream& os, const DataSet& dataSet);

#endif /* DataSet_hpp */
