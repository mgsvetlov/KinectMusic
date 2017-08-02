#include "NN.h"
#include "DataSet.hpp"
#include <time.h>

//Neural Network (Milti Layer Perceptron)
int main(int argc, const char * argv[]) {
    std::srand (static_cast<unsigned>(time(NULL)));
    
    //Data Set params
    Epoch (*GenFunc) (int) = &GenFunc2;
    int epochs (1);       //epochs number in generated Data Set
    int inputs (5);         //input params number in epoch
    int outputs (2);        //output params number in epoch (outputs: 0 or 1)

    DataSet dataSet(GenFunc, epochs, inputs, outputs);
    //dataSet.Out();
    
    //Neuron params
    double a = 1.7159;      // parameters for y = a * tanh(b * v)
    double b = 0.66666667;
    double etaSeed = 3e-3;  //maximum learning rate
    double etaBias = 0.9;   // learning rate for layer n: eta[n] = eta[n-1] * etaBias;
    Neuron::SetParams(a, b, etaSeed, etaBias);
    
    //Neural Net params
    std::vector<int> layers = { dataSet.GetInputs(), dataSet.GetInputs()}; //hidden layers neurons number
    NN nn (dataSet.GetInputs(), dataSet.GetOutputs(), layers);
    nn.Init();
    if(nn.Train(&dataSet)){
        std::cout << nn;
        DataSet dataSetTest(GenFunc, epochs, inputs, outputs);
        int firstEpochsToOut(10);//number of first epochs to output
        nn.Test(&dataSetTest, firstEpochsToOut);
    }
    //nn.Out();
    return 0;
}


//kost.tk@mail.ru