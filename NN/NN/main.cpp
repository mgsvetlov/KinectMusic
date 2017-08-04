#include "../../KinectMusic/KinectMusic/gestureExtraction/ml/nn/NN.h"
#include "../../KinectMusic/KinectMusic/gestureExtraction/ml/nn/DataSet.hpp"
#include <time.h>
#include <iostream>
#include <fstream>
#include <sstream>

std::string getCurrentTime(){
    time_t t = time(0);   // get time now
    struct tm * now = localtime( & t );
    std::stringstream ss;
    ss << '_' << (now->tm_year + 1900) << '_' << (now->tm_mon + 1) << '_'
    <<  now->tm_mday << '_'<< now->tm_hour << '_'<< now->tm_min;
    return ss.str();
}

//Neural Network (Milti Layer Perceptron)
int main(int argc, const char * argv[]) {
    std::srand (static_cast<unsigned>(time(NULL)));
    std::string fileName, paramsFileName;
    if(argc >= 2){
        fileName = std::string(argv[1]);
    }
    else{
        std::cout << "Please, enter file name:\n";
        std::cin >> fileName;
    }
    
    if(argc >= 3){
        paramsFileName = std::string(argv[2]);
    }
    
    bool isTest (false);
    if(argc > 3){ //test
        //fileName = "/Users/mikhailsvetlov/Dropbox/Politech/Diplom/prototype/build/Products/Release/ann/test/_2017_8_4_16_13.log";
        fileName = "ann/test/" + fileName;
        isTest= true;
    }
    else { //train
        //fileName = "/Users/mikhailsvetlov/Dropbox/Politech/Diplom/prototype/build/Products/Release/ann/train/_2017_8_4_16_13.log";
        fileName = "ann/train/" + fileName;
    }
    
    //paramsFileName = "/Users/mikhailsvetlov/Dropbox/Politech/Diplom/prototype/build/Products/Release/ann/params/_2017_8_4_17_58.log";
    if(paramsFileName != "")
        paramsFileName = "ann/params/" + paramsFileName;
    
    DataSet dataSet(fileName, isTest);

    //Neuron params
    double a = 1.7159;      // parameters for y = a * tanh(b * v)
    double b = 0.66666667;
    double etaSeed = 3e-3;  //maximum learning rate
    double etaBias = 0.9;   // learning rate for layer n: eta[n] = eta[n-1] * etaBias;
    Neuron::SetParams(a, b, etaSeed, etaBias);
    
    NN* nn = nullptr;
    if(paramsFileName == "") {
        std::vector<int> layers = { dataSet.GetInputs()};
        nn = new NN(dataSet.GetInputs(), dataSet.GetOutputs(), layers);
    }
    else {
        nn = new NN(paramsFileName);
    }
    nn->Connect();
    
    if(!isTest){
        if(nn->Train(&dataSet)){
            std::ofstream log;// = new std::ofstream();
            log.open("ann/params/" + getCurrentTime() + ".log");
            log << *nn;
            log.close();
        }
    }
    else {
        nn->Test(&dataSet);
    }
    delete nn;
    /*
    //Data Set params
    Epoch (*GenFunc) (int) = &GenFunc2;
    int epochs (1);       //epochs number in generated Data Set
    int inputs (5);         //input params number in epoch
    int outputs (2);        //output params number in epoch (outputs: 0 or 1)
    
    DataSet dataSet(GenFunc, epochs, inputs, outputs);
    
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
    */
    return 0;
}


//kost.tk@mail.ru