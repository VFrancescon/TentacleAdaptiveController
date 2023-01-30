#ifndef CONTROLLERPROTOTYPE
#define CONTROLLERPROTOTYPE

#include "extraCameraFuncs.hpp"
#include "ExtraComputationFuncs.hpp"

inline bool file_exists(const std::string &name)
{
    struct stat buffer;
    return (stat(name.c_str(), &buffer) == 0);
}

inline int meanError(std::vector<double> &desired, std::vector<double> &observed)
{
    double d_error, sum = 0;
    int error;
    for (size_t i = 0; i < desired.size(); i++)
    {
        sum += desired[i] - observed[i];
    }
    d_error = sum / desired.size();

    return (int)(d_error * 1000);
}

inline int derivativeAdjustment(double d_error, int error){
    //The higher the error, the smaller the return value
    //we apply it to:
    //  1. float smallAdjustment
    //  2. int signFlag

    int scaledDiff = (int) (d_error / (double) error * 100);
    switch(scaledDiff){
        case 0 ... 1: 
            return 8;
        break;

        case 2 ... 4: 
            return 4;
        break;

        case 5 ... 8:
            return 2;
        break;

        default:
            return 1;
        break;
    }
}

int main(int argc, char *argv[]);

#endif