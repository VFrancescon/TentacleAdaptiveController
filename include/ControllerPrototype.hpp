#ifndef CONTROLLERPROTOTYPE
#define CONTROLLERPROTOTYPE

#include "extraCameraFuncs.hpp"
#include "ExtraComputationFuncs.hpp"
#include <ctime>

inline bool file_exists(const std::string &name)
{
    struct stat buffer;
    return (stat(name.c_str(), &buffer) == 0);
}

inline int meanError(std::vector<double> &desired, std::vector<double> &observed)
{
    double d_error, sum = 0;
    int error;
    std::cout << "Inside meanError.\n";
    std::cout << "Size of desired angles: " << desired.size() << "\nSize of observed angles: " << observed.size() << "\n";
    for (size_t i = 0; i < desired.size(); i++)
    {
        sum += desired[i] - observed[i];
    }
    
    d_error = sum / desired.size();
    if( std::isnan(d_error) ){
        std::cout << "Caught anomaly.";
        std::cout << "Error: " << d_error << "\n";
        std::cout << "sum: " << sum << "\n";
        std::cout << "desired.size(): " << desired.size() << "\n";
        return 15;
    }
    return (int)(d_error );
}

inline int derivativeAdjustment(double d_error, int error){
    //The higher the error, the smaller the return value
    //we apply it to:
    //  1. float smallAdjustment
    //  2. int signFlag

    int scaledDiff = (int) (d_error / (double) error * 100);
    switch(scaledDiff){
        case 0 ... 1: 
            std::cout << "Kd = " << 8 << "\n";
            return 8;
        break;

        case 2 ... 4: 
            std::cout << "Kd = " << 4 << "\n";
            return 4;
        break;

        case 5 ... 8:
            std::cout << "Kd = " << 2 << "\n";
            return 2;
        break;

        default:
            std::cout << "Kd = " << 1 << "\n";
            return 1;
        break;
    }
}

int main(int argc, char *argv[]);

#endif