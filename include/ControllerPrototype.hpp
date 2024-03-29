#ifndef CONTROLLERPROTOTYPE
#define CONTROLLERPROTOTYPE

#include "extraCameraFuncs.hpp"
#include "ExtraComputationFuncs.hpp"
#include <ctime>
#include <chrono>

inline bool file_exists(const std::string &name)
{
    struct stat buffer;
    return (stat(name.c_str(), &buffer) == 0);
}

inline int meanError(std::vector<double> &desired, std::vector<double> &observed)
{
    double d_error, sum = 0;
    int error;
    // std::cout << "Inside meanError.\n";
    // std::cout << "Size of desired angles: " << desired.size() << "\nSize of observed angles: " << observed.size() << "\n";
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

inline double xwiseError(std::vector<double> observedX, std::vector<double> desiredX)
{
    double sum = 0;
    for (size_t i = 0; i < observedX.size(); i++)
    {
        sum += (desiredX[i] - observedX[i]) * (i+1);
    }
    return sum / observedX.size();
}

inline double ywiseError(std::vector<double> observedY, std::vector<double> desiredY)
{
    double sum = 0;
    for (size_t i = 0; i < observedY.size(); i++)
    {
        sum += (desiredY[i] - observedY[i]) * (i+1);
    }
    return sum / observedY.size();
}



inline int derivativeAdjustment(double d_error, int error){
    //The higher the error, the smaller the return value
    //we apply it to:
    //  1. float smallAdjustment
    //  2. int signFlag

    int scaledDiff = (int) ( d_error / (double) error * 100);
    // std::cout << "d_error: " << d_error << " - error: " << error 
    // std::cout << " scaledDiff: " << scaledDiff << "\n";
    switch(scaledDiff){
        case 0 ... 5: 
            // std::cout << "Kd = " << 4 << "\n";
            return 16;
        break;

        case 6 ... 20: 
            // std::cout << "Kd = " << 2 << "\n";
            return 8;
        break;

        case 21 ... 60:
            // std::cout << "Kd = " << 2 << "\n";
            return 4;
        break;

        default:
            // std::cout << "Kd = " << 1 << "\n";
            return 1;
        break;
    }
}

inline double derivativeAdjustmentF(double d_error){
        double Kd;
        switch (abs( (int) d_error)) {
        case 0 ... 15:
            Kd = 1.40;
            break;

        case 16 ... 40:
            Kd = 1.20;
            break;

        case 41 ... 60:
            Kd = 1.10;
            break;

        default:
            Kd = 1;
            break;
    }
    return Kd;
}

int main(int argc, char *argv[]);

#endif