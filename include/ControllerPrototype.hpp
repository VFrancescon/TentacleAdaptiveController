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

int main(int argc, char *argv[]);

#endif