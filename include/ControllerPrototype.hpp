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

inline int pieceWiseError(std::vector<double> desired, std::vector<double> observed){
    double avg;
    int error;
    // desired.pop_back();
    std::vector<double> diff(desired.size());
    if(desired.size() == observed.size()){
        for(int i = 0; i < desired.size(); i++){
            diff[i] = desired[i] - observed[i];
        }
        avg = avgVect(diff);
        // std::cout << "pieceWise precasting: " << avg << "\n";
        return (int) avg;
    } else {
        std::cout << "piecwise. No matching sizes\n";
        std::cout << "Size of desired angles: " << desired.size() << "\nSize of observed angles: " << observed.size() << "\n";
        return 0;}
}

inline int pieceWiseErrorWeighted(std::vector<double> desired, std::vector<double> observed){
    double avg;
    int error;
    // desired.pop_back();
    std::vector<double> diff(desired.size());
    if(desired.size() == observed.size()){
        for(int i = 0; i < desired.size(); i++){
            diff[i] = desired[i] - observed[i];
            diff[i] *= (i+1);
        }
        avg = avgVect(diff);
        // std::cout << "pieceWiseWeighted precasting: " << avg << "\n";
        return (int) avg;
    } else {
        std::cout << "weighted. No matching sizes\n";
        std::cout << "Size of desired angles: " << desired.size() << "\nSize of observed angles: " << observed.size() << "\n";
        return 0;}
}

inline int positionWiseError(std::vector<Point> idealPoints, std::vector<Point> observedPoints){
    double avg;
    int error;
    std::vector<double> scaledValues;
    for(int i = 0; i < idealPoints.size(); i++){
        int ithxDiff = observedPoints[i].x - idealPoints[i].x;
        int ithyDiff = observedPoints[i].y - idealPoints[i].y;
        double ithDistance = std::sqrt(ithxDiff*ithxDiff + ithyDiff*ithyDiff);
        // std::cout << "i : " << i << " distance " << ithDistance << "\n";
        scaledValues.push_back(ithDistance*i+1);
    }
    avg = avgVect(scaledValues);
    error = (int) avg;
    return error;
}

inline int positionWiseError(std::vector<Point2d> idealPoints, std::vector<Point2d> observedPoints){
    double avg;
    double error;
    std::vector<double> scaledValues;
    for(int i = 0; i < idealPoints.size(); i++){
        double ithxDiff = observedPoints[i].x - idealPoints[i].x;
        double ithyDiff = observedPoints[i].y - idealPoints[i].y;
        double ithDistance = std::sqrt(ithxDiff*ithxDiff + ithyDiff*ithyDiff);
        // std::cout << "i : " << i << " distance " << ithDistance << "\n";
        scaledValues.push_back(ithDistance*i+1);
    }
    avg = avgVect(scaledValues);
    error = avg;
    return error;
}

inline int positionWiseError(std::vector<Vector3d> idealPoints, std::vector<Vector3d> observedPoints){
    double avg;
    double error;
    std::vector<double> scaledValues;
    for(int i = 0; i < idealPoints.size(); i++){
        double ithxDiff = observedPoints[i](0) - idealPoints[i](0);
        double ithyDiff = observedPoints[i](2) - idealPoints[i](2);
        double ithDistance = std::sqrt(ithxDiff*ithxDiff + ithyDiff*ithyDiff);
        // std::cout << "i : " << i << " distance " << ithDistance << "\n";
        scaledValues.push_back(ithDistance*i+1);
    }
    avg = avgVect(scaledValues);
    error = avg;
    return error;
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
            return 8;
        break;

        case 6 ... 20: 
            // std::cout << "Kd = " << 2 << "\n";
            return 4;
        break;

        case 21 ... 60:
            // std::cout << "Kd = " << 2 << "\n";
            return 2;
        break;

        default:
            // std::cout << "Kd = " << 1 << "\n";
            return 1;
        break;
    }
}

int main(int argc, char *argv[]);

#endif