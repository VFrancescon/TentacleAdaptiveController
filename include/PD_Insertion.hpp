#ifndef PD_INSERTION
#define PD_INSERTION

#include "AdaptiveControllerCompClass/compClass.hpp"
#include "AdaptiveControllerVisionClass/visionClass.hpp"
#include "HCoilMiddlewareLib/HCoilMiddlewareLib.hpp"
#include <ctime>
#include <chrono>

inline bool file_exists(const std::string &name)
{
    struct stat buffer;
    return (stat(name.c_str(), &buffer) == 0);
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

/**
 * @brief Averages an std::vector.
 *
 * @tparam T any numerical type.
 * @param inputVec input vector, size n.
 * @return double average.
 */
template <typename T>
inline double avgVect(std::vector<T> inputVec)
{
    double avg, sum = 0;

    for (auto i : inputVec)
    {
        sum += i;
    }
    avg = sum / inputVec.size();
    return avg;
}

Mat pylonPtrToMat(Pylon::CGrabResultPtr &ptrGrabResult, Pylon::CImageFormatConverter &formatConverter){
    const uint8_t *pImageBuffer = (uint8_t *)ptrGrabResult->GetBuffer();
    Pylon::CPylonImage pylonImage;
    formatConverter.Convert(pylonImage, ptrGrabResult);
    Mat post_img =
        cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(),
                CV_8UC3, (uint8_t *)pylonImage.GetBuffer());
    return post_img;
}

void visualizePoints(Mat &post_img, std::vector<Point> idealPoints, std::vector<Point> Joints){
    for (int i = 0; i < idealPoints.size() - 1; i++) {
            // std::cout << " " << i;
            line(post_img, idealPoints[i], idealPoints[i + 1],
                 Scalar(0, 0, 255), 2);
            circle(post_img, idealPoints[i], 3, Scalar(0, 255, 0), FILLED);
            putText(post_img, std::to_string(i), idealPoints[i],
                    FONT_HERSHEY_SIMPLEX, 1.0, Scalar(255, 0, 0));
        }
        // std::cout << "\n";
        circle(post_img, idealPoints[idealPoints.size() - 1], 3,
               Scalar(0, 255, 0), FILLED);
        putText(post_img, std::to_string(idealPoints.size() - 1),
                idealPoints[idealPoints.size() - 1], FONT_HERSHEY_SIMPLEX, 1.0,
                Scalar(255, 0, 0));

    for (auto i : Joints) {
        circle(post_img, i, 4, Scalar(255, 0, 0), FILLED);
    }
}


int main(int argc, char* argv[]);

#endif // PD_INSERTION