#ifndef CAMERAFUNCS
#define CAMERAFUNCS

#include <iostream>
#include <eigen3/Eigen/Core>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/ximgproc.hpp>
#include <pylon/PylonIncludes.h>
#include <source/AStar.hpp>
#include <sys/stat.h>
using namespace cv;

extern int threshold_low;
extern int threshold_high;
extern int link_lenght;

extern int PYLON_WIDTH;
extern int PYLON_HEIGHT;
extern float exposureTime;

/**
 * @brief Generates a Mask containing all pre-introduction elements to block them out in processing.
 * Use cv::Mat.copy_to(dst, mask) to apply later.
 *
 * @param src Pre-introduction image, featuring any undesired data
 * @return Mat Mask to be applied to all subsequent frames
 */
Mat IntroducerMask(Mat src);

/**
 * @brief Compares two cv::Points, x-wise. Used for sorting
 *
 * @param lhs point on Lhs
 * @param rhs point on Rhs
 * @return true lhs is smaller than rhs
 * @return false lhs is larger than rhs
 */
bool xWiseSort(Point lhs, Point rhs);

/**
 * @brief Compares two cv::Points, y-wise. Used for sorting
 *
 * @param lhs point on Lhs
 * @param rhs point on Rhs
 * @return true lhs is larger than rhs
 * @return false lhs is smaller than rhs
 */
bool yWiseSort(Point lhs, Point rhs);

/**
 * @brief Computes angles between a sorted list of joints, obtained from dot product.
 *
 * @param Joints vector containing the position of each identified Joint. Size n
 * @return std::vector<double> Size n-1 list of angles, top down.
 */
std::vector<double> computeAngles(std::vector<Point> Joints);

/**
 * @brief Computes the ideal points required to reach the desired joint angles.
 *
 * @param p0 Starting point
 * @param desiredAngles_ Size n-1. Set of desired Joint Angles
 * @return std::vector<Point> Size n. Outputted set of points that start at p0 and respect all desired angles.
 */
std::vector<Point> computeIdealPoints(Point p0, std::vector<double> desiredAngles_);

/**
 * @brief Finds ordered list of joints from a given masked image. Uses Zhang Suen thinning.
 *
 * @param post_img_masked Source image, containing isolated catheter only.
 * @param contours reference to contorus vector, sent for debugging.
 * @return std::vector<Point> Set of points containing all the detected joint angles. Size n.
 */
std::vector<Point> findJoints(Mat post_img_masked, std::vector<std::vector<Point>> &contours);

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


std::vector< Point> equally_spaced_points(const std::vector< Point>& cntLine, int jointNumber);

#endif