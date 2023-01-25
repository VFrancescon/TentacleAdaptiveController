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


Mat IntroducerMask(Mat src);

bool xWiseSort(Point lhs, Point rhs);

bool yWiseSort(Point lhs, Point rhs);

std::vector<double> computeAngles(std::vector<Point> Joints);

std::vector<Point> computeIdealPoints(Point p0, std::vector<double> desiredAngles_);

std::vector<Point> findJoints(Mat post_img_masked, std::vector<std::vector<Point>> &contours);

template<typename T>
double avgVect(std::vector<T> inputVec);


#endif