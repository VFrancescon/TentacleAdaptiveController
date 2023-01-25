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


int threshold_low = 130;
int threshold_high = 255;
int link_lenght = 50;

int PYLON_WIDTH = 2048;
int PYLON_HEIGHT = 1536;

Mat IntroducerMask(Mat src);

bool xWiseSort(Point lhs, Point rhs){
    return (lhs.x < rhs.x);
}

bool yWiseSort(Point lhs, Point rhs){
    return (lhs.y > rhs.y);
}

int main(int argc, char* argv[]);

