#include <opencv2/opencv.hpp>
#include "AdaptiveControllerVisionClass/visionClass.hpp"

using namespace cv;

int main(int argc, char* argv[]){
    Mat src = imread("/home/vittorio/TentacleAdaptiveController/dayofimg.png", IMREAD_COLOR);
    VisionClass viz;

    Mat hsv, mask, element;
    cvtColor(src, hsv, COLOR_BGR2HSV);

    inRange(hsv, Scalar(4, 78, 236), Scalar(255, 255, 255), mask);
    blur(hsv, hsv, Size(3, 3));
    Point p1(0, 0), p2(0, src.rows), p3(src.cols * 0.15, 0);
    std::vector<Point> lpts = {p1, p2, p3};

    Point p4(src.cols, 0), p5(src.cols, src.rows), p6(src.cols * 0.75, 0);
    std::vector<Point> rpts = {p4, p5, p6};

    Mat final_result;
    bitwise_and(src, src, final_result, mask);
    element = getStructuringElement(MORPH_DILATE, Size(3, 3));
    dilate(final_result, final_result,element);
    rectangle(final_result, Point(0,0), Point(final_result.cols, final_result.rows * 0.3), 1, FILLED);

    polylines(final_result, lpts, true, Scalar(0, 0, 0), 125);
    polylines(final_result, rpts, true, Scalar(0,0,0), 145);
    

    imshow("src", final_result);
    imshow("hsv", hsv);
    waitKey(0);


    return 0;
}