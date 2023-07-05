#ifndef VISIONCLASS
#define VISIONCLASS

#include <pylon/PylonIncludes.h>
#include <sys/stat.h>

#include <eigen3/Eigen/Core>
#include <functional>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/ximgproc.hpp>
#include <source/AStar.hpp>
using namespace cv;
/**
 * @brief Class containing all vision related functions and variables
 *
 */
class VisionClass {
   private:
    int threshold_low;
    int threshold_high;
    int link_lenght;

    int h_low = 0, s_low = 255, v_low = 162;
    float rect_h = 0.15;
    int PYLON_WIDTH;
    int PYLON_HEIGHT;
    float exposureTime;
    int JointNumber;

    Point p0frame;
    /**
     * @brief Compares two cv::Points, x-wise. Used for sorting
     *
     * @param lhs point on Lhs
     * @param rhs point on Rhs
     * @return true lhs is smaller than rhs
     * @return false lhs is larger than rhs
     */
    static bool xWiseSort(Point lhs, Point rhs);

    /**
     * @brief Compares two cv::Points, y-wise. Used for sorting
     *
     * @param lhs point on Lhs
     * @param rhs point on Rhs
     * @return true lhs is larger than rhs
     * @return false lhs is smaller than rhs
     */
    static bool yWiseSort(Point lhs, Point rhs);

    bool euclideanSort(Point lhs, Point rhs);

   public:
    Mat mask;

    /**
     * @brief instantiates a VisionClass object with default parameters
     *
     */
    VisionClass(/* args */);

    /**
     * @brief Construct a new Vision Class object with custom parameters
     *
     * @param threshold_low
     * @param threshold_high
     * @param link_lenght
     * @param PYLON_WIDTH
     * @param PYLON_HEIGHT
     * @param exposureTime
     * @param p0frame
     */
    // VisionClass(int threshold_low=115, int threshold_high=255, int
    // link_lenght=40,
    //             int PYLON_WIDTH=1920, int PYLON_HEIGHT=1200, float
    //             exposureTime=1500, Point p0frame=Point(0,0));
    ~VisionClass();

    /**
     * @brief Generates a Mask containing all pre-introduction elements to block
     * them out in processing. Use cv::Mat.copy_to(dst, mask) to apply later.
     *
     * @param src Pre-introduction image, featuring any undesired data
     * @return Mat Mask to be applied to all subsequent frames
     */
    Mat IntroducerMask(Mat src);

    Mat isolatePhantom(Mat src);

    /**
     * @brief Computes angles between a sorted list of joints, obtained from dot
     * product.
     *
     * @param Joints vector containing the position of each identified Joint.
     * Size n
     * @return std::vector<double> Size n-1 list of angles, top down.
     */
    std::vector<double> computeAngles(std::vector<Point> Joints);

    /**
     * @brief Computes the ideal points required to reach the desired joint
     * angles.
     *
     * @param p0 Starting point
     * @param desiredAngles_ Size n-1. Set of desired Joint Angles
     * @return std::vector<Point> Size n. Outputted set of points that start at
     * p0 and respect all desired angles.
     */
    std::vector<Point> computeIdealPoints(Point p0,
                                          std::vector<double> desiredAngles_);

    /**

    /**
     * @brief Finds ordered list of joints from a given masked image. Uses Zhang
    Suen thinning. Number of Joints is given as arg, the link_l is computed
     *
     * @param post_img_masked processed image
     * @param contours contours, really not necessary
     * @param JointNumber number of joints we're looking for
     * @param baseFrame base frame of the robot, if known
     * @return std::vector<Point>
     */
    std::vector<Point> findJoints(Mat post_img_masked,
                                  std::vector<std::vector<Point>> &contours,
                                  int JointNumber = 6,
                                  Point baseFrame = Point(0, 0));

    std::vector<Point> findJoints(Mat post_img_masked,
                                  Point baseFrame = Point(0, 0));

    std::vector<Point> findCtrLine(Mat post_img_masked,
                                   std::vector<std::vector<Point>> &contours,
                                   Point baseFrame = Point(0, 0));

    std::vector<cv::Point> equally_spaced_points(
        const std::vector<cv::Point> &cntLine, int jointNumber);

    void drawLegend(Mat &post_img);

    /**
     * @brief Takes in raw image, process and applies mask. Also resizes to
     * rcols, rrows. Pulls mask from internal class variable
     *
     * @param post_img image to process
     * @param rrows rows to resize to
     * @param rcols cols to resize to
     * @return Mat processed image
     */
    Mat preprocessImg(Mat post_img, int rrows, int rcols);

    /**
     * @brief Takes in raw image, process and applies mask. Does no resizing.
     * Handle it manually first. Pulls mask from internal class variable.
     *
     * @param post_img image to process
     * @param rrows rows to resize to
     * @param rcols cols to resize to
     * @return Mat processed image
     */
    Mat preprocessImg(Mat post_img);

    /**
     * @brief Set the Hsv Low object
     * 
     * @param h_low 
     * @param s_low 
     * @param v_low 
     */
    void setHsvLow(int h_low, int s_low, int v_low);

    /**
     * @brief Set the Rect W object
     * 
     * @param rectW 
     */
    void setRectW(float rectW);

    /**
     * @brief Set the Threshold Low object
     *
     * @param threshold_low
     */
    void setThresholdLow(int threshold_low);

    /**
     * @brief Set the Threshold High object
     *
     * @param threshold_high
     */
    void setThresholdHigh(int threshold_high);
    /**
     * @brief Set the Link Lenght object
     *
     * @param link_lenght
     */
    void setLinkLenght(int link_lenght);

    /**
     * @brief Set the Pylon Dims object
     *
     * @param PYLON_WIDTH
     * @param PYLON_HEIGHT
     */
    void setPylonDims(int PYLON_WIDTH, int PYLON_HEIGHT);

    /**
     * @brief Set the Exposure Time object
     *
     * @param exposureTime
     */
    void setExposureTime(float exposureTime);

    /**
     * @brief Set the P0 Frame object
     *
     * @param p0frame
     */
    void setP0Frame(Point p0frame);

    /**
     * @brief Get the Joint Number object
     *
     * @return int
     */
    int getJointNumber();

    /**
     * @brief Get the Threshold Low object
     *
     * @return int
     */
    int getThresholdLow();

    /**
     * @brief Get the Threshold High object
     *
     * @return int
     */
    int getThresholdHigh();

    /**
     * @brief Get the Link Lenght object
     *
     * @return int
     */
    int getLinkLenght();

    /**
     * @brief Get the Pylon Width object
     *
     * @return int
     */
    int getPylonWidth();

    /**
     * @brief Get the Pylon Height object
     *
     * @return int
     */
    int getPylonHeight();

    /**
     * @brief Get the Exposure Time object
     *
     * @return float
     */
    float getExposureTime();

    /**
     * @brief Get the P0 Frame object
     *
     * @return Point
     */
    Point getP0Frame();
};

#endif