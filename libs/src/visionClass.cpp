#include "AdaptiveControllerVisionClass/visionClass.hpp"

VisionClass::VisionClass() {
    this->threshold_low = 115;
    this->threshold_high = 255;
    this->link_lenght = 40;

    this->PYLON_WIDTH = 1920;
    this->PYLON_HEIGHT = 1200;
    this->exposureTime = 15000.0;
    this->p0frame = Point(0, 0);
}

// VisionClass::VisionClass(int threshold_low, int threshold_high, int
// link_lenght,
//                          int PYLON_WIDTH, int PYLON_HEIGHT, float
//                          exposureTime, Point p0frame) {
//     this->threshold_low = threshold_low;
//     this->threshold_high = threshold_high;
//     this->link_lenght = link_lenght;
//     this->PYLON_WIDTH = PYLON_WIDTH;
//     this->PYLON_HEIGHT = PYLON_HEIGHT;
//     this->exposureTime = exposureTime;
//     this->p0frame = p0frame;
// }

void VisionClass::setThresholdLow(int threshold_low) {
    this->threshold_low = threshold_low;
}

void VisionClass::setThresholdHigh(int threshold_high) {
    this->threshold_high = threshold_high;
}

void VisionClass::setLinkLenght(int link_lenght) {
    this->link_lenght = link_lenght;
}

void VisionClass::setPylonDims(int PYLON_WIDTH, int PYLON_HEIGHT) {
    this->PYLON_WIDTH = PYLON_WIDTH;
    this->PYLON_HEIGHT = PYLON_HEIGHT;
}

void VisionClass::setExposureTime(float exposureTime) {
    this->exposureTime = exposureTime;
}

void VisionClass::setP0Frame(Point p0frame) { this->p0frame = p0frame; }

Mat VisionClass::IntroducerMask(Mat src) {
    Mat src_GRAY, element;
    // create a greyscale copy of the image
    //  flip(src, src, 1);

    cvtColor(src, src_GRAY, COLOR_BGR2GRAY);

    // apply blur and threshold so that only the tentacle is visible
    blur(src_GRAY, src_GRAY, Size(5, 5));
    threshold(src_GRAY, src_GRAY, this->threshold_low, this->threshold_high,
              THRESH_BINARY_INV);

    element = getStructuringElement(MORPH_DILATE, Size(3, 3));
    dilate(src_GRAY, src_GRAY, element);

    bitwise_not(src_GRAY, src_GRAY);

    return src_GRAY;
}

Mat VisionClass::isolatePhantom(Mat src) {
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
    // element = getStructuringElement(MORPH_DILATE, Size(3, 3));
    // dilate(final_result, final_result,element);
    // rectangle(mask, Point(0,0), Point(mask.cols, mask.rows * 0.3), Scalar(0,0,0), 8, FILLED);
    Point rp1(0,0), rp2(0, mask.rows * 0.3), rp3(mask.cols, mask.rows * 0.3), rp4(mask.cols, 0);
    std::vector<Point> rectP = {rp1, rp2, rp3, rp4};
    std::vector<std::vector<Point>> allShapes;
    allShapes.push_back(rectP);

    // fillPoly(mask, rectP, Scalar(0,0,0), 8);
    fillPoly(mask, allShapes, Scalar(0,0,0), 8, 0, Point());
    polylines(mask, lpts, true, Scalar(0, 0, 0), 125);
    polylines(mask, rpts, true, Scalar(0,0,0), 145);
    this->mask = mask;
    return mask;
}

bool VisionClass::xWiseSort(Point lhs, Point rhs) { return (lhs.x < rhs.x); }

bool VisionClass::yWiseSort(cv::Point lhs, cv::Point rhs) {
    if (lhs.y == rhs.y) {
        return lhs.x < rhs.x;  // Sort by x values if y values are the same
    }
    return lhs.y < rhs.y;  // Sort primarily by y values
}

bool VisionClass::euclideanSort(cv::Point lhs, cv::Point rhs) {
    double lhx = this->p0frame.x - lhs.x;
    double lhy = this->p0frame.y + lhs.y;
    double rhx = this->p0frame.x - rhs.x;
    double rhy = this->p0frame.y + rhs.y;
    double lhsDistance = std::sqrt(lhx * lhx + lhy * lhy);
    double rhsDistance = std::sqrt(rhx * rhx + rhy * rhy);
    return (lhsDistance < rhsDistance);
}

std::vector<double> VisionClass::computeAngles(std::vector<Point> Joints) {
    std::vector<double> angles;
    std::vector<Point> vects;
    std::vector<int> angleSign;
    Joints.insert(Joints.begin(), Point(Joints[0].x, 0));
    for (int i = 1; i < Joints.size(); i++) {
        vects.push_back(Point{Joints[i].x - Joints[i - 1].x,
                              Joints[i].y - Joints[i - 1].y});
        // int dx, dy;
        // dx = Joints[i].x - Joints[i - 1].x;
        // if(dx < 0) angleSign.push_back(-1);
        // else angleSign.push_back(1);
    }

    for (int i = 0; i < vects.size() - 1; i++) {
        // a = atan2d(x1*y2 -y1*x2, x1*x2 + y1*y2)
        // method taken from
        // https://uk.mathworks.com/matlabcentral/answers/180131-how-can-i-find-the-angle-between-two-vectors-including-directional-information
        // generally that is atan(cross(v1,v2) / dot(v1,v2))
        double th =
            atan2(vects[i + 1].x * vects[i].y - vects[i + 1].y * vects[i].x,
                  vects[i + 1].x * vects[i].x + vects[i + 1].y * vects[i].y);
        // double dproduct = vects[i].dot(vects[i + 1]);
        // double nproduct = norm(vects[i]) * norm(vects[i + 1]);
        // double th = acos(dproduct / nproduct);
        angles.push_back((th * 180 / M_PI));
    }

    return angles;
}

std::vector<Point> VisionClass::computeIdealPoints(
    Point p0, std::vector<double> desiredAngles_) {
    std::vector<Point> ideal;
    ideal.push_back(p0);
    for (int i = 1; i < desiredAngles_.size(); i++) {
        double angle = 0;
        for (int k = 0; k < i; k++) angle += desiredAngles_[k];
        int xdiff =
            (double)(this->link_lenght) * 1.5 * sin(angle * M_PI / 180);
        int ydiff =
            (double)(this->link_lenght) * 1.5 * cos(angle * M_PI / 180);
        Point pn =
            Point{(int)(ideal[i - 1].x + xdiff), (int)(ideal[i - 1].y + ydiff)};
        ideal.push_back(pn);
    }
    // std::cout << " Inside ideal points comp. Points calced:\n";
    // for(auto i : ideal){
    //     std::cout << " " << i;
    // }
    return ideal;
}

std::vector<Point> VisionClass::findJoints(
    Mat post_img_masked, std::vector<std::vector<Point>> &contours,
    int JointNumber, Point baseFrame) {
    // std::cout << "---------------\n\nExtafuncs version\n";
    Mat contours_bin;
    // std::vector<std::vector<Point> > contours;
    std::vector<Vec4i> hierarchy;
    // find contours, ignore hierarchy
    findContours(post_img_masked, contours, hierarchy, RETR_LIST,
                 CHAIN_APPROX_SIMPLE);
    contours_bin = Mat::zeros(post_img_masked.size(), CV_8UC1);

    // draw contours and fill the open area
    drawContours(contours_bin, contours, -1, Scalar(255, 255, 255), cv::FILLED,
                 LINE_8, hierarchy);
    // empty matrix. Set up to 8-bit 1 channel data. Very important to set up
    // properly.
    Mat skeleton =
        Mat::zeros(post_img_masked.rows, post_img_masked.rows, CV_8U);

    // take the filled contour and thin it using Zhang Suen method. Only works
    // with 8-bit 1 channel data.
    ximgproc::thinning(contours_bin, skeleton, 0);

    contours.clear();
    hierarchy.clear();
    findContours(skeleton, contours, hierarchy, RETR_LIST, CHAIN_APPROX_SIMPLE);

    std::vector<Point> cntLine;
    findNonZero(skeleton, cntLine);
    p0frame = baseFrame;
    // std::sort(cntLine.begin(), cntLine.end(), euclideanSort);
    std::sort(cntLine.begin(), cntLine.end(),
              std::bind(&VisionClass::euclideanSort, this,
                        std::placeholders::_1, std::placeholders::_2));

    // std::reverse(cntLine.begin(), cntLine.end());

    std::vector<Point> Joints;
    // int jointCount = (int)std::ceil((float)(cntLine.size() /
    // (float)link_lenght)); link_lenght = 60;
    this->link_lenght =
        std::ceil((float)(cntLine.size() - 1) / (float)JointNumber);
    // std::cout << "Size of centre-line " << cntLine.size() << "\n"
    // << "JointCount: " << JointNumber<< "\n"
    // << "Link lenght " << link_lenght << "\n";

    if (JointNumber) {
        Joints = this->equally_spaced_points(cntLine, JointNumber);
    }

    return Joints;
}

std::vector<Point> VisionClass::findJoints(Mat post_img_masked,
                                           Point baseFrame) {
    std::vector<Point> Joints;
    Mat contours_bin;
    std::vector<std::vector<Point>> contours;
    std::vector<Vec4i> hierarchy;
    // find contours, ignore hierarchy
    findContours(post_img_masked, contours, hierarchy, RETR_LIST,
                 CHAIN_APPROX_SIMPLE);
    contours_bin = Mat::zeros(post_img_masked.size(), CV_8UC1);

    // draw contours and fill the open area
    drawContours(contours_bin, contours, -1, Scalar(255, 255, 255), cv::FILLED,
                 LINE_8, hierarchy);
    // empty matrix. Set up to 8-bit 1 channel data. Very important to set up
    // properly.
    Mat skeleton =
        Mat::zeros(post_img_masked.rows, post_img_masked.rows, CV_8U);

    // take the filled contour and thin it using Zhang Suen method. Only works
    // with 8-bit 1 channel data.
    ximgproc::thinning(contours_bin, skeleton, 0);
    findContours(skeleton, contours, hierarchy, RETR_LIST, CHAIN_APPROX_SIMPLE);

    std::vector<Point> cntLine;
    findNonZero(skeleton, cntLine);
    p0frame = baseFrame;
    std::sort(cntLine.begin(), cntLine.end(),
              std::bind(&VisionClass::euclideanSort, this,
                        std::placeholders::_1, std::placeholders::_2));

    int JointNumber = cntLine.size() / this->link_lenght;
    if (JointNumber) {
        Joints = this->equally_spaced_points(cntLine, JointNumber);
        return Joints;
    } else return std::vector<Point>(); //an empty list
    
}

std::vector<Point> VisionClass::findCtrLine(
    Mat post_img_masked, std::vector<std::vector<Point>> &contours,
    Point baseFrame) {
    // std::cout << "---------------\n\nExtafuncs version\n";
    Mat contours_bin;
    // std::vector<std::vector<Point> > contours;
    std::vector<Vec4i> hierarchy;
    // find contours, ignore hierarchy
    findContours(post_img_masked, contours, hierarchy, RETR_LIST,
                 CHAIN_APPROX_SIMPLE);
    contours_bin = Mat::zeros(post_img_masked.size(), CV_8UC1);

    // draw contours and fill the open area
    drawContours(contours_bin, contours, -1, Scalar(255, 255, 255), cv::FILLED,
                 LINE_8, hierarchy);
    // empty matrix. Set up to 8-bit 1 channel data. Very important to set up
    // properly.
    Mat skeleton =
        Mat::zeros(post_img_masked.rows, post_img_masked.rows, CV_8U);

    // take the filled contour and thin it using Zhang Suen method. Only works
    // with 8-bit 1 channel data.
    ximgproc::thinning(contours_bin, skeleton, 0);

    contours.clear();
    hierarchy.clear();
    findContours(skeleton, contours, hierarchy, RETR_LIST, CHAIN_APPROX_SIMPLE);

    std::vector<Point> cntLine;
    findNonZero(skeleton, cntLine);
    p0frame = baseFrame;
    std::sort(cntLine.begin(), cntLine.end(),
              std::bind(&VisionClass::euclideanSort, this,
                        std::placeholders::_1, std::placeholders::_2));

    return cntLine;
}

/**
 * Behold! A function that computes equally spaced points from a given
 * list of ordered cv::Point elements! Now you can finally sleep at night
 * knowing that the top of the image is included. Phew!
 *
 * @param cntLine A vector of ordered cv::Point elements representing the
 * contour line.
 * @param jointNumber The desired number of equally spaced points (joints).
 * @return A vector containing the computed equally spaced points.
 */
std::vector<cv::Point> VisionClass::equally_spaced_points(
    const std::vector<cv::Point> &cntLine, int jointNumber) {
    int n = cntLine.size();
    std::vector<cv::Point> joints;

    if (jointNumber < 1) {
        std::cerr << "Error: Your joint number is feeling lonely. Give it a "
                     "positive integer, please."
                  << std::endl;
        return joints;
    }

    if (jointNumber > n) {
        std::cerr
            << "Error: Trying to squeeze more joints than points, are we? "
               "Let's keep it less than or equal to the number of points."
            << std::endl;
        return joints;
    }

    // We perform advanced math to calculate the step size for equally spaced
    // elements
    int step = std::ceil(static_cast<float>(n - 1) /
                         static_cast<float>(jointNumber - 1));

    // Fear not! We shall access equally spaced elements, including the elusive
    // first and last elements
    std::vector<cv::Point>::const_iterator it = cntLine.begin();
    for (int i = 0; i < jointNumber; ++i) {
        joints.push_back(*it);
        // std::cout << "Adding the point of interest: \n" << *it << "\n";

        if (std::distance(it, cntLine.end()) <= step) {
            break;
        }

        std::advance(it, step - 1);
    }

    return joints;
}

void VisionClass::drawLegend(Mat &post_img) {
    cv::Point TopLeftLegend(0, 390);
    cv::Point BottomRightLegend(230, 450);
    cv::Point InnerTopLeftLegend(4, 404);
    cv::Point InnerBottomRightLegend(196, 446);

    rectangle(post_img, TopLeftLegend, BottomRightLegend, Scalar(0, 0, 0), 4);
    rectangle(post_img, InnerTopLeftLegend, InnerBottomRightLegend,
              Scalar(255, 255, 255), 1);

    putText(post_img, "Detected", TopLeftLegend + Point(84, 26),
            FONT_HERSHEY_DUPLEX, 1, Scalar(0, 0, 0));
    putText(post_img, "Desired", TopLeftLegend + Point(84, 52),
            FONT_HERSHEY_DUPLEX, 1, Scalar(0, 0, 0));
    // blue rect. Desired
    rectangle(post_img, TopLeftLegend + Point(5, 12),
              TopLeftLegend + Point(80, 22), Scalar(255, 0, 0), FILLED);
    // red rect. Detected
    rectangle(post_img, TopLeftLegend + Point(5, 38),
              TopLeftLegend + Point(80, 48), Scalar(0, 0, 255), FILLED);
}

Mat VisionClass::preprocessImg(Mat post_img, int rrows, int rcols) {
    resize(post_img, post_img, Size(rcols, rrows), INTER_LINEAR);
    Mat post_img_grey, post_img_th;
    Mat post_img_masked = Mat::zeros(Size(rcols, rrows), CV_8UC1);

    cvtColor(post_img, post_img_grey, COLOR_BGR2GRAY);
    blur(post_img_grey, post_img_grey, Size(5, 5));
    threshold(post_img_grey, post_img_th, this->threshold_low,
              this->threshold_high, THRESH_BINARY_INV);
    // post_img_th.copyTo(post_img_masked);
    bitwise_and(post_img_th, post_img_th, post_img_masked, this->mask);

    return post_img_masked;
}

Mat VisionClass::preprocessImg(Mat post_img) {
    Mat post_img_grey, post_img_th;
    Mat post_img_masked;

    cvtColor(post_img, post_img_grey, COLOR_BGR2GRAY);
    blur(post_img_grey, post_img_grey, Size(5, 5));
    threshold(post_img_grey, post_img_th, this->threshold_low,
              this->threshold_high, THRESH_BINARY_INV);
    // post_img_th.copyTo(post_img_masked);
    post_img_th.copyTo(post_img_masked, this->mask);
    return post_img_masked;
}

VisionClass::~VisionClass() {}

int VisionClass::getThresholdLow() { return this->threshold_low; }

int VisionClass::getThresholdHigh() { return this->threshold_high; }

int VisionClass::getLinkLenght() { return this->link_lenght; }

int VisionClass::getPylonWidth() { return this->PYLON_WIDTH; }

int VisionClass::getPylonHeight() { return this->PYLON_HEIGHT; }

float VisionClass::getExposureTime() { return this->exposureTime; }

Point VisionClass::getP0Frame() { return this->p0frame; }

int VisionClass::getJointNumber() { return this->JointNumber; }