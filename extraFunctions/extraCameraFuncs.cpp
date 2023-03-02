#include "extraCameraFuncs.hpp"

int threshold_low = 232;
int threshold_high = 255;
int link_lenght = 65;

int PYLON_WIDTH = 1920;
int PYLON_HEIGHT = 1200;
float exposureTime = 15000.0;

Mat IntroducerMask(Mat src)
{
    Mat src_GRAY, element;
    // create a greyscale copy of the image
    //  flip(src, src, 1);

    cvtColor(src, src_GRAY, COLOR_BGR2GRAY);

    // apply blur and threshold so that only the tentacle is visible
    blur(src_GRAY, src_GRAY, Size(5, 5));
    threshold(src_GRAY, src_GRAY, threshold_low, threshold_high, THRESH_BINARY_INV);

    element = getStructuringElement(MORPH_DILATE, Size(3, 3));
    dilate(src_GRAY, src_GRAY, element);

    bitwise_not(src_GRAY, src_GRAY);

    return src_GRAY;
}

bool xWiseSort(Point lhs, Point rhs)
{
    return (lhs.x < rhs.x);
}

bool yWiseSort(Point lhs, Point rhs)
{
    return (lhs.y > rhs.y);
}

std::vector<double> computeAngles(std::vector<Point> Joints)
{
    std::vector<double> angles;
    std::vector<Point> vects;
    std::vector<int> angleSign;
    Joints.insert(Joints.begin(), Point(Joints[0].x, 0));
    for (int i = 1; i < Joints.size(); i++)
    {
        vects.push_back(Point{Joints[i].x - Joints[i - 1].x, Joints[i].y - Joints[i - 1].y});
        // int dx, dy;
        // dx = Joints[i].x - Joints[i - 1].x;
        // if(dx < 0) angleSign.push_back(-1);
        // else angleSign.push_back(1);
    }

    for (int i = 0; i < vects.size() - 1; i++)
    {
        // a = atan2d(x1*y2 -y1*x2, x1*x2 + y1*y2)
        // method taken from https://uk.mathworks.com/matlabcentral/answers/180131-how-can-i-find-the-angle-between-two-vectors-including-directional-information
        //generally that is atan(cross(v1,v2) / dot(v1,v2))
        double th = atan2(vects[i].x * vects[i+1].y - vects[i].y * vects[i+1].x,
                          vects[i].x * vects[i+1].x + vects[i].y * vects[i+1].y);
        // double dproduct = vects[i].dot(vects[i + 1]);
        // double nproduct = norm(vects[i]) * norm(vects[i + 1]);
        // double th = acos(dproduct / nproduct);
        angles.push_back((th * 180 / M_PI));
    }

    return angles;
}

std::vector<Point> computeIdealPoints(Point p0, std::vector<double> desiredAngles_)
{
    std::vector<Point> ideal;

    ideal.push_back(p0);
    for (int i = 1; i < desiredAngles_.size(); i++)
    {
        double angle = 0;
        for (int k = 0; k < i; k++)
            angle += desiredAngles_[k];
        int xdiff = (link_lenght + 10) * sin(angle * M_PI / 180);
        int ydiff = (link_lenght + 10) * cos(angle * M_PI / 180);
        Point pn = Point{(int)(ideal[i - 1].x + xdiff), (int)(ideal[i - 1].y + ydiff)};
        ideal.push_back(pn);
    }

    return ideal;
}

std::vector<Point> findJoints(Mat post_img_masked, std::vector<std::vector<Point>> &contours)
{

    Mat contours_bin;
    // std::vector<std::vector<Point> > contours;
    std::vector<Vec4i> hierarchy;
    // find contours, ignore hierarchy
    findContours(post_img_masked, contours, hierarchy, RETR_LIST, CHAIN_APPROX_SIMPLE);
    contours_bin = Mat::zeros(post_img_masked.size(), CV_8UC1);

    // draw contours and fill the open area
    drawContours(contours_bin, contours, -1, Scalar(255, 255, 255), cv::FILLED, LINE_8, hierarchy);
    // empty matrix. Set up to 8-bit 1 channel data. Very important to set up properly.
    Mat skeleton = Mat::zeros(post_img_masked.rows, post_img_masked.rows, CV_8U);

    // take the filled contour and thin it using Zhang Suen method. Only works with 8-bit 1 channel data.
    ximgproc::thinning(contours_bin, skeleton, 0);

    contours.clear();
    hierarchy.clear();
    findContours(skeleton, contours, hierarchy, RETR_LIST, CHAIN_APPROX_SIMPLE);

    std::vector<Point> cntLine;
    findNonZero(skeleton, cntLine);
    std::sort(cntLine.begin(), cntLine.end(), yWiseSort);
    // std::reverse(cntLine.begin(), cntLine.end());

    std::vector<Point> Joints;
    int jointCount = (int)std::ceil((float)(cntLine.size() / (float)link_lenght));
    // std::cout << "Size of centre-line " << cntLine.size() << "\n"
    // << "JointCount: " << jointCount << "\n";

    if (jointCount)
    {
        std::vector<Point>::iterator cntLineIterator = cntLine.begin();
        for (int i = 0; i < jointCount; i++)
        {
            Joints.push_back(*cntLineIterator);
            std::advance(cntLineIterator, link_lenght);
        }
    }
    std::reverse(Joints.begin(), Joints.end());
    // std::cout << "Number of joints " << Joints.size() << "\n";

    return Joints;
}