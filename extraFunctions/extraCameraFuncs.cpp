#include "extraCameraFuncs.hpp"

int threshold_low = 200;
int threshold_high = 255;
int link_lenght = 65;
int JointNumber = 6;

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

// bool yWiseSort(Point lhs, Point rhs)
// {
//     return (lhs.y > rhs.y);
// }

bool yWiseSort(cv::Point lhs, cv::Point rhs) {
    if (lhs.y == rhs.y) {
        return lhs.x < rhs.x; // Sort by x values if y values are the same
    }
    return lhs.y < rhs.y; // Sort primarily by y values
}



bool euclideanSort(cv::Point lhs, cv::Point rhs) {
    double lhsDistance = std::sqrt(lhs.x * lhs.x + lhs.y * lhs.y);
    double rhsDistance = std::sqrt(rhs.x * rhs.x + rhs.y * rhs.y);
    return lhsDistance < rhsDistance;
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
        double th = atan2(vects[i+1].x * vects[i].y - vects[i+1].y * vects[i].x,
                          vects[i+1].x * vects[i].x + vects[i+1].y * vects[i].y);
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
    // std::cout << "---------------\n\nExtafuncs version\n";
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
    std::sort(cntLine.begin(), cntLine.end(), euclideanSort);
    // std::reverse(cntLine.begin(), cntLine.end());

    std::vector<Point> Joints;
    // int jointCount = (int)std::ceil((float)(cntLine.size() / (float)link_lenght));
    // link_lenght = 60;
    link_lenght = std::ceil( 
        (float) (cntLine.size()-1) / (float) JointNumber
    ) ;
    // std::cout << "Size of centre-line " << cntLine.size() << "\n"
    // << "JointCount: " << JointNumber<< "\n"
    // << "Link lenght " << link_lenght << "\n";

    if (JointNumber)
    {
        Joints = equally_spaced_points(cntLine, JointNumber);

        // std::vector<Point>::iterator cntLineIterator = cntLine.end()-1;
        // std::vector<Point>::iterator cntLineEnd = cntLine.end() -1;

        // //find the number of viable points
        // // int availablePoints = cntLineEnd - cntLineIterator;
        // // std::cout << "Available: " << availablePoints << "\n"
        // // << "vs the size of the vect: " << cntLine.size() << "\n";

        // for (int i = 0; i < JointNumber; i++)
        // {
        //     Joints.push_back(*cntLineIterator);
        //     std::cout << "Pushing back point: \n" << *cntLineIterator << "\n";
        //     std::advance(cntLineIterator, -1*link_lenght);
        // }
        // std::cout << "Actual last point is: " << *cntLineEnd << "\n";
        // Joints.push_back(*cntLineEnd);
        // std::cout << "Pushing back point: \n" << *cntLineIterator << "\n";

    }
    // std::reverse(Joints.begin(), Joints.end());
    // std::cout << "Number of joints " << Joints.size() << "\n";

    return Joints;
}


/**
 * Behold! A function that computes equally spaced points from a given
 * list of ordered cv::Point elements! Now you can finally sleep at night
 * knowing that the top of the image is included. Phew!
 *
 * @param cntLine A vector of ordered cv::Point elements representing the contour line.
 * @param jointNumber The desired number of equally spaced points (joints).
 * @return A vector containing the computed equally spaced points.
 */
std::vector<cv::Point> equally_spaced_points(const std::vector<cv::Point>& cntLine, int jointNumber) {
    int n = cntLine.size();
    std::vector<cv::Point> joints;

    if (jointNumber < 1) {
        std::cerr << "Error: Your joint number is feeling lonely. Give it a positive integer, please." << std::endl;
        return joints;
    }

    if (jointNumber > n) {
        std::cerr << "Error: Trying to squeeze more joints than points, are we? Let's keep it less than or equal to the number of points." << std::endl;
        return joints;
    }

    // We perform advanced math to calculate the step size for equally spaced elements
    int step = std::ceil(static_cast<float>(n - 1) / static_cast<float>(jointNumber - 1));

    // Fear not! We shall access equally spaced elements, including the elusive first and last elements
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