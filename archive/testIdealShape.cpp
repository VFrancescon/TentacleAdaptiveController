#include "extraCameraFuncs.hpp"

int main(int argc, char *argv[]){

    Mat post_img = Mat::zeros(Size(600,600),CV_8UC3);
    int jointEff = 5;
    int jointNo = jointEff + 1;
    std::vector<double> DesiredAngles(jointNo);
    DesiredAngles[0] = -20;
    DesiredAngles[1] = 30;
    DesiredAngles[2] = 10;
    DesiredAngles[3] = 30;
    DesiredAngles[4] = 45;
    DesiredAngles[jointEff] = 0;
    Point p0 = Point{300,100};
    std::vector<Point> idealPoints;
    idealPoints = computeIdealPoints(p0, DesiredAngles);

    rectangle(post_img, Point(0,0), Point(600,600), Scalar(0,0,0), FILLED);

    for (int i = 0; i < idealPoints.size() - 1; i++)
    {
        line(post_img, idealPoints[i], idealPoints[i + 1], Scalar(0, 0, 255));
        circle(post_img, idealPoints[i], 2, Scalar(255, 0, 0));
    }

    imshow("Post", post_img);
    char c = (char)waitKey(0);
    return 0;
}