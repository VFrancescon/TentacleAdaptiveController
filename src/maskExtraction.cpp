#include <opencv2/opencv.hpp>

#include "AdaptiveControllerVisionClass/visionClass.hpp"

using namespace cv;

int main(int argc, char* argv[]) {
    Mat mask_src = imread(
        "/home/vittorio/TentacleAdaptiveController/test_imgs/INITIAL.png",
        IMREAD_COLOR);
    Mat inserted_src = imread(
        "/home/vittorio/TentacleAdaptiveController/test_imgs/INSERTED.png",
        IMREAD_COLOR);
    Mat inserted_src_backup = inserted_src.clone();
    VisionClass viz;
    viz.setLinkLenght(30);
    Mat isolatedPhantom = viz.isolatePhantom(mask_src);
    cvtColor(inserted_src_backup, inserted_src_backup, COLOR_BGR2GRAY);
    blur(inserted_src_backup, inserted_src_backup, Size(3, 3));
    threshold(inserted_src_backup, inserted_src_backup, 150, 255,
              THRESH_BINARY_INV);
    
    Mat disp;
    inserted_src_backup.copyTo(disp, isolatedPhantom);
    
    std::vector<Point> joints = viz.findJoints(disp);
    int k = 0;
    for(auto i: joints){
        circle(inserted_src, i, 5, Scalar(255, 0, 0), -1);
        putText(inserted_src, std::to_string(k), i, FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 0, 255), 2);    
        k++;
    }
    viz.setP0Frame(joints[0]);
    std::vector<double> desiredAngles = {-40, -5, -5, -5, 0};
    std::vector<Point> idealPoints = viz.computeIdealPoints(viz.getP0Frame(), desiredAngles); 
    for(auto i: idealPoints){
        circle(inserted_src, i, 5, Scalar(0, 255, 0), -1);
    }

    // imshow("inserted", inserted_src_backup);
    // imshow("isolated", isolatedPhantom);
    // imshow("disp", disp);
    imshow("inserted_src", inserted_src);
    waitKey(0);

    return 0;
}