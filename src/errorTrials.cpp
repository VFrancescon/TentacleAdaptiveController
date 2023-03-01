#include "ControllerPrototype.hpp"

double upperError = 10e3;
double lowError = 7e3;

int main(int argc, char* argv[]){
    int jointEff = 5;
    int jointNo = jointEff + 1;
    std::vector<int> DesiredAngles(jointNo);
    DesiredAngles[0] = 10;
    DesiredAngles[1] = 15;
    DesiredAngles[2] = 30;
    DesiredAngles[3] = 25;
    DesiredAngles[4] = 20;
    DesiredAngles[jointEff] = 0;

    /**************************************************************
     *
     *
     * PYLON SETUP
     *
     *
     *****************************************************************/
    Mat pre_img, post_img, intr_mask;
    Pylon::PylonInitialize();
    Pylon::CImageFormatConverter formatConverter;
    formatConverter.OutputPixelFormat = Pylon::PixelType_BGR8packed;
    Pylon::CPylonImage pylonImage;
    Pylon::CInstantCamera camera(Pylon::CTlFactory::GetInstance().CreateFirstDevice());
    camera.Open();
    Pylon::CIntegerParameter width(camera.GetNodeMap(), "Width");
    Pylon::CIntegerParameter height(camera.GetNodeMap(), "Height");
    Pylon::CEnumParameter pixelFormat(camera.GetNodeMap(), "PixelFormat");
    Pylon::CFloatParameter(camera.GetNodeMap(), "ExposureTime").SetValue(20000.0);
    Size frameSize = Size((int)width.GetValue(), (int)height.GetValue());
    int codec = VideoWriter::fourcc('M', 'J', 'P', 'G');
    width.TrySetValue(PYLON_WIDTH, Pylon::IntegerValueCorrection_Nearest);
    height.TrySetValue(PYLON_HEIGHT, Pylon::IntegerValueCorrection_Nearest);
    Pylon::CPixelTypeMapper pixelTypeMapper(&pixelFormat);
    Pylon::EPixelType pixelType = pixelTypeMapper.GetPylonPixelTypeFromNodeValue(pixelFormat.GetIntValue());

    camera.StartGrabbing(Pylon::GrabStrategy_LatestImageOnly);
    Pylon::CGrabResultPtr ptrGrabResult;
    camera.RetrieveResult(5000, ptrGrabResult, Pylon::TimeoutHandling_ThrowException);
    const uint8_t *preImageBuffer = (uint8_t *)ptrGrabResult->GetBuffer();
    formatConverter.Convert(pylonImage, ptrGrabResult);
    pre_img = cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC3, (uint8_t *)pylonImage.GetBuffer());

    int rrows = pre_img.rows * 3 / 8;
    int rcols = pre_img.cols * 3 / 8;
    Point p0 = Point{-2000, 2000};

    while (camera.IsGrabbing())
    {
        camera.RetrieveResult(5000, ptrGrabResult, Pylon::TimeoutHandling_ThrowException);
        const uint8_t *pImageBuffer = (uint8_t *)ptrGrabResult->GetBuffer();
        formatConverter.Convert(pylonImage, ptrGrabResult);
        post_img = cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC3, (uint8_t *)pylonImage.GetBuffer());

        if (post_img.empty())
        {
            break;
        }
        resize(post_img, post_img, Size(rcols,rrows), INTER_LINEAR);
        Mat post_img_grey, post_img_th;
        Mat post_img_masked = Mat::zeros(Size(rcols,rrows), CV_8UC1);

        cvtColor(post_img, post_img_grey, COLOR_BGR2GRAY);
        blur(post_img_grey, post_img_grey, Size(5, 5));
        threshold(post_img_grey, post_img_th, threshold_low, threshold_high, THRESH_BINARY_INV);
        // post_img_th.copyTo(post_img_masked, intr_mask);
        post_img_th.copyTo(post_img_masked);

        std::vector<Point> Joints;
        std::vector<std::vector<Point>> contours;

        Joints = findJoints(post_img_masked, contours);
        int JointsObserved = Joints.size();

        for (auto i : Joints)
        {
            circle(post_img, i, 4, Scalar(255, 0, 0), FILLED);
        }
        drawContours(post_img, contours, -1, Scalar(255, 255, 0));

        std::vector<double> angles;
        std::vector<double> desiredAngles_ = std::vector<double>(DesiredAngles.end() - JointsObserved - 1, DesiredAngles.end() - 1);
        std::vector<Point> idealPoints;
        if (p0 == Point{-2000, 2000})
            p0 = Joints[0];

        idealPoints = computeIdealPoints(p0, desiredAngles_);
        angles = computeAngles(Joints);
        for (int i = 0; i < idealPoints.size() - 1; i++)
        {
            line(post_img, idealPoints[i], idealPoints[i + 1], Scalar(0, 0, 255));
            circle(post_img, idealPoints[i], 2, Scalar(255, 0, 0));
        }

        // if(JointsObserved != jointsCached){

        int jointsCached = JointsObserved;
        std::vector<double> dAngleSlice = std::vector<double>(desiredAngles_.end() - angles.size(), desiredAngles_.end());
        // std::vector<double> dAngleSlice = desiredAngles_;
        int error = meanError(dAngleSlice, angles);
        int errorPiecewise = pieceWiseError(dAngleSlice, angles);
        int errorPiecewiseWeighted = pieceWiseErrorWeighted(dAngleSlice, angles);

        std::cout << "--------------------\nError Dump\n";
        std::cout << "DesiredAngles vs Observed Angles\n";
        for(int i = 0; i < dAngleSlice.size(); i++){
            std::cout << dAngleSlice[i] << " - " << angles[i] << "\n";
        }

        std::cout << "meanError: " << error << "\n";
        std::cout << "piecewise: " << errorPiecewise << "\n";
        std::cout << "piecewiseweighted: " << errorPiecewiseWeighted << "\n";

        imshow("Post", post_img);
        char c = (char)waitKey(0);
        if (c == 27)
            break;
    }

    return 0;
}