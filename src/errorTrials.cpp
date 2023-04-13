#include "ControllerPrototype.hpp"

double upperError = 10e3;
double lowError = 7e3;

int main(int argc, char* argv[]){
    int jointEff = 5;
    int jointNo = jointEff + 1;
    std::vector<double> DesiredAngles(jointNo);
    if (argc == 6)
    {
        DesiredAngles[0] = std::stod(argv[1]);
        DesiredAngles[1] = std::stod(argv[2]);
        DesiredAngles[2] = std::stod(argv[3]);
        DesiredAngles[3] = std::stod(argv[4]);
        DesiredAngles[4] = std::stod(argv[5]);
        DesiredAngles[jointEff] = 0;
    } else {
        DesiredAngles[0] = 45;
        DesiredAngles[1] = 0;
        DesiredAngles[2] = 0;
        DesiredAngles[3] = 0;
        DesiredAngles[4] = 0;
        DesiredAngles[jointEff] = 0;
    }
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
        resize(post_img, post_img, Size(rcols, rrows), INTER_LINEAR);
        Mat post_img_grey, post_img_th;
        Mat post_img_masked = Mat::zeros(Size(rcols, rrows), CV_8UC1);

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
        std::vector<Point> idealPoints;
        // if (p0 == Point{-2000, 2000})
            p0 = Joints[0];

        idealPoints = computeIdealPoints(p0, DesiredAngles);
        // std::cout << "Desired angles slice size: " << DesiredAngles.size() << "\n";

        angles = computeAngles(Joints);
        for (int i = 0; i < idealPoints.size() - 1; i++)
        {
            // std::cout << " " << i;
            line(post_img, idealPoints[i], idealPoints[i + 1], Scalar(0, 0, 255), 2);
            circle(post_img, idealPoints[i], 3, Scalar(0, 255, 0), FILLED);
            putText(post_img, std::to_string(i), idealPoints[i],
                    FONT_HERSHEY_SIMPLEX, 1.0, Scalar(255, 0, 0));
        }
        // std::cout << "\n";
        circle(post_img, idealPoints[ idealPoints.size()-1], 3, Scalar(0, 255, 0), FILLED);
        putText(post_img, std::to_string(idealPoints.size()-1), idealPoints[idealPoints.size() -1],
                FONT_HERSHEY_SIMPLEX, 1.0, Scalar(255, 0, 0));
        angles = computeAngles(Joints);
        for (int i = 0; i < idealPoints.size() - 1; i++)
        {
            line(post_img, idealPoints[i], idealPoints[i + 1], Scalar(0, 0, 255), 2);
            circle(post_img, idealPoints[i], 3, Scalar(0, 255, 0), FILLED);
        }

        /**
         * @brief DRAW A LEGEND
         */
        cv::Point TopLeftLegend(0, 390);
        cv::Point BottomRightLegend(230, 450);
        cv::Point InnerTopLeftLegend(4, 404);
        cv::Point InnerBottomRightLegend(196, 446);

        rectangle(post_img, TopLeftLegend, BottomRightLegend, Scalar(0, 0, 0), 4);
        rectangle(post_img, InnerTopLeftLegend, InnerBottomRightLegend, Scalar(255, 255, 255), 1);

        putText(post_img, "Detected",
                TopLeftLegend + Point(84, 26), FONT_HERSHEY_DUPLEX,
                1, Scalar(0, 0, 0));
        putText(post_img, "Desired",
                TopLeftLegend + Point(84, 52), FONT_HERSHEY_DUPLEX,
                1, Scalar(0, 0, 0));
        //blue rect. Desired
        rectangle(post_img, TopLeftLegend + Point(5, 12), TopLeftLegend + Point(80, 22),
                  Scalar(255, 0, 0), FILLED);
        //red rect. Detected
        rectangle(post_img, TopLeftLegend + Point(5, 38), TopLeftLegend + Point(80, 48),
                  Scalar(0, 0, 255), FILLED);
        /**
         * @brief DRAW A LEGEND
         */


        // if(JointsObserved != jointsCached){

        // std::vector<double> dAngleSlice = std::vector<double>(desiredAngles_.end() - angles.size(), desiredAngles_.end());
        // std::vector<double> dAngleSlice = desiredAngles_;
        int error = pieceWiseErrorWeighted(DesiredAngles, angles);
        int pointError = positionWiseError(idealPoints, Joints);
        std::cout << "\n\n---------------------------------------------------------\n\n";

        // Controller Logic
        // if e < 0: signFlag = -1
        // else signFlag = 1
        // then e = abs(e)
        // Scenario 1. e < LowS -> Do Nothing
        // Scenario 2. LowS < e < HighS -> Field + P*signFlag
        // Scenario 3. e > HighS -> K += signFlag
        int signFlag = (error < 0) ? -1 : 1;
        std::cout << "Piecewise Error " << error << "\n";
        std::cout << "Pointwise Error " << pointError << "\n";

        imshow("Post", post_img);
        char c = (char)waitKey(5e2);
        if (c == 27)
            break;
    }

    return 0;
}