#include "ControllerPrototype.hpp"
#include "HCoilMiddlewareLib/HCoilMiddlewareLib.hpp"


int main(int argc, char *argv[]) {
    /**
     * Get today's date
     */
    time_t curr_time;
    tm *curr_tm;
    time(&curr_time);
    curr_tm = localtime(&curr_time);
    char date_string[100];
    strftime(date_string, 50, "%d_%m_%y_%H%M%S", curr_tm);
    std::string date(date_string);
    std::ofstream recordPerformance;
    recordPerformance.open("../PD_BORDERLESS_Results.csv", std::ios_base::app);
    recordPerformance << date << "\n";
    recordPerformance
        << "Step, Error(t), Error(t-1), E_Multiplier, Bx, By, Bz\n";

    int jointEff = 5;
    int jointNo = jointEff + 1;
    int jointMultiplier = 2;

    // timesteps are equal to joint no
    int timesteps = jointEff;
    Vector3d reconciliationAngles = Vector3d{0, 0, 180};
    double EMultiplier = 5;
    /* * * * * * * * * * * * * * * * * * * * * * * * *
     * PRECOMPUTATION FOR EACH TIMESTEP BEGINS HERE  *
     * * * * * * * * * * * * * * * * * * * * * * * * */
    std::vector<Vector3d> AppliedFields;

    std::vector<double> DesiredAngles(jointNo);
    if (argc == 6) {
        DesiredAngles[0] = std::stod(argv[1]);
        DesiredAngles[1] = std::stod(argv[2]);
        DesiredAngles[2] = std::stod(argv[3]);
        DesiredAngles[3] = std::stod(argv[4]);
        DesiredAngles[4] = std::stod(argv[5]);
        DesiredAngles[jointEff] = 0;
    } else {
        DesiredAngles[0] = 20;
        DesiredAngles[1] = 10;
        DesiredAngles[2] = 10;
        DesiredAngles[3] = 15;
        DesiredAngles[4] = 15;
        DesiredAngles[jointEff] = 0;
    }
    bool rightHandBend = true;
    rightHandBend =
        std::signbit(avgVect(DesiredAngles)) ? -1: 1;  // signit returns 1 if argument is negative. 0 if positive

    std::vector<Vector3d> Magnetisations(jointNo);
    Magnetisations[0] = Vector3d(-0.0011, 0, -0.0028);
    Magnetisations[1] = Vector3d(-0.0028, 0, -0.001);
    Magnetisations[2] = Vector3d(0, 0, -0.003);
    Magnetisations[3] = Vector3d(-0.003, 0, 0);
    Magnetisations[4] = Vector3d(0, 0, -0.003);
    Magnetisations[jointEff] = Vector3d(0, 0, 0);

    std::vector<double> DesiredAnglesSPLIT(jointEff*jointMultiplier);
    std::vector<Vector3d> MagnetisationsSPLIT(jointEff*jointMultiplier);
    if(jointMultiplier > 1){
    //convert sub5 joint numbers
        for(int i = 0; i < jointEff * jointMultiplier; i++) {
            DesiredAnglesSPLIT[i] = DesiredAngles[i / jointMultiplier] / jointMultiplier;
            MagnetisationsSPLIT[i] = Magnetisations[i / jointMultiplier];
        }
        DesiredAnglesSPLIT.push_back(0);
        MagnetisationsSPLIT.push_back(Vector3d(0, 0, 0));
    } else {
        std::cout << "Using defaults\n";
        DesiredAnglesSPLIT = DesiredAngles;
        MagnetisationsSPLIT = Magnetisations;
    }

    std::vector<PosOrientation> iPosVec(jointEff * jointMultiplier + 1);
    std::vector<Joint> iJoints(jointEff * jointMultiplier + 1);
    for (int i = 0; i < iPosVec.size(); i++)
    {
        iJoints[i].assignPosOri(iPosVec[i]);
    }

    for (int i = 0; i < iJoints.size(); i++)
    {
        iJoints[i].q = Vector3d(0, DesiredAnglesSPLIT[i] * M_PI / 180, 0);
        iJoints[i].LocMag = MagnetisationsSPLIT[i];
    }
    // create vector of links for properties
    std::vector<Link> iLinks(jointEff * jointMultiplier);
    adjustStiffness(iLinks, EMultiplier, jointMultiplier);
    Vector3d field = CalculateField(iLinks, iJoints, iPosVec);
    field = RotateField(field, reconciliationAngles);
    field(1) = 0;
    double bx = field(0);
    double by = field(1);
    double bz = field(2);
    std::cout << "Initial answer:\n" << field << "\n";

    /**************************************************************
     *
     * Middleware Setup
     *
     *****************************************************************/
    MiddlewareLayer mid(true);

    /**************************************************************
     *
     * PYLON SETUP
     *
     *****************************************************************/
    Mat pre_img, post_img, intr_mask;
    Pylon::PylonInitialize();
    Pylon::CImageFormatConverter formatConverter;
    formatConverter.OutputPixelFormat = Pylon::PixelType_BGR8packed;
    Pylon::CPylonImage pylonImage;
    Pylon::CInstantCamera camera(
        Pylon::CTlFactory::GetInstance().CreateFirstDevice());
    camera.Open();
    Pylon::CIntegerParameter width(camera.GetNodeMap(), "Width");
    Pylon::CIntegerParameter height(camera.GetNodeMap(), "Height");
    Pylon::CEnumParameter pixelFormat(camera.GetNodeMap(), "PixelFormat");

    Pylon::CFloatParameter(camera.GetNodeMap(), "ExposureTime")
        .SetValue(20000.0);

    Size frameSize = Size((int)width.GetValue(), (int)height.GetValue());
    int codec = VideoWriter::fourcc('M', 'J', 'P', 'G');
    width.TrySetValue(PYLON_WIDTH, Pylon::IntegerValueCorrection_Nearest);
    height.TrySetValue(PYLON_HEIGHT, Pylon::IntegerValueCorrection_Nearest);
    Pylon::CPixelTypeMapper pixelTypeMapper(&pixelFormat);
    Pylon::EPixelType pixelType =
        pixelTypeMapper.GetPylonPixelTypeFromNodeValue(
            pixelFormat.GetIntValue());
    camera.StartGrabbing(Pylon::GrabStrategy_LatestImageOnly);
    Pylon::CGrabResultPtr ptrGrabResult;
    camera.RetrieveResult(5000, ptrGrabResult,
                          Pylon::TimeoutHandling_ThrowException);
    const uint8_t *preImageBuffer = (uint8_t *)ptrGrabResult->GetBuffer();
    formatConverter.Convert(pylonImage, ptrGrabResult);
    pre_img = cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(),
                      CV_8UC3, (uint8_t *)pylonImage.GetBuffer());

    // resizing the image for faster processing
    int rrows = pre_img.rows * 3 / 8;
    int rcols = pre_img.cols * 3 / 8;

    /**************************************************************
     *
     *
     * Video Output Setup
     *
     *
     *****************************************************************/
    std::string outputPath = "PD_BORDERLESS_" + date + ".avi";

    while (file_exists(outputPath)) {
        outputPath += "_1";
    }

    VideoWriter video_out(outputPath, VideoWriter::fourcc('M', 'J', 'P', 'G'),
                          10, Size(rcols, rrows));

    // resize(pre_img, pre_img, Size(rcols, rrows), INTER_LINEAR);
    // Mat pre_img1 = Mat::zeros(Size(rcols, rrows), CV_8UC3);
    // intr_mask = IntroducerMask(pre_img1);
    intr_mask = IntroducerMask(pre_img);
    int jointsCached = 0;
    int error = 0, prev_xerror = 0, prev_yerror = 0;
    int dx_error = 0, dy_error = 0;
    int step_count = 0;
    Point p0 = Point{-2000, 2000};
    bool firstRun = true;
    bool finished = false;
    int baseline_error;
    int zero_count;
    int signFlag;
    recordPerformance << step_count << "," << error << "," << dx_error << ","
                      << EMultiplier << "," << field(0) << "," << field(1)
                      << "," << field(2) << "\n";
    std::cout << "Ready to go. Press enter";
    std::cin.get();

    double success_val = 0.2;
    double low_val = 0.35;
    int rightFlag = (rightFlag ? 1 : -1);
    while (camera.IsGrabbing()) {
        camera.RetrieveResult(5000, ptrGrabResult,
                              Pylon::TimeoutHandling_ThrowException);
        const uint8_t *pImageBuffer = (uint8_t *)ptrGrabResult->GetBuffer();
        formatConverter.Convert(pylonImage, ptrGrabResult);
        post_img =
            cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(),
                    CV_8UC3, (uint8_t *)pylonImage.GetBuffer());

        if (post_img.empty()) {
            break;
        }
        resize(post_img, post_img, Size(rcols, rrows), INTER_LINEAR);
        Mat post_img_grey, post_img_th;
        Mat post_img_masked = Mat::zeros(Size(rcols, rrows), CV_8UC1);

        cvtColor(post_img, post_img_grey, COLOR_BGR2GRAY);
        blur(post_img_grey, post_img_grey, Size(5, 5));
        threshold(post_img_grey, post_img_th, threshold_low, threshold_high,
                  THRESH_BINARY_INV);
        // post_img_th.copyTo(post_img_masked, intr_mask);
        post_img_th.copyTo(post_img_masked);

        std::vector<Point> Joints;
        std::vector<std::vector<Point>> contours;

        Joints = findJoints(post_img_masked, contours, jointEff*jointMultiplier+1);
        int JointsObserved = Joints.size();
        for (auto i : Joints) {
            circle(post_img, i, 4, Scalar(255, 0, 0), FILLED);
        }
        drawContours(post_img, contours, -1, Scalar(255, 255, 0));
        std::vector<double> angles;
        std::vector<double> desiredAngles_ =
            std::vector<double>(DesiredAnglesSPLIT.begin(), DesiredAnglesSPLIT.end() - 1);
        std::vector<Point> idealPoints;
        // if (p0 == Point{-2000, 2000})
        p0 = Joints[0];

        idealPoints = computeIdealPoints(p0, DesiredAnglesSPLIT);
        // std::cout << "Desired angles slice size: " << DesiredAngles.size() <<
        // "\n";

        angles = computeAngles(Joints);
        for (int i = 0; i < idealPoints.size() - 1; i++) {
            // std::cout << " " << i;
            line(post_img, idealPoints[i], idealPoints[i + 1],
                 Scalar(0, 0, 255), 2);
            circle(post_img, idealPoints[i], 3, Scalar(0, 255, 0), FILLED);
            putText(post_img, std::to_string(i), idealPoints[i],
                    FONT_HERSHEY_SIMPLEX, 1.0, Scalar(255, 0, 0));
        }
        // std::cout << "\n";
        circle(post_img, idealPoints[idealPoints.size() - 1], 3,
               Scalar(0, 255, 0), FILLED);
        putText(post_img, std::to_string(idealPoints.size() - 1),
                idealPoints[idealPoints.size() - 1], FONT_HERSHEY_SIMPLEX, 1.0,
                Scalar(255, 0, 0));

        // #region legend
        /**
         * @brief DRAW A LEGEND
         */
        cv::Point TopLeftLegend(0, 390);
        cv::Point BottomRightLegend(230, 450);
        cv::Point InnerTopLeftLegend(4, 404);
        cv::Point InnerBottomRightLegend(196, 446);

        rectangle(post_img, TopLeftLegend, BottomRightLegend, Scalar(0, 0, 0),
                  4);
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
        /**
         * @brief DRAW A LEGEND
         */
        // #endregion

        jointsCached = JointsObserved;
        std::vector<double> dAngleSlice = std::vector<double>(
            desiredAngles_.end() - angles.size(), desiredAngles_.end());
        // std::vector<double> dAngleSlice = desiredAngles_;

        std::vector<double> desiredX, observedX, desiredY, observedY;
        for(auto i: idealPoints){
            desiredX.push_back(i.x);
            desiredY.push_back(i.y);
        }
        for(auto i: Joints){
            observedX.push_back(i.x);
            observedY.push_back(i.y);
        }
        double xError = xwiseError(desiredX, observedX);
        double yError = ywiseError(desiredY, observedY);
        dx_error = xError - prev_xerror;
        dy_error = yError - prev_yerror;
        prev_xerror = xError;
        prev_yerror = yError;

        if(firstRun){
            baseline_error = (xError + yError) / 2;
            firstRun = false;
        }
        double baselineX = ((xError + yError) / 2) / baseline_error;
        int xFlag = std::signbit(xError) ? 1 : -1;
        int yFlag = std::signbit(yError) ? -1 : 1;
        int signFlag;
        if( xFlag == -1 && yFlag == 1){
            signFlag = -1;
        } else signFlag = xFlag;

        double Kp = 1;
        double Kd = derivativeAdjustmentF(dx_error);


        if (finished) {
            std::cout << "Victory\n";
            recordPerformance << step_count << "," << error << "," << dx_error
                              << "," << EMultiplier << "," << field(0) << ","
                              << field(1) << "," << field(2) << "\n";
            std::cout << "End of operations\n";
            imshow("Post", post_img);
            video_out.write(post_img);
            char c = (char)waitKey(0);
            if (c == 27)
                break;
            else
                continue;
        }
        if (baselineX < 0.25) { 
            finished = true;
            continue;
        } else if ( baseline_error > 0.25 && baseline_error < 0.5 ) { 
            std::cout << "Adjusting field from\n" << field << "\n";
            field += ( Kp * Kd) * signFlag * field;
            std::cout << "To\n" << field << "\n";
        } else { 
            std::cout << "Adjusting Emultiplier from " << EMultiplier << " to ";
            EMultiplier += (signFlag * Kd * 10 / 2);
            std::cout << EMultiplier << "\n";
            adjustStiffness(iLinks, EMultiplier);
            field = CalculateField(iLinks, iJoints, iPosVec);
            field = RotateField(field, reconciliationAngles);
        }

        field(1) = abs(field(0)) * -0.5;
        bx = field(0);
        by = field(1);
        bz = field(2);
        std::cout << "E: " << EMultiplier << " applied field:\n"
                  << field << "\n";

        if (abs(field(0)) > 20 && abs(field(2)) > 15 && abs(field(1)) > 20)
            break;
        if (EMultiplier < 0) {
            EMultiplier = 0;
            continue;
        }

        mid.set3DField(field);
        step_count++;
        recordPerformance << step_count << "," << error << "," << dx_error << ","
                          << EMultiplier << "," << field(0) << "," << field(1)
                          << "," << field(2) << "\n";

        imshow("Post", post_img);
        video_out.write(post_img);
        char c = (char)waitKey(1e3);
        if (c == 27) break;
    }
    video_out.release();
    mid.~MiddlewareLayer();
    return 0;
}
