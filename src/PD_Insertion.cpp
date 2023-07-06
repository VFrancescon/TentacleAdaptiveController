#include "PD_Insertion.hpp"

int rrows;
int rcols;

// const std::string rawFrame = "Raw Frame";
// const std::string phantom = "Phantom";
// const std::string processed = "Processed";

const std::string fullOutput = "Full Output";

int main(int argc, char *argv[]) {
    // namedWindow(rawFrame);
    // namedWindow(phantom);
    // namedWindow(processed);
    namedWindow(fullOutput);
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
    // std::ofstream recordPerformance;
    // recordPerformance.open("../PD_BORDERLESS_Results.csv",
    // std::ios_base::app); recordPerformance << date << "\n"; recordPerformance
    //     << "Step, JointNo, Ex(t), Ey(t), BaselineX, E_Multiplier, Bx, By,
    //     Bz\n";

    CompClass comp;
    VisionClass viz;
    viz.setThresholdLow(80);
    viz.setLinkLenght(15);
    viz.setHsvLow(0, 255, 162);
    int jointEff = 5;
    int jointNo = jointEff + 1;
    int jointMultiplier = 1;
    switch (jointMultiplier) {
        case 2:
            viz.setLinkLenght(15);
            break;

        default:
            viz.setLinkLenght(30);
            break;
    }
    // timesteps are equal to joint no
    int timesteps = jointEff;
    Vector3d reconciliationAngles = Vector3d{180, 90, 180};
    double EMultiplier = 5;
    /* * * * * * * * * * * * * * * * * * * * * * * * *
     * PRECOMPUTATION FOR EACH TIMESTEP BEGINS HERE  *
     * * * * * * * * * * * * * * * * * * * * * * * * */
    std::vector<Vector3d> AppliedFields;

    std::vector<double> DesiredAngles(jointNo);
    if (argc > 5) {
        DesiredAngles[0] = std::stod(argv[1]);
        DesiredAngles[1] = std::stod(argv[2]);
        DesiredAngles[2] = std::stod(argv[3]);
        DesiredAngles[3] = std::stod(argv[4]);
        DesiredAngles[4] = std::stod(argv[5]);
        DesiredAngles[jointEff] = 0;
    } else {
        DesiredAngles[0] = 5;
        DesiredAngles[1] = 5;
        DesiredAngles[2] = 15;
        DesiredAngles[3] = 15;
        DesiredAngles[4] = 10;
        DesiredAngles[jointEff] = 0;
    }
    if (argc == 2 || argc == 7) {
        jointMultiplier = std::stoi(argv[argc - 1]);
    }
    int rightHandBend = 0;
    rightHandBend =
        std::signbit(avgVect(DesiredAngles))
            ? -1
            : 1;  // signit returns 1 if argument is negative. 0 if positive

    std::vector<Vector3d> Magnetisations(jointNo);
    Magnetisations[0] = Vector3d(-0.0011, 0, -0.0028);
    Magnetisations[1] = Vector3d(-0.0028, 0, -0.001);
    Magnetisations[2] = Vector3d(0, 0, -0.003);
    Magnetisations[3] = Vector3d(-0.003, 0, 0);
    Magnetisations[4] = Vector3d(0, 0, -0.003);
    Magnetisations[jointEff] = Vector3d(0, 0, 0);

    // split angles and magnetisations
    std::vector<double> DesiredAnglesSPLIT(jointEff * jointMultiplier);
    std::vector<Vector3d> MagnetisationsSPLIT(jointEff * jointMultiplier);
    if (jointMultiplier > 1) {
        // convert sub5 joint numbers
        for (int i = 0; i < jointEff * jointMultiplier; i++) {
            DesiredAnglesSPLIT[i] =
                DesiredAngles[i / jointMultiplier] / jointMultiplier;
            MagnetisationsSPLIT[i] = Magnetisations[i / jointMultiplier];
        }
        DesiredAnglesSPLIT.push_back(0);
        MagnetisationsSPLIT.push_back(Vector3d(0, 0, 0));
    } else {
        std::cout << "Using defaults\n";
        DesiredAnglesSPLIT = DesiredAngles;
        MagnetisationsSPLIT = Magnetisations;
    }

    // instantiated positions, joints and links
    // std::vector<PosOrientation> iPosVec(jointEff * jointMultiplier + 1);
    // std::vector<Joint> iJoints(jointEff * jointMultiplier + 1);
    // for (int i = 0; i < iPosVec.size(); i++) {
    //     iJoints[i].assignPosOri(iPosVec[i]);
    // }
    // for (int i = 0; i < iJoints.size(); i++) {
    //     iJoints[i].q = Vector3d(0, DesiredAnglesSPLIT[i] * M_PI / 180, 0);
    //     iJoints[i].LocMag = MagnetisationsSPLIT[i];
    // }
    // // create vector of links for properties
    // std::vector<Link> iLinks(jointEff * jointMultiplier);

    // // assign stiffness
    // comp.adjustStiffness(iLinks, EMultiplier, jointMultiplier);

    int mid_OPMODE = 1;
    MiddlewareLayer mid(mid_OPMODE);
    mid.set3DField(0, 0, 0);

    // // 5. boot up Pylon backend
    // /**************************************************************
    //  *
    //  * PYLON SETUP
    //  *
    //  *****************************************************************/
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

    Size frameSize = Size((int)width.GetValue(), (int)height.GetValue());
    int codec = VideoWriter::fourcc('M', 'J', 'P', 'G');
    width.TrySetValue(1920, Pylon::IntegerValueCorrection_Nearest);
    height.TrySetValue(1200, Pylon::IntegerValueCorrection_Nearest);
    Pylon::CFloatParameter(camera.GetNodeMap(), "ExposureTime")
        .SetValue(25000.0);

    Pylon::CFloatParameter(camera.GetNodeMap(), "BslSaturation")
        .TrySetValue(1.5, Pylon::FloatValueCorrection_ClipToRange);

    Pylon::CFloatParameter(camera.GetNodeMap(), "BslHue")
        .TrySetValue(20, Pylon::FloatValueCorrection_ClipToRange);

    Pylon::CFloatParameter(camera.GetNodeMap(), "BslContrast")
        .TrySetValue(0.5, Pylon::FloatValueCorrection_ClipToRange);

    Pylon::CPixelTypeMapper pixelTypeMapper(&pixelFormat);
    Pylon::EPixelType pixelType =
        pixelTypeMapper.GetPylonPixelTypeFromNodeValue(
            pixelFormat.GetIntValue());
    camera.StartGrabbing(Pylon::GrabStrategy_LatestImageOnly);
    Pylon::CGrabResultPtr ptrGrabResult;
    camera.RetrieveResult(5000, ptrGrabResult,
                          Pylon::TimeoutHandling_ThrowException);

    Mat pre_img, post_img;
    try {
        const uint8_t *preImageBuffer = (uint8_t *)ptrGrabResult->GetBuffer();
    } catch (const Pylon::RuntimeException e) {
        std::cout << e.what() << "\n";
    }
    formatConverter.Convert(pylonImage, ptrGrabResult);
    pre_img = cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(),
                      CV_8UC3, (uint8_t *)pylonImage.GetBuffer());
    rrows = pre_img.rows * 3 / 8;
    rcols = pre_img.cols * 3 / 8;
    while (true) {
        Pylon::CGrabResultPtr ptrGrabResult;
        camera.RetrieveResult(5000, ptrGrabResult,
                              Pylon::TimeoutHandling_ThrowException);
        try {
            const uint8_t *preImageBuffer =
                (uint8_t *)ptrGrabResult->GetBuffer();
        } catch (const Pylon::RuntimeException e) {
            std::cout << e.what() << "\n";
        }
        formatConverter.Convert(pylonImage, ptrGrabResult);
        pre_img = cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(),
                          CV_8UC3, (uint8_t *)pylonImage.GetBuffer());
        resize(pre_img, pre_img, Size(), 0.375, 0.375);
        cv::imshow(fullOutput, pre_img);
        char c = (char)cv::waitKey(1);
        if (c == 27 || c == 10) {
            break;
        }
    }
    // cv::destroyAllWindows();
    // resizing the image for faster processing

    /*****************************************************************
     * Video Output Setup
     *****************************************************************/
    std::string angleSTR;
    for (auto i : DesiredAngles) {
        angleSTR += std::to_string(i) + "_";
    }

    std::string outputPath = "PD_INSERTION_" + angleSTR + "_" + date + ".avi";

    while (file_exists(outputPath)) {
        outputPath += "_1";
    }

    // VideoWriter video_out(outputPath, VideoWriter::fourcc('M', 'J', 'P',
    // 'G'),
    //                       10, Size(rcols, rrows));

    int error = 0, prev_xerror = 0, prev_yerror = 0;
    int dx_error = 0, dy_error = 0;
    int step_count = 0;
    Point p0 = Point{rcols / 2, 0};
    bool finished = true;
    int baseline_error;
    int signFlag;

    // std::cout << "Ready to go. Press enter";
    // std::cin.get();
    // mid.retractIntroducer(10);

    auto start = std::chrono::high_resolution_clock::now();
    auto end = std::chrono::high_resolution_clock::now();
    bool controllerActive = true;
    bool first_run;
    bool solving_time = false;

    int joints_found = 0;
    int joints_to_solve = 2;

    bool initialSetup = true;
    Mat phantom_mask;
    while (camera.IsGrabbing()) {
        // 1. get a phantom mask
        camera.RetrieveResult(5000, ptrGrabResult,
                              Pylon::TimeoutHandling_ThrowException);
        const uint8_t *pImageBuffer = (uint8_t *)ptrGrabResult->GetBuffer();
        formatConverter.Convert(pylonImage, ptrGrabResult);

        if (initialSetup) {
            pre_img =
                cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(),
                        CV_8UC3, (uint8_t *)pylonImage.GetBuffer());
            if (pre_img.empty()) {
                break;
            }
            resize(pre_img, pre_img, Size(rcols, rrows), INTER_LINEAR);
            phantom_mask = viz.isolatePhantom(pre_img);
            std::vector<Mat> prelimShow = {phantom_mask, pre_img};
            Mat prelimOut;
            vconcat(prelimShow, prelimOut);
            imshow(fullOutput, prelimOut);
            char key = (char)waitKey(0);
            if (key == 27) {
                break;
            }
            // cv::destroyAllWindows();

            // 2. push 1 joint in.
            mid.retractIntroducer(10);
            first_run = true;
            initialSetup = false;
        } else {  // we have established a phantom mask

            Mat grabbedFrame =
                cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(),
                        CV_8UC3, (uint8_t *)pylonImage.GetBuffer());
            resize(grabbedFrame, grabbedFrame, Size(rcols, rrows),
                   INTER_LINEAR);
            Mat processed_frame = viz.preprocessImg(grabbedFrame);
            std::vector<Point> Joints;
            if (first_run)
                Joints = viz.findJoints(processed_frame);
            else
                Joints = viz.findJoints(processed_frame, p0);
            joints_found = Joints.size();
            solving_time = joints_found == joints_to_solve;
            if (joints_found != 0) {
                p0 = Joints.at(0);
                // std::cout << "p0 is " << p0.x << "," << p0.y << "\n";
                viz.setP0Frame(p0);
                first_run = false;
                for (auto i : Joints) {
                    circle(grabbedFrame, i, 5, Scalar(0, 0, 255), -1);
                }
            }

            if (solving_time) {  // run the controller here
                std::cout << "Controller would run here\n";
                /**
                 * @brief calculating splits and visualising here
                 * 
                 */
                std::vector<double> dAnglesS = std::vector<double>(
                    DesiredAnglesSPLIT.begin(),
                    DesiredAnglesSPLIT.begin() + joints_to_solve);

                std::vector<Vector3d> MagS = std::vector<Vector3d>(
                    MagnetisationsSPLIT.end() - joints_to_solve - 1,
                    MagnetisationsSPLIT.end());

                std::vector<Point> dPoints =
                    viz.computeIdealPoints(p0, dAnglesS);

                // for (auto i : dPoints) {
                //     circle(grabbedFrame, i, 5, Scalar(255, 0, 0), -1);
                // }
                for (int i = 0; i < dPoints.size(); i++) {
                    // std::cout << " " << i;
                    line(post_img, dPoints[i], dPoints[i + 1],
                         Scalar(0, 0, 255), 2);
                    circle(post_img, dPoints[i], 3, Scalar(0, 255, 0), FILLED);
                    // putText(post_img, std::to_string(i), dPoints[i],
                    //         FONT_HERSHEY_SIMPLEX, 1.0, Scalar(255, 0, 0));
                }
                circle(post_img, dPoints[dPoints.size() - 1], 3,
                       Scalar(0, 255, 0), FILLED);
                /**
                 * @brief calculating splits and visualising here
                 * 
                 */
                
                /**
                 * @brief calculate errors here
                 * 
                 */
                

                /**
                 * @brief and run the controller here
                 * 
                 */
                joints_to_solve++;
            } else if (joints_found > joints_to_solve) {
                joints_to_solve++;
            } else
                mid.retractIntroducer(10);

            // imshow(rawFrame, grabbedFrame);
            // imshow(processed, processed_frame);
            // imshow(phantom, phantom_mask);
            viz.drawLegend(grabbedFrame);
            std::vector<Mat> outputImgs = {grabbedFrame, processed_frame,
                                           phantom_mask};
            Mat output;
            vconcat(outputImgs, output);
            imshow("output", output);
            char c = (char)waitKey(0);
            if (c == 27) {
                break;
            }
        }  // we have established a phantom mask

    }  // while camera is grabbing

    // video_out.release();
    std::cout << "retracting by " << mid.stepper_count << " steps\n";
    mid.stepIntroducer(mid.stepper_count);
    camera.StopGrabbing();
    camera.DestroyDevice();
    // recordPerformance.close();
    return 0;
}