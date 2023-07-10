#include "PD_Insertion.hpp"

int rrows;
int rcols;

const std::string rawFrame = "Raw Frame";
const std::string phantom = "Phantom";
const std::string processed = "Processed";

int main(int argc, char *argv[]) {
    namedWindow(rawFrame);
    namedWindow(phantom);
    namedWindow(processed);
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
    Vector3d reconciliationAngles = Vector3d{90, 0, 180};
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
        DesiredAngles[0] = -20;
        DesiredAngles[1] = -10;
        DesiredAngles[2] = -10;
        DesiredAngles[3] = -5;
        DesiredAngles[4] = 0;
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

    // comp.adjustStiffness(iLinks, EMultiplier, jointMultiplier);

    int mid_OPMODE = 1;
    MiddlewareLayer mid(mid_OPMODE);
    Vector3d field = Vector3d(0, 0, 0);
    mid.set3DField(field);

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
        cv::imshow(rawFrame, pre_img);
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
    // std::string angleSTR;
    // for (auto i : DesiredAngles) {
    //     angleSTR += std::to_string(i) + "_";
    // }

    std::string outputPath = "PD_INSERTION_"  + date + ".avi";
    std::string procPath = "PD_INSERTION_PROC_" + date + ".avi";
    while (file_exists(outputPath)) {
        outputPath += "_1";
    }

    VideoWriter rawVideoOut(outputPath, VideoWriter::fourcc('M', 'J', 'P', 'G'),
                          10, Size(rcols, rrows));
    VideoWriter procVideoOut(procPath, VideoWriter::fourcc('M', 'J', 'P', 'G'),
                          10, Size(rcols, rrows));

    int error = 0, prev_xerror = 0, prev_yerror = 0;
    int dx_error = 0, dy_error = 0;
    int step_count = 0;
    Point p0 = Point{396, 75};
    bool finished = true;
    int baseline_error;
    int signFlag;

    // std::cout << "Ready to go. Press enter";
    // std::cin.get();
    // mid.retractIntroducer(10);

    auto start = std::chrono::high_resolution_clock::now();
    auto end = std::chrono::high_resolution_clock::now();
    bool controllerActive = true;
    bool settingUpController = true;
    bool first_run;
    bool solving_time = false;

    bool got_baseline = false;

    int joints_found = 0;
    int joints_to_solve = 2;

    bool initialSetup = true;
    Mat phantom_mask;

    std::vector<Link> iLinks;
    std::vector<PosOrientation> iPosVec;
    std::vector<Joint> iJoints;
    float bx = field(0);
    float by = field(1);
    float bz = field(2);
    double xError = 0, yError = 0, EAdjust = 0;
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
            imshow(phantom, phantom_mask);
            imshow(rawFrame, pre_img);
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
            rawVideoOut.write(grabbedFrame);
            Mat processed_frame = viz.preprocessImg(grabbedFrame);
            std::vector<Point> Joints;
            if (first_run) {
                Joints = viz.findJoints(processed_frame);
            } else
                Joints = viz.findJoints(processed_frame, p0);
            joints_found = Joints.size();
            solving_time = joints_found == joints_to_solve;
            if (joints_found != 0) {
                p0 = Joints.at(0);
                // std::cout << "p0 is " << p0.x << "," << p0.y << "\n";
                viz.setP0Frame(p0);
                first_run = false;
                for (auto i : Joints) {
                    circle(grabbedFrame, i, 5, Scalar(255, 0, 0), -1);
                }
                std::vector<Link> iLinks(joints_found);
            }
            /**
             * @brief calculating splits and visualising here
             *
             */
            std::vector<double> dAnglesS = std::vector<double>(
                DesiredAnglesSPLIT.begin(),
                DesiredAnglesSPLIT.begin() + joints_to_solve);

            std::vector<Point> dPoints = viz.computeIdealPoints(p0, dAnglesS);
            for (int i = 0; i < dPoints.size() - 1; i++) {
                // std::cout << " " << i;
                line(grabbedFrame, dPoints[i], dPoints[i + 1],
                     Scalar(0, 0, 255), 2);
                circle(grabbedFrame, dPoints[i], 3, Scalar(0, 0, 255), FILLED);
                // putText(post_img, std::to_string(i), dPoints[i],
                //         FONT_HERSHEY_SIMPLEX, 1.0, Scalar(255, 0, 0));
            }
            circle(grabbedFrame, dPoints[dPoints.size() - 1], 3,
                   Scalar(0, 255, 0), FILLED);

            if (solving_time) {  // run the controller here
                std::cout << "Controller would run here\n";

                /**
                 * @brief calculate errors here
                 *
                 */
                std::vector<double> desiredX, observedX, desiredY, observedY;
                for (auto i : dPoints) {
                    desiredX.push_back(i.x);
                    desiredY.push_back(i.y);
                }
                for (auto i : Joints) {
                    observedX.push_back(i.x);
                    observedY.push_back(i.y);
                }
                xError = xwiseError(desiredX, observedX);
                yError = ywiseError(desiredY, observedY);
                dx_error = xError - prev_xerror;
                dy_error = yError - prev_yerror;
                prev_xerror = xError;
                prev_yerror = yError;
                if (!got_baseline) {
                    baseline_error = sqrt(pow(xError, 2) + pow(yError, 2));
                    got_baseline = true;
                }
                double baseline_wrt_X =
                    ((abs(xError) + yError) / 2) / baseline_error;
                int xFlag = std::signbit(xError) ? -1 : 1;
                int yFlag = std::signbit(yError) ? -1 : 1;
                int signFlag = xFlag;
                double Kp = 0.9;
                double Kd = derivativeAdjustmentF(dx_error);

                std::vector<Vector3d> MagS = std::vector<Vector3d>(
                    MagnetisationsSPLIT.end() - joints_to_solve - 1,
                    MagnetisationsSPLIT.end());

                if (settingUpController) {
                    iLinks.clear();
                    iJoints.clear();
                    iPosVec.clear();
                    for (int i = 0; i < joints_to_solve + 1; i++) {
                        iJoints.push_back(Joint());
                        iPosVec.push_back(PosOrientation());
                        iJoints[i].assignPosOri(iPosVec[i]);
                    }
                    for (int i = 0; i < iJoints.size(); i++) {
                        iJoints[i].q = Vector3d(0, dAnglesS[i] * M_PI / 180, 0);
                        iJoints[i].LocMag = MagS[i];
                    }
                    for (int i = 0; i < iJoints.size() - 1; i++) {
                        iLinks.push_back(Link());
                    }
                    comp.adjustStiffness(iLinks, EMultiplier, jointMultiplier);
                    field = comp.CalculateField(iLinks, iJoints, iPosVec);
                    field = comp.RotateField(field, reconciliationAngles);
                    // field(2) = -5;
                    bx = field(0);
                    by = field(1);
                    bz = field(2);

                    mid.set3DField(field);
                    settingUpController = false;
                    procVideoOut.write(grabbedFrame);
                    imshow(rawFrame, grabbedFrame);
                    imshow(processed, processed_frame);
                    imshow(phantom, phantom_mask);
                    waitKey(1);
                    continue;
                }
                bool winCon = baseline_wrt_X < 0.1;
                bool adjustField = baseline_wrt_X > 0.1 && baseline_wrt_X < 0.4;
                bool adjustE = !winCon && !adjustField;
                if (winCon) {
                    finished = true;
                } else if (adjustField) {
                    std::cout << "Adjusting field from\n" << field << "\n";
                    field += (Kp * Kd) / 4 * signFlag * field;
                    std::cout << "To\n" << field << "\n";
                } else {
                    std::cout << "Adjusting Emultiplier from " << EMultiplier
                              << " to ";
                    EAdjust = (Kd)*jointMultiplier * signFlag;
                    EMultiplier += EAdjust;
                    std::cout << EMultiplier << "\n";
                    comp.adjustStiffness(iLinks, EMultiplier);
                    field = comp.CalculateField(iLinks, iJoints, iPosVec);
                    field = comp.RotateField(field, reconciliationAngles);
                }
                // field(2) = -5;
                bx = field(0);
                by = field(1);
                bz = field(2);
                mid.set3DField(field);

                if (finished) {
                    std::cout << "Finished\n";
                    joints_to_solve++;
                    finished = false;
                    got_baseline = false;
                }
            } else if (joints_found > joints_to_solve) {
                joints_to_solve++;
            } else if (joints_to_solve > 6)
                break;
            else {
                mid.retractIntroducer(2);
            }

            // imshow(rawFrame, grabbedFrame);
            // imshow(processed, processed_frame);
            // imshow(phantom, phantom_mask);
            // viz.drawLegend(grabbedFrame);
            procVideoOut.write(grabbedFrame);
            imshow(rawFrame, grabbedFrame);
            imshow(processed, processed_frame);
            imshow(phantom, phantom_mask);
            char c = (char)waitKey(0);
            if (c == 27) {
                break;
            }
        }  // we have established a phantom mask

    }  // while camera is grabbing

    // rawVideoOut.release();
    std::cout << "retracting by " << mid.stepper_count << " steps\n";
    // mid.stepIntroducer(mid.stepper_count);
    camera.StopGrabbing();
    camera.DestroyDevice();
    rawVideoOut.release();
    procVideoOut.release();
    // recordPerformance.close();
    return 0;
}