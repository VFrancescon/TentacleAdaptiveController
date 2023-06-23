#include "PD_Insertion.hpp"

int rrows;
int rcols;
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
        << "Step, JointNo, Ex(t), Ey(t), BaselineX, E_Multiplier, Bx, By, Bz\n";

    CompClass comp;
    VisionClass viz;
    viz.setThresholdLow(170);
    int jointEff = 5;
    int jointNo = jointEff + 1;
    int jointMultiplier = 1;
    switch (jointMultiplier) {
        case 2:
            viz.setLinkLenght(40);
            break;

        default:
            viz.setLinkLenght(60);
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
        DesiredAngles[2] = 5;
        DesiredAngles[3] = 5;
        DesiredAngles[4] = 5;
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
        
    Mat pre_img;
    try {const uint8_t *preImageBuffer = (uint8_t *)ptrGrabResult->GetBuffer();
    } catch( const Pylon::RuntimeException e){
        std::cout << e.what() << "\n";
    }
    formatConverter.Convert(pylonImage, ptrGrabResult);
    pre_img = cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(),
                      CV_8UC3, (uint8_t *)pylonImage.GetBuffer());

    // resizing the image for faster processing
    rrows = pre_img.rows * 3 / 8;
    rcols = pre_img.cols * 3 / 8;
    
    /*****************************************************************
     * Video Output Setup
     *****************************************************************/
    std::string angleSTR;
    for (auto i : DesiredAngles) {
        angleSTR += std::to_string(i) + "_";
    }

    std::string outputPath = "PD_BORDERLESS_" + angleSTR + "_" + date + ".avi";

    while (file_exists(outputPath)) {
        outputPath += "_1";
    }

    VideoWriter video_out(outputPath, VideoWriter::fourcc('M', 'J', 'P', 'G'),
                          10, Size(rcols, rrows));
    resize(pre_img, pre_img, Size(rcols, rrows), INTER_LINEAR);
    Mat phantom_mask = viz.isolatePhantom(pre_img);
    // intr_mask = viz.IntroducerMask(pre_img);

    int error = 0, prev_xerror = 0, prev_yerror = 0;
    int dx_error = 0, dy_error = 0;
    int step_count = 0;
    Point p0 = Point{rcols / 2, 0};
    bool firstRun = true;
    bool finished = true;
    int baseline_error;
    int signFlag;

    std::cout << "Ready to go. Press enter";
    std::cin.get();
    // mid.retractIntroducer(10);

    auto start = std::chrono::high_resolution_clock::now();
    auto end = std::chrono::high_resolution_clock::now();
    bool controllerActive = true;
    int jointsFound = 0;
    int jointToSolve = 1;

    // while (camera.IsGrabbing()) {
    while (true) {
        // if 2s have expired
        // if (std::chrono::duration_cast<std::chrono::milliseconds>(end -
        // start)
        //         .count() > 1000) {
        //     start = std::chrono::high_resolution_clock::now();
        //     controllerActive = !controllerActive;
        // }
        Vector3d field;
        camera.RetrieveResult(5000, ptrGrabResult,
                              Pylon::TimeoutHandling_ThrowException);
        const uint8_t *pImageBuffer = (uint8_t *)ptrGrabResult->GetBuffer();
        Pylon::CPylonImage pylonImage;
        formatConverter.Convert(pylonImage, ptrGrabResult);
        Mat post_img =
            cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(),
                    CV_8UC3, (uint8_t *)pylonImage.GetBuffer());
        Mat display_img = post_img;
        resize(display_img, display_img, Size(rcols, rrows), INTER_LINEAR);
        post_img = viz.preprocessImg(post_img, rrows, rcols);
        viz.drawLegend(display_img);
        std::vector<Point> Joints = viz.findJoints(post_img);
        jointsFound = Joints.size(); //FIND OUT HOW MANY JOINTS
        if(jointsFound) p0 = Joints.at(0);

        while (jointToSolve == jointsFound) {
            std::vector<double> DesiredAnglesSPLIT_slice =
                std::vector<double>(DesiredAnglesSPLIT.begin(),
                                    DesiredAnglesSPLIT.begin() + jointsFound);
            std::vector<Vector3d> MagnetisationsSPLIT_slice =
                std::vector<Vector3d>(
                    MagnetisationsSPLIT.end() - jointsFound - 1,
                    MagnetisationsSPLIT.end());

            // DesiredAnglesSPLIT_slice.push_back(0);

            std::vector<PosOrientation> iPosVec(jointsFound + 1);
            std::vector<Joint> iJoints(jointsFound + 1);
            for (int i = 0; i < iPosVec.size(); i++) {
                iJoints[i].assignPosOri(iPosVec[i]);
            }

            for (int i = 0; i < iJoints.size(); i++) {
                iJoints[i].q =
                    Vector3d(0, DesiredAnglesSPLIT_slice[i] * M_PI / 180, 0);
                iJoints[i].LocMag = MagnetisationsSPLIT_slice[i];
            }
            std::vector<Link> iLinks(jointsFound);
            // TODO: Look at this line. p0 will need to calculated somewhat
            std::vector<Point> idealPoints =
                viz.computeIdealPoints(p0, DesiredAnglesSPLIT_slice);
            std::vector<double> desiredX, observedX, desiredY, observedY;
            for (auto i : idealPoints) {
                desiredX.push_back(i.x);
                desiredY.push_back(i.y);
            }
            for (auto i : Joints) {
                observedX.push_back(i.x);
                observedY.push_back(i.y);
            }
            visualizePoints(display_img, idealPoints, Joints);

            double xError = xwiseError(desiredX, observedX);
            double yError = ywiseError(desiredY, observedY);
            dx_error = xError - prev_xerror;
            dy_error = yError - prev_yerror;
            prev_xerror = xError;
            prev_yerror = yError;
            finished = true;

            // controller happens here
            if (firstRun) {
                firstRun = false;
                comp.adjustStiffness(iLinks, EMultiplier, jointMultiplier);
                field = comp.CalculateField(iLinks, iJoints, iPosVec);
                float bx = field[0];
                float by = field[1];
                float bz = field[2];
                baseline_error = (abs(xError) + yError) / 2;
                // 3. initial field
                // 4. baseline error readings
            }  // if first run of the controller
            // if (controllerActive) {
                controllerActive = false;
                double baselineX =
                    ((abs(xError) + yError) / 2) / baseline_error;
                int xFlag = std::signbit(xError) ? 1 : -1;
                int yFlag = std::signbit(yError) ? -1 : 1;
                int signFlag;

                if (xFlag == -1 && yFlag == 1) {
                    signFlag = -1;
                } else
                    signFlag = xFlag;

                double Kp = 0.5;
                double Kd = derivativeAdjustmentF(dx_error);

                if (baselineX < 0.1) {
                    finished = true;
                    continue;
                } else if (baselineX > 0.1 && baselineX < 0.4) {
                    std::cout << "Adjusting field from\n" << field << "\n";
                    field += (Kp * Kd) * signFlag * rightHandBend * field;
                    std::cout << "To\n" << field << "\n";
                } else {
                    std::cout << "Adjusting Emultiplier from " << EMultiplier
                              << " to ";
                    EMultiplier +=
                        (Kd)*jointMultiplier * signFlag * rightHandBend;
                    std::cout << EMultiplier << "\n";
                    comp.adjustStiffness(iLinks, EMultiplier);
                    field = comp.CalculateField(iLinks, iJoints, iPosVec) *
                            rightHandBend;
                    field = comp.RotateField(field, reconciliationAngles);
                }
                if (abs(field(0)) > 20 && abs(field(2)) > 15 &&
                    abs(field(1)) > 20)
                    break;
                if (EMultiplier < 0) {
                    EMultiplier = 0;
                    continue;
                }

                mid.set3DField(field);

                if (finished) {
                    firstRun = true;
                    jointToSolve++;
                }
                cv::imshow("Post", display_img);
                video_out.write(display_img);
                char c = (char)waitKey(1000);
                if (c == 27) break;
            // }  // if controller is active

        }
        if (jointsFound < jointToSolve){
            mid.retractIntroducer(10);
        } else jointToSolve = jointsFound;
        cv::imshow("Post", display_img);
        cv::imshow("Masked img", post_img);
        cv::imshow("Mask", viz.mask);
        video_out.write(display_img);
        char c = (char)waitKey(0);
        if (c == 27) break;
        mid.retractIntroducer(10);


    }  // while camera is grabbing

    video_out.release();
    mid.stepIntroducer(mid.stepper_count);
    camera.StopGrabbing();
    camera.DestroyDevice();
    recordPerformance.close();
    return 0;
}