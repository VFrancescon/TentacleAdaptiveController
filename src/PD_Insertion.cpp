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
    std::ofstream recordPerformance;
    recordPerformance.open("../PD_BORDERLESS_Results.csv", std::ios_base::app);
    recordPerformance << date << "\n";
    recordPerformance << "Frame, Joints Active, Error(x), "
                      << "Baseline Error, E_Multiplier, Bx, By, Bz";
    int frameCount = 0;
    CompClass comp;
    VisionClass viz;
    viz.setThresholdLow(80);
    viz.setlenAdj(1.25);
    viz.setHsvLow(0, 255, 162);
    int jointEff = 5;
    int jointNo = jointEff + 1;
    int jointMultiplier = 1;
    switch (jointMultiplier) {
        case 2:
            viz.setLinkLenght(15);
            break;

        default:
            viz.setLinkLenght(25);
            break;
    }
    // timesteps are equal to joint no
    int timesteps = jointEff;
    Vector3d reconciliationAngles = Vector3d{90, 0, 180};
    double EMultiplier = 10;
    /* * * * * * * * * * * * * * * * * * * * * * * * *
     * PRECOMPUTATION FOR EACH TIMESTEP BEGINS HERE  *
     * * * * * * * * * * * * * * * * * * * * * * * * */
    std::vector<Vector3d> AppliedFields;

    std::vector<std::vector<double>> AllConfigurations;
    std::vector<double> DesiredAngles(jointNo);
    if (argc > 5) {
        DesiredAngles[0] = std::stod(argv[1]);
        DesiredAngles[1] = std::stod(argv[2]);
        DesiredAngles[2] = std::stod(argv[3]);
        DesiredAngles[3] = std::stod(argv[4]);
        DesiredAngles[4] = std::stod(argv[5]);
        DesiredAngles[jointEff] = 0;
    } else {
        DesiredAngles[0] = -15;
        DesiredAngles[1] = -10;
        DesiredAngles[2] = -10;
        DesiredAngles[3] = -5;
        DesiredAngles[4] = -5;
        DesiredAngles[jointEff] = 0;
    }
    AllConfigurations.push_back(DesiredAngles);
    AllConfigurations.push_back(std::vector<double>{0, 5, 5, 10, 5, 0});
    AllConfigurations.push_back(std::vector<double>{40, 10, 5, 0, 0, 0});
    AllConfigurations.push_back(std::vector<double>{-40, 5, 0, 0, 0, 0});
    AllConfigurations.push_back(std::vector<double>{-20, 15, 0, 0, 0, 0});
    int rightHandBend = 0;
    if (argc == 2 || argc == 7) {
        jointMultiplier = std::stoi(argv[argc - 1]);
    }

    std::vector<Vector3d> Magnetisations(jointNo);
    Magnetisations[0] = Vector3d(-0.0011, 0, -0.0028);
    Magnetisations[1] = Vector3d(-0.0028, 0, -0.001);
    Magnetisations[2] = Vector3d(0, 0, -0.003);
    Magnetisations[3] = Vector3d(-0.003, 0, 0);
    Magnetisations[4] = Vector3d(0, 0, -0.003);
    Magnetisations[jointEff] = Vector3d(0, 0, 0);

    // // split angles and magnetisations
    // std::vector<double> DesiredAnglesSPLIT(jointEff * jointMultiplier);
    // std::vector<Vector3d> MagnetisationsSPLIT(jointEff * jointMultiplier);
    // if (jointMultiplier > 1) {
    //     // convert sub5 joint numbers
    //     for (int i = 0; i < jointEff * jointMultiplier; i++) {
    //         DesiredAnglesSPLIT[i] =
    //             DesiredAngles[i / jointMultiplier] / jointMultiplier;
    //         MagnetisationsSPLIT[i] = Magnetisations[i / jointMultiplier];
    //     }
    //     DesiredAnglesSPLIT.push_back(0);
    //     MagnetisationsSPLIT.push_back(Vector3d(0, 0, 0));
    // } else {
    //     std::cout << "Using defaults\n";
    //     DesiredAnglesSPLIT = DesiredAngles;
    //     MagnetisationsSPLIT = Magnetisations;
    // }

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

    std::string outputPath = "PD_INSERTION_" + date + ".avi";
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
    Point p0 = Point{0, 0};
    bool finished = true;
    int baseline_error;
    int signFlag;
    double small_scale = 1;
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
    bool extrapush = false;
    bool winCon;
    bool adjustField;
    bool adjustE;
    bool moving = false;
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
    int active_index = 3;
    const int max_index = 4;
    std::vector<int> retractions = {45, 0, 0, 25, 0};
    int to_retract = 0;
    std::vector<double> rectangles = {0.15, 0.15, 0.6, 0.69, 0.69};
    std::vector<double> ActiveConfiguration;
    while (camera.IsGrabbing()) {
        // 1. get a phantom mask
        camera.RetrieveResult(5000, ptrGrabResult,
                              Pylon::TimeoutHandling_ThrowException);
        const uint8_t *pImageBuffer = (uint8_t *)ptrGrabResult->GetBuffer();
        formatConverter.Convert(pylonImage, ptrGrabResult);

        // check for which bend you have here.
        // every time we successfully solve 5 joints, we advance the list of
        // stuff to solve and reset initialSetup, which will cause us to get a
        // new phantom mask

        if (initialSetup) {
            if (active_index > max_index) {
                std::cout << "Solved all configurations. Exiting.\n";
                break;
            }
            pre_img =
                cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(),
                        CV_8UC3, (uint8_t *)pylonImage.GetBuffer());
            if (pre_img.empty()) {
                break;
            }
            resize(pre_img, pre_img, Size(rcols, rrows), INTER_LINEAR);
            viz.setRectW(rectangles.at(active_index));
            phantom_mask = viz.isolatePhantom(pre_img);
            std::cout << "Phantom mask found. Press enter to continue.\n";
            imshow(phantom, phantom_mask);
            imshow(rawFrame, pre_img);
            char key = (char)waitKey(1000);
            if (key == 27) {
                break;
            }
            // 2. push 1 joint in.
            mid.retractIntroducer(2);
            first_run = true;
            initialSetup = false;
            settingUpController = true;
            got_baseline = false;
            joints_to_solve = 2;
            ActiveConfiguration.clear();
            ActiveConfiguration = AllConfigurations.at(active_index);
            to_retract = retractions.at(active_index);
            step_count = to_retract;
            // active_index++;
            rightHandBend = avgVect(ActiveConfiguration) < 0 ? 1 : -1;

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
            if (joints_found > 4)
                viz.setlenAdj(1.1);
            else
                viz.setlenAdj(1.3);
            if(active_index == 4 ) viz.setLinkLenght(23);
            solving_time = joints_found == joints_to_solve;
            if (joints_found != 0) {
                if (first_run) p0 = Joints.at(0);
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
                ActiveConfiguration.begin(),
                ActiveConfiguration.begin() + joints_to_solve);

            std::vector<Point> dPoints = viz.computeIdealPoints(p0, dAnglesS);
            if (!moving && joints_found > 1) {
                for (int i = 0; i < dPoints.size() - 1; i++) {
                    // std::cout << " " << i;
                    line(grabbedFrame, dPoints[i], dPoints[i + 1],
                         Scalar(0, 0, 255), 2);
                    circle(grabbedFrame, dPoints[i], 3, Scalar(0, 0, 255),
                           FILLED);
                    // putText(post_img, std::to_string(i), dPoints[i],
                    //         FONT_HERSHEY_SIMPLEX, 1.0, Scalar(255, 0, 0));
                }
                circle(grabbedFrame, dPoints[dPoints.size() - 1], 3,
                       Scalar(0, 255, 0), FILLED);
            }

            if (moving) {
                mid.set3DField(0, 0, 0);
                if (step_count != 0) {
                    if (step_count > 0)
                        mid.stepIntroducer();
                    else
                        mid.retractIntroducer();
                    step_count--;
                    procVideoOut.write(grabbedFrame);
                    imshow(rawFrame, grabbedFrame);
                    imshow(processed, processed_frame);
                    imshow(phantom, phantom_mask);
                    waitKey(100);
                } else {
                    initialSetup = true;
                    moving = false;
                    active_index++;
                    continue;
                }
            } else if (solving_time && !extrapush) {  // run the controller here
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
                double baseline_wrt_X = (xError) / baseline_error;
                int xFlag = std::signbit(baseline_error - xError) ? -1 : 1;
                int yFlag = std::signbit(baseline_error - yError) ? -1 : 1;

                double Kp = 0.4;
                double Kd = derivativeAdjustmentF(dx_error);

                std::vector<Vector3d> MagS = std::vector<Vector3d>(
                    Magnetisations.end() - joints_to_solve - 1,
                    Magnetisations.end());

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
                    field(2) = 0;
                    bx = field(0);
                    by = field(1);
                    bz = field(2);

                    mid.set3DField(field);
                    settingUpController = false;
                    recordPerformance << frameCount++ << "," << joints_to_solve
                                      << "," << xError << "," << baseline_error
                                      << "," << EMultiplier << "," << bx << ","
                                      << by << "," << bz << "\n";
                    procVideoOut.write(grabbedFrame);
                    imshow(rawFrame, grabbedFrame);
                    imshow(processed, processed_frame);
                    imshow(phantom, phantom_mask);
                    waitKey(1);
                    continue;
                }
                if (baseline_error <= 4) small_scale = 2.75;
                winCon = baseline_wrt_X < 0.25 * small_scale;
                adjustField = baseline_wrt_X > 0.25 * small_scale &&
                              baseline_wrt_X < 0.4 * small_scale;
                adjustE = !winCon && !adjustField;
                if (baseline_error <= 1) winCon = true;
                if (winCon) {
                    finished = true;
                } else if (adjustField) {
                    std::cout << "Adjusting field from\n" << field << "\n";
                    // field += ((Kp * Kd) / 4 * xFlag  * rightHandBend) * field
                    // ;
                    field = field * (1 + (Kp * Kd)* 0.5 * xFlag * rightHandBend);
                } else {
                    std::cout << "Adjusting Emultiplier from " << EMultiplier
                              << " to ";
                    EAdjust = (Kd)*jointMultiplier * xFlag * rightHandBend;
                    EMultiplier += EAdjust;
                    std::cout << EMultiplier << "\n";
                    comp.adjustStiffness(iLinks, EMultiplier);
                    field = comp.CalculateField(iLinks, iJoints, iPosVec);
                    field = comp.RotateField(field, reconciliationAngles);
                }
                field(2) = 0;
                std::cout << "Field\n" << field << "\n";
                bx = field(0);
                by = field(1);
                bz = field(2);
                // if(abs(bx) > 20){
                //     std::cout << "Hold on\n";
                // }
                mid.set3DField(field);
                if (finished) {
                    std::cout << "Finished\n";
                    joints_to_solve++;
                    if (joints_to_solve >= 5 && joints_found >= 5) moving = true;
                    finished = false;
                    got_baseline = false;
                }
                // usleep(5e6);
            } else if (joints_found > joints_to_solve) {
                mid.stepIntroducer();
            } else {
                mid.retractIntroducer(2);
                // if(joints_found > 3) mid.retractIntroducer(10);
            }

            // imshow(rawFrame, grabbedFrame);
            // imshow(processed, processed_frame);
            // imshow(phantom, phantom_mask);
            // viz.drawLegend(grabbedFrame);
            recordPerformance << frameCount++ << "," << joints_to_solve << ","
                              << xError << "," << baseline_error << ","
                              << EMultiplier << "," << bx << "," << by << ","
                              << bz << "\n";
            procVideoOut.write(grabbedFrame);
            imshow(rawFrame, grabbedFrame);
            imshow(processed, processed_frame);
            imshow(phantom, phantom_mask);
            extrapush = false;
            char c = (char)waitKey(500);
            if (c == 'n') {
                mid.retractIntroducer(2);
                extrapush = true;
            } else if (c == 27) {
                break;
            }

        }  // we have established a phantom mask

    }  // while camera is grabbing

    // rawVideoOut.release();
    // std::cout << "retracting by " << mid.stepper_count << " steps\n";
    // mid.stepIntroducer(mid.stepper_count);
    mid.unwindIntroducer();
    camera.StopGrabbing();
    camera.DestroyDevice();
    rawVideoOut.release();
    procVideoOut.release();
    // recordPerformance.close();
    return 0;
}