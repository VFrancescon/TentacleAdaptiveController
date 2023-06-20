#include "PD_Insertion.hpp"

int rrows;
int rcols;
int main(int argc, char* argv[]){

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
    
    int jointEff = 5;
    int jointNo = jointEff + 1;
    int jointMultiplier = 1;

    // timesteps are equal to joint no
    int timesteps = jointEff;
    Vector3d reconciliationAngles = Vector3d{0, 90, 180};
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
        DesiredAngles[0] = 15;
        DesiredAngles[1] = 15;
        DesiredAngles[2] = 20;
        DesiredAngles[3] = 20;
        DesiredAngles[4] = 25;
        DesiredAngles[jointEff] = 0;
    }
    if( argc == 2 || argc == 7) {
        jointMultiplier = std::stoi(argv[argc-1]);
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

    std::vector<PosOrientation> iPosVec(jointEff * jointMultiplier + 1);
    std::vector<Joint> iJoints(jointEff * jointMultiplier + 1);
    for (int i = 0; i < iPosVec.size(); i++) {
        iJoints[i].assignPosOri(iPosVec[i]);
    }

    for (int i = 0; i < iJoints.size(); i++) {
        iJoints[i].q = Vector3d(0, DesiredAnglesSPLIT[i] * M_PI / 180, 0);
        iJoints[i].LocMag = MagnetisationsSPLIT[i];
    }
    // create vector of links for properties
    std::vector<Link> iLinks(jointEff * jointMultiplier);
    comp.adjustStiffness(iLinks, EMultiplier, jointMultiplier);
    Vector3d field = comp.CalculateField(iLinks, iJoints, iPosVec);
    field = comp.RotateField(field, reconciliationAngles);
    field(1) = 0;
    double bx = field(0);
    double by = field(1);
    double bz = field(2);
    std::cout << "Initial answer:\n" << field << "\n";
    int mid_OPMODE = 1;
    MiddlewareLayer mid(mid_OPMODE);

    //5. boot up Pylon backend
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
    width.TrySetValue(viz.getPylonWidth(), Pylon::IntegerValueCorrection_Nearest);
    height.TrySetValue(viz.getPylonHeight(), Pylon::IntegerValueCorrection_Nearest);
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
    rrows = pre_img.rows * 3 / 8;
    rcols = pre_img.cols * 3 / 8;

    //6. setup Controller variables
    //7. setup controller classes
    //8. start loop

    return 0;   
}