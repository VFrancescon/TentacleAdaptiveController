#include "HCoilMiddlewareLib/HCoilMiddlewareLib.hpp"
#include "ControllerPrototype.hpp"

double upperError = 10e3;
double lowError = 7e3;
double smallAdjustment = 0.1f;
int main(int argc, char *argv[])
{

    int jointEff = 5;
    int jointNo = jointEff + 1;

    // timesteps are equal to joint no
    int timesteps = jointEff;
    // Vector3d reconciliationAngles = Vector3d{-90, 0, 90};
    double EMulitplier = 5;
    /* * * * * * * * * * * * * * * * * * * * * * * * *
     * PRECOMPUTATION FOR EACH TIMESTEP BEGINS HERE  *
     *                                               *
     *                                               *
     * * * * * * * * * * * * * * * * * * * * * * * * */
    std::vector<Vector3d> AppliedFields;

    std::vector<int> DesiredAngles(jointNo);
    DesiredAngles[0] = 10;
    DesiredAngles[1] = 15;
    DesiredAngles[2] = 15;
    DesiredAngles[3] = 20;
    DesiredAngles[4] = 20;
    DesiredAngles[jointEff] = 0;

    std::vector<Vector3d> Magnetisations(jointNo);
    Magnetisations[0] = Vector3d(-0.0011, 0, -0.0028);
    Magnetisations[1] = Vector3d(-0.0028, 0, 0.001);
    Magnetisations[2] = Vector3d(0, 0, -0.003);
    Magnetisations[3] = Vector3d(-0.003, 0, 0);
    Magnetisations[4] = Vector3d(0, 0, -0.003);
    Magnetisations[jointEff] = Vector3d(0, 0, 0);

    std::vector<PosOrientation> iPosVec(jointNo);
    std::vector<Joint> iJoints(jointNo);
    for (int i = 0; i < jointNo; i++)
    {
        iJoints[i].assignPosOri(iPosVec[i]);
    }

    for (int i = 0; i < jointNo; i++)
    {
        iJoints[i].q = Vector3d(0, DesiredAngles[i], 0);
        iJoints[i].LocMag = Magnetisations[i];
    }

    // create vector of links for properties
    std::vector<Link> iLinks(jointEff);
    adjustStiffness(iLinks, EMulitplier);

    /**************************************************************
     *
     *
     * CalculateField for one configuration only
     *
     *
     *****************************************************************/
    // Vector3d field = RotateField(solution, reconciliationAngles);
    // Vector3d field = CalculateField(iLinks, iJoints, iPosVec);
    // field(1) = 0;
    // std::cout << "Initial answer: " << field << "\n";

    /**************************************************************
     *
     *
     * CalculateField for all configurations
     *
     *
     *****************************************************************/
    
    // Vector3d field = RotateField(solution, reconciliationAngles);
    std::vector<Vector3d> allFields;
    for(int i = 2; i < jointNo+1; i++){
        std::vector<Link> iLinks_ = std::vector<Link>(iLinks.end()-i+1, iLinks.end());
        std::vector<Joint> iJoints_ = std::vector<Joint>(iJoints.end()-i, iJoints.end());
        std::vector<PosOrientation> iPosVec_ = std::vector<PosOrientation>(iPosVec.end()-i, iPosVec.end());
        Vector3d field = CalculateField(iLinks_, iJoints_, iPosVec_);
        field(1) = 0;
        allFields.push_back(field);
    }
    int l = 1;
    for(auto field: allFields){
        std::cout << "Answer at timestep " << l++ << ":\n" << field << "\n";
    }

    /**************************************************************
     *
     *
     * Middleware Setup
     *
     *
     *****************************************************************/
    MiddlewareLayer mid(false);

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

    /**************************************************************
     *
     *
     * Video Output Setup
     *
     *
     *****************************************************************/
    std::string outputPath = "C_PROTOTYPE.avi";

    while (file_exists(outputPath))
    {
        outputPath += "_1";
    }

    VideoWriter video_out(outputPath, VideoWriter::fourcc('M', 'J', 'P', 'G'), 10,
                          Size(rrows, rcols));

    // resize(pre_img, pre_img, Size(rrows, rcols), INTER_LINEAR);
    // Mat pre_img1 = Mat::zeros(Size(rrows, rcols), CV_8UC3);
    // intr_mask = IntroducerMask(pre_img1);
    intr_mask = IntroducerMask(pre_img);
    int jointsCached = 0;
    bool bestSolutionFound = true;
    Point p0 = Point{-2000, 2000};
    double bx_add = 0, bz_add = 0;
    int d_error = 0;
    int error =0, prev_error = 0;
    std::cout << "Ready to go. Press enter";
    std::cin.get();
    Vector3d runtimeField;
    std::vector<Link> iLinks_;
    std::vector<Joint> iJoints_;
    std::vector<PosOrientation> iPosVec_;

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
        resize(post_img, post_img, Size(rrows, rcols), INTER_LINEAR);
        Mat post_img_grey, post_img_th;
        Mat post_img_masked = Mat::zeros(Size(rrows, rcols), CV_8UC1);

        cvtColor(post_img, post_img_grey, COLOR_BGR2GRAY);
        blur(post_img_grey, post_img_grey, Size(5, 5));
        threshold(post_img_grey, post_img_th, threshold_low, threshold_high, THRESH_BINARY_INV);
        // post_img_th.copyTo(post_img_masked, intr_mask);
        post_img_th.copyTo(post_img_masked);

        std::vector<Point> Joints;
        std::vector<std::vector<Point>> contours;

        Joints = findJoints(post_img_masked, contours);
        int JointsObserved = Joints.size();

        if(bestSolutionFound){
            //Push until we get a new joint
            while(jointsCached != JointsObserved){
                mid.stepIntroducer();
                usleep(10e6);
            }
            bestSolutionFound = false;
            runtimeField = allFields[JointsObserved];
            iLinks_ = std::vector<Link>(iLinks.end()-JointsObserved,  iLinks.end());
            iJoints_ = std::vector<Joint>(iJoints.end()-JointsObserved-1, iJoints.end());
            iPosVec_ = std::vector<PosOrientation>(iPosVec.end()-JointsObserved-1, iPosVec.end());
        }
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

        jointsCached = JointsObserved;
        std::vector<double> dAngleSlice = std::vector<double>(desiredAngles_.end() - angles.size(), desiredAngles_.end());
        // std::vector<double> dAngleSlice = desiredAngles_;
        error = meanError(dAngleSlice, angles);
        d_error = abs(error - prev_error);
        prev_error = error;
        int K_derivative = derivativeAdjustment(d_error, error);
        std::cout << "\n\n---------------------------------------------------------\n\n";
        // Controller Logic
        // if e < 0: signFlag = -1
        // else signFlag = 1
        // then e = abs(e)
        // Scenario 1. e < LowS -> Do Nothing
        // Scenario 2. LowS < e < HighS -> Field + P*signFlag
        // Scenario 3. e > HighS -> K += signFlag
        int signFlag = (error < 0) ? -1 : 1;
        std::cout << "Error " << error << "\n";
        error = abs(error);
        if (error < lowError)
        {
            imshow("Post", post_img);
            video_out.write(post_img);
            char c = (char)waitKey(0);
            if (c == 27)
                break;
            else
                continue;
        }
        else if (error > lowError && error < upperError)
        {
            runtimeField += runtimeField * smallAdjustment * signFlag * K_derivative;
            std::cout << "Adjusting field\n";
        }
        else if (error > upperError)
        {
            EMulitplier += (signFlag * K_derivative);
            adjustStiffness(iLinks, EMulitplier);
            runtimeField = CalculateField(iLinks_, iJoints_, iPosVec_);
            runtimeField(1) = 0;
            std::cout << "Adjusting E\n";
        }

        std::cout << "E: " << EMulitplier << " applied field:\n"
                  << runtimeField << "\n";

        if (abs(runtimeField(0)) > 20 && abs(runtimeField(2)) > 15 && abs(runtimeField(1)) > 20)
            break;
        if (EMulitplier < 0)
            break;

        mid.set3DField(runtimeField);

        imshow("Post", post_img);
        video_out.write(post_img);
        char c = (char)waitKey(50e2);
        if (c == 27)
            break;
    }
    video_out.release();
    mid.~MiddlewareLayer();
    // destroyAllWindows();
    // Pylon::PylonTerminate();

    return 0;
}


