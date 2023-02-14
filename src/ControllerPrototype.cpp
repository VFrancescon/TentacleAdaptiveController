#include "HCoilMiddlewareLib/HCoilMiddlewareLib.hpp"
#include "ControllerPrototype.hpp"

double upperError = 15;
double lowError = 11;

int main(int argc, char *argv[])
{
    /**
     * Get today's date
    */
    time_t curr_time;
    tm * curr_tm;
    time(&curr_time);
    curr_tm = localtime(&curr_time);
    char date_string[100];
    strftime(date_string, 50, "%d_%m_%y_%H%M%S", curr_tm);
    std::string date(date_string);
    std::ofstream recordPerformance;
    recordPerformance.open("../P_Results.csv", std::ios_base::app);
    recordPerformance << date << "\n";
    recordPerformance << "Step, Error(t), Error(t-1), E_Multiplier, Bx, By, Bz\n";
    
    int jointEff = 5;
    int jointNo = jointEff + 1;

    // timesteps are equal to joint no
    int timesteps = jointEff;
    Vector3d reconciliationAngles = Vector3d{0, 0, 180};
    double EMulitplier = 5;
    /* * * * * * * * * * * * * * * * * * * * * * * * *
     * PRECOMPUTATION FOR EACH TIMESTEP BEGINS HERE  *
     *                                               *
     *                                               *
     * * * * * * * * * * * * * * * * * * * * * * * * */
    std::vector<Vector3d> AppliedFields;

    std::vector<int> DesiredAngles(jointNo);
    DesiredAngles[0] = 10;
    DesiredAngles[1] = 20;
    DesiredAngles[2] = 20;
    DesiredAngles[3] = 15;
    DesiredAngles[4] = 45;
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

    Vector3d field = CalculateField(iLinks, iJoints, iPosVec);
    field = RotateField(field, reconciliationAngles);
    field(1) = 0;
    std::cout << "Initial answer:\n" << field << "\n";

    /**************************************************************
     *
     *
     * Middleware Setup
     *
     *
     *****************************************************************/
    MiddlewareLayer mid(true);
    mid.set3DField(field);

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
    Pylon::CInstantCamera camera(Pylon::CTlFactory::GetInstance().CreateFirstDevice() );
    camera.Open();
    Pylon::CIntegerParameter width     ( camera.GetNodeMap(), "Width");
    Pylon::CIntegerParameter height    ( camera.GetNodeMap(), "Height");
    Pylon::CEnumParameter pixelFormat  ( camera.GetNodeMap(), "PixelFormat");
    
    Pylon::CFloatParameter(camera.GetNodeMap(), "ExposureTime").SetValue(20000.0);
    
    
    
    Size frameSize= Size((int)width.GetValue(), (int)height.GetValue());
    int codec = VideoWriter::fourcc('M', 'J', 'P', 'G');
    width.TrySetValue(PYLON_WIDTH, Pylon::IntegerValueCorrection_Nearest);
    height.TrySetValue(PYLON_HEIGHT, Pylon::IntegerValueCorrection_Nearest);
    Pylon::CPixelTypeMapper pixelTypeMapper( &pixelFormat);
    Pylon::EPixelType pixelType = pixelTypeMapper.GetPylonPixelTypeFromNodeValue(pixelFormat.GetIntValue());
    camera.StartGrabbing(Pylon::GrabStrategy_LatestImageOnly);
    Pylon::CGrabResultPtr ptrGrabResult;
    camera.RetrieveResult(5000, ptrGrabResult, Pylon::TimeoutHandling_ThrowException);
    const uint8_t* preImageBuffer = (uint8_t*) ptrGrabResult->GetBuffer();
    formatConverter.Convert(pylonImage, ptrGrabResult);
    pre_img = cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC3, (uint8_t *) pylonImage.GetBuffer());

    // VideoWriter video_out(home_path + "coil_manipulator/output.avi", VideoWriter::fourcc('M', 'J', 'P', 'G'), 10, 
    //             Size(img.rows / 2, img.cols * 3 / 8));

    //resizing the image for faster processing
    int rrows = pre_img.rows * 3 / 8;
    int rcols = pre_img.cols * 3 / 8; 

    /**************************************************************
     *
     *
     * Video Output Setup
     *
     *
     *****************************************************************/
    std::string outputPath = "C_PROTOTYPE_" + date + ".avi";

    while (file_exists(outputPath))
    {
        outputPath += "_1";
    }

    VideoWriter video_out(outputPath, VideoWriter::fourcc('M', 'J', 'P', 'G'), 10,
                          Size(rcols, rrows));

    // resize(pre_img, pre_img, Size(rcols, rrows), INTER_LINEAR);
    // Mat pre_img1 = Mat::zeros(Size(rcols, rrows), CV_8UC3);
    // intr_mask = IntroducerMask(pre_img1);
    intr_mask = IntroducerMask(pre_img);
    int jointsCached = 0;
    int step_count = 0;
    int error = 0, prev_error = 0;
    int d_error = 0;
    Point p0 = Point{-2000, 2000};
    double bx_add = 0, bz_add = 0;

    recordPerformance << step_count << "," << error << "," <<
        d_error << "," << EMulitplier << "," << field(0) << "," <<
        field(1) << "," << field(2) << "\n";


    std::cout << "Ready to go. Press enter";
    std::cin.get();

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
        std::vector<double> desiredAngles_ = std::vector<double>(DesiredAngles.begin(), DesiredAngles.end() - 1);
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
            std::cout << "Victory!\n";            
            video_out.write(post_img);
            step_count++;
            recordPerformance << step_count << "," << error << "," <<
                d_error << "," << EMulitplier << "," << field(0) << "," <<
                field(1) << "," << field(2) << "\n";

            std::cout << "Final set of joint angles:\n";
            std::vector<double> finalAngles = computeAngles(Joints);
            for(auto i: finalAngles){
                std::cout << i << "\n";
            }
            std::cout << "End of operations\n";

            imshow("Post", post_img);
            char c = (char)waitKey(0);
            if (c == 27)
                break;
            else{
                
                continue;
                }
        }
        else if (error > lowError && error < upperError)
        {
            field += field * 0.1 * signFlag;
            std::cout << "Adjusting field\n";
        }
        else if (error > upperError)
        {
            EMulitplier += signFlag;
            adjustStiffness(iLinks, EMulitplier);
            field = CalculateField(iLinks, iJoints, iPosVec);
            field = RotateField(field, reconciliationAngles);
            field(1) = 0;
            std::cout << "Adjusting E\n";
        }

        std::cout << "E: " << EMulitplier << " applied field:\n"
                  << field << "\n";

        if (abs(field(0)) > 20 && abs(field(2)) > 15 && abs(field(1)) > 20)
            break;
        if (EMulitplier < 0)
            break;

        mid.set3DField(field);
        step_count++;
        recordPerformance << step_count << "," << error << "," <<
            d_error << "," << EMulitplier << "," << field(0) << "," <<
            field(1) << "," << field(2) << "\n";

        imshow("Post", post_img);
        video_out.write(post_img);
        char c = (char)waitKey(10e2);
        if (c == 27)
            break;
    }
    video_out.release();
    mid.~MiddlewareLayer();
    recordPerformance.close();
    // destroyAllWindows();
    // Pylon::PylonTerminate();
    return 0;

}
