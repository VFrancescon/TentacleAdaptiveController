#include "extraCameraFuncs.hpp"

// std::vector<Point> findJoints(Mat post_img_masked, std::vector<std::vector<Point>> &contours, std::vector<Point> &cntLine)
// {

//     Mat contours_bin;
//     // std::vector<std::vector<Point> > contours;
//     std::vector<Vec4i> hierarchy;
//     // find contours, ignore hierarchy
//     findContours(post_img_masked, contours, hierarchy, RETR_LIST, CHAIN_APPROX_SIMPLE);
//     contours_bin = Mat::zeros(post_img_masked.size(), CV_8UC1);

//     // draw contours and fill the open area
//     drawContours(contours_bin, contours, -1, Scalar(255, 255, 255), cv::FILLED, LINE_8, hierarchy);
//     // empty matrix. Set up to 8-bit 1 channel data. Very important to set up properly.
//     Mat skeleton = Mat::zeros(post_img_masked.rows, post_img_masked.rows, CV_8U);

//     // take the filled contour and thin it using Zhang Suen method. Only works with 8-bit 1 channel data.
//     ximgproc::thinning(contours_bin, skeleton, 0);

//     contours.clear();
//     hierarchy.clear();
//     findContours(skeleton, contours, hierarchy, RETR_LIST, CHAIN_APPROX_SIMPLE);

//     findNonZero(skeleton, cntLine);
//     std::sort(cntLine.begin(), cntLine.end(), yWiseSort);
//     // std::reverse(cntLine.begin(), cntLine.end());

//     std::vector<Point> Joints;
//     int jointCount = (int)cntLine.size() / link_lenght;
//     std::cout << "Size of centre-line " << cntLine.size() << "\n";

//     if (jointCount)
//     {
//         for (int i = 0; i < jointCount; i++)
//         {
//             Joints.push_back(cntLine[link_lenght * (i)]);
//         }
//     }
//     std::reverse(Joints.begin(), Joints.end());
//     std::cout << "Number of joints " << Joints.size() << "\n";

//     return Joints;
// }

int main(int argc, char* argv[]){
    
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
    
    Pylon::CFloatParameter(camera.GetNodeMap(), "ExposureTime").SetValue(exposureTime);
    
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
        std::vector<Point> cntLine;
        std::vector<std::vector<Point>> contours;

        Joints = findJoints(post_img_masked, contours);
        // std::cout << "--------------------Points--------------------\n";
        // for(auto i: cntLine){
        //     std::cout << "point: " << i << "\n";
        // }
        // // std::reverse(cntLine.begin(), cntLine.end());
        // std::vector<Point>::iterator cntLineIterator = cntLine.begin();
        
        // float floatJoints = std::ceil( (float) (cntLine.size() / (float) link_lenght));
        // std::cout << "unrounded: " << cntLine.size() << " " << link_lenght << " " <<
        // ((float) cntLine.size() / (float) link_lenght) << 
        //     " rounded: " << floatJoints << "\n";
        
        // for(int i = 0; i < (int) floatJoints; i++){
        //     circle(post_img, *cntLineIterator, 4, Scalar(255,0,0), FILLED);
        //     std::advance(cntLineIterator, link_lenght);
        // }

        // std::cout << "--------------------Points--------------------\n";
        int l = 0;
        for (auto i : Joints)
        {
            circle(post_img, i, 4, Scalar(255, 0, 0), FILLED);
            putText(post_img, std::to_string(l++), i, FONT_HERSHEY_COMPLEX, 1.0, Scalar(255,255,0));
        }
        // drawContours(post_img, contours, -1, Scalar(255, 255, 0));
        imshow("Post", post_img);
        char c = (char)waitKey(10);
        if (c == 27) break;
    }
    
    
    return 0;
}