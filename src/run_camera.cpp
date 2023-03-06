#include "extraCameraFuncs.hpp"

using namespace cv;
int main(int argc, char *argv[])
{
    Mat img;
    Pylon::PylonInitialize();
    Pylon::CImageFormatConverter formatConverter;
    formatConverter.OutputPixelFormat = Pylon::PixelType_BGR8packed;
    Pylon::CPylonImage pylonImage;
    Pylon::CInstantCamera camera(Pylon::CTlFactory::GetInstance().CreateFirstDevice());
    camera.Open();
    Pylon::CIntegerParameter width(camera.GetNodeMap(), "Width");
    Pylon::CIntegerParameter height(camera.GetNodeMap(), "Height");
    Pylon::CEnumParameter pixelFormat(camera.GetNodeMap(), "PixelFormat");

    Pylon::CFloatParameter(camera.GetNodeMap(), "ExposureTime").SetValue(exposureTime);

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
    img = cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC3, (uint8_t *)pylonImage.GetBuffer());

    // VideoWriter video_out(home_path + "coil_manipulator/output.avi", VideoWriter::fourcc('M', 'J', 'P', 'G'), 10,
    //             Size(img.rows / 2, img.cols * 3 / 8));

    // resizing the image for faster processing
    int rows = img.rows * 3 / 8;
    int cols = img.cols * 3 / 8;
    resize(img, img, Size(cols, rows), INTER_LINEAR);

    while (camera.IsGrabbing())
    {
        camera.RetrieveResult(5000, ptrGrabResult, Pylon::TimeoutHandling_ThrowException);
        const uint8_t *pImageBuffer = (uint8_t *)ptrGrabResult->GetBuffer();
        formatConverter.Convert(pylonImage, ptrGrabResult);
        img = cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC3, (uint8_t *)pylonImage.GetBuffer());

        // cap >> img;
        if (img.empty())
        {
            break;
        }

        // make image smaller
        resize(img, img, Size(cols, rows), INTER_LINEAR);
        // flip(img, img, 1);
        // line( img, Point(0, cols/3), Point(cols, rows/3), Scalar(0,0,0) ); //horizontal lines
        // line( img, Point(0, cols*2/3), Point(cols, rows/3*2), Scalar(0,0,0) );

        // line( img, Point(rows/3, 0), Point(rows/3, cols), Scalar(0,0,0) ); //horizontal lines
        // line( img, Point(rows*2/3), Point(rows*2/3, cols), Scalar(0,0,0) );

        // /**
        //  * @brief DRAW A LEGEND
        //  */
        // cv::Point TopLeftLegend(0, 390);
        // cv::Point BottomRightLegend(230, 450);
        // cv::Point InnerTopLeftLegend(4, 404);
        // cv::Point InnerBottomRightLegend(196, 446);

        // rectangle(img, TopLeftLegend, BottomRightLegend, Scalar(0, 0, 0), 4);
        // rectangle(img, InnerTopLeftLegend, InnerBottomRightLegend, Scalar(255, 255, 255), 1);

        // putText(img, "Detected",
        //         TopLeftLegend + Point(84, 26), FONT_HERSHEY_DUPLEX,
        //         1, Scalar(0, 0, 0));
        // putText(img, "Desired",
        //         TopLeftLegend + Point(84, 52), FONT_HERSHEY_DUPLEX,
        //         1, Scalar(0, 0, 0));
        // //blue rect. Desired
        // rectangle(img, TopLeftLegend + Point(5, 12), TopLeftLegend + Point(80, 22),
        //           Scalar(255, 0, 0), FILLED);
        // //red rect. Detected
        // rectangle(img, TopLeftLegend + Point(5, 38), TopLeftLegend + Point(80, 48),
        //           Scalar(0, 0, 255), FILLED);
        // /**
        //  * @brief DRAW A LEGEND
        //  */

        // crosshair
        line(img, Point(cols / 2 - 20, rows / 2), 
            Point(cols / 2 + 20, rows / 2), Scalar(0, 0, 0));
        line(img, Point(cols / 2, rows / 2 - 20), 
            Point(cols / 2, rows / 2 + 20), Scalar(0, 0, 0));

        Point p = Point(30, 30);
        imshow("Camera Feed", img);
        char c = (char)waitKey(1);
        if (c == 27)
            break;
    }
    Pylon::PylonTerminate();
    destroyAllWindows();
    return 0;
}