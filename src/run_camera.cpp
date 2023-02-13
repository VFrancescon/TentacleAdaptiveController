#include <algorithm>
#include <opencv2/ximgproc.hpp>
#include <opencv2/video.hpp>
#include <fstream>
#include <chrono>
#include <unistd.h>
#include <iostream>
#include <eigen3/Eigen/Core>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <pylon/PylonIncludes.h>
#include <sys/stat.h>
#include <iomanip>
using namespace cv;


int threshold_low = 131;
int threshold_high = 255;
int link_length = 55;

int PYLON_WIDTH = 1920;
int PYLON_HEIGHT = 1200;


using namespace cv;
int main(int argc, char* argv[]){
    Mat img;
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
    img = cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC3, (uint8_t *) pylonImage.GetBuffer());

    // VideoWriter video_out(home_path + "coil_manipulator/output.avi", VideoWriter::fourcc('M', 'J', 'P', 'G'), 10, 
    //             Size(img.rows / 2, img.cols * 3 / 8));

    //resizing the image for faster processing
    int rows = img.rows * 3 / 8;
    int cols = img.cols * 3 / 8; 
    resize(img, img, Size(cols, rows), INTER_LINEAR);

    while(camera.IsGrabbing()){
        camera.RetrieveResult(5000, ptrGrabResult, Pylon::TimeoutHandling_ThrowException);
        const uint8_t* pImageBuffer = (uint8_t*) ptrGrabResult->GetBuffer();
        formatConverter.Convert(pylonImage, ptrGrabResult);
        img = cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC3, (uint8_t *) pylonImage.GetBuffer());

        // cap >> img;
        if(img.empty())
        {
            break;
        }
        
        //make image smaller 
        resize(img, img, Size(cols, rows), INTER_LINEAR);
        // flip(img, img, 1);
        // line( img, Point(0, cols/3), Point(cols, rows/3), Scalar(0,0,0) ); //horizontal lines
        // line( img, Point(0, cols*2/3), Point(cols, rows/3*2), Scalar(0,0,0) );

        // line( img, Point(rows/3, 0), Point(rows/3, cols), Scalar(0,0,0) ); //horizontal lines
        // line( img, Point(rows*2/3), Point(rows*2/3, cols), Scalar(0,0,0) );

        //crosshair
        line(img, Point(cols/2-20, rows/2), Point(cols/2+20, rows/2), Scalar(0,0,0));
        line(img, Point(cols/2, rows/2-20), Point(cols/2, rows/2+20), Scalar(0,0,0));

        
        Point p = Point(30,30);
        imshow("Camera Feed", img);
        char c= (char)waitKey(1);
        if(c==27) break;
    }
    Pylon::PylonTerminate();
    destroyAllWindows();
    return 0;
}