#include <iostream>
#include <eigen3/Eigen/Core>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/ximgproc.hpp>
#include <pylon/PylonIncludes.h>
#include <source/AStar.hpp>
#include <sys/stat.h>
#include "AdaptiveControllerVisionClass/visionClass.hpp"
using namespace cv;

int main( int argc, char* argv[]){

    int exposureTime = 20000;

    int PYLON_WIDTH = 1920;
    int PYLON_HEIGHT = 1200;
    Mat frame, frame_HSV, mask, final_result;

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
        .SetValue(exposureTime);
    Size frameSize = Size((int)width.GetValue(), (int)height.GetValue());
    int codec = VideoWriter::fourcc('M', 'J', 'P', 'G');
    width.TrySetValue(PYLON_WIDTH, Pylon::IntegerValueCorrection_Nearest);
    height.TrySetValue(PYLON_HEIGHT, Pylon::IntegerValueCorrection_Nearest);
    Pylon::CFloatParameter(camera.GetNodeMap(), "ExposureTime")
        .SetValue(20000.0);

    Pylon::CFloatParameter(camera.GetNodeMap(), "Saturation")
        .TrySetValue(0.7, Pylon::FloatValueCorrection_ClipToRange);

    Pylon::CFloatParameter(camera.GetNodeMap(), "Hue")
        .TrySetValue(10, Pylon::FloatValueCorrection_ClipToRange);

    Pylon::CFloatParameter(camera.GetNodeMap(), "Contrast")
        .TrySetValue(0.4, Pylon::FloatValueCorrection_ClipToRange);

    // Pylon::CEnumParameter(camera.GetNodeMap(), "BslLightSourcePreset")
    //     .TrySetValue("FactoryLED6000K");
    // Pylon::CEnumParameter(camera.GetNodeMap(), "BslLightSourcePresetFeatureSelector").TrySetValue("ColorAdjustment");
    // Pylon::CBooleanParameter(camera.GetNodeMap(), "BslLightSourcePresetFeatureEnable").TrySetValue(false);

    Pylon::CPixelTypeMapper pixelTypeMapper(&pixelFormat);
    Pylon::EPixelType pixelType =
        pixelTypeMapper.GetPylonPixelTypeFromNodeValue(
            pixelFormat.GetIntValue());
    camera.StartGrabbing(Pylon::GrabStrategy_LatestImageOnly);
    Pylon::CGrabResultPtr ptrGrabResult;
    VisionClass viz;
    while(camera.IsGrabbing()){ 
        camera.RetrieveResult(5000, ptrGrabResult,
                              Pylon::TimeoutHandling_ThrowException);
        const uint8_t* pImageBuffer = (uint8_t*)ptrGrabResult->GetBuffer();
        formatConverter.Convert(pylonImage, ptrGrabResult);
        frame = cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(),
                        CV_8UC3, (uint8_t*)pylonImage.GetBuffer());
            int rrows = frame.rows * 3 / 8;
            int rcols = frame.cols * 3 / 8;
        // cap >> frame;


        if (frame.empty()) {
            break;
        }
        resize( frame, frame, Size(frame.cols * 3 / 8, frame.rows * 3 / 8) , INTER_LINEAR );
        cvtColor(frame, frame_HSV, COLOR_BGR2HSV);
        inRange(frame_HSV, Scalar(0,99,67), Scalar(255,255,255), mask);
        
        /*delete left-hand artifacts*/
        Point p1(0,0), p2(0, frame.rows), p3(frame.cols * 0.15, 0);
        std::vector<Point> lpts = {p1, p2, p3};

        Point p4(frame.cols, 0), p5(frame.cols, frame.rows), p6(frame.cols * 0.95, 0);
        std::vector<Point> rpts = {p4, p5, p6};

        polylines(mask, lpts, true, Scalar(0,0,0), 125);
        polylines(mask, rpts, true, Scalar(0,0,0), 120);
        
        
        bitwise_and(frame, frame, final_result, mask);
        final_result = viz.preprocessImg(frame, rrows, rcols);

        blur(mask, mask, Size(3,3));
        imshow("ProcessedImage", final_result);
        imshow("Raw", frame);
        // imshow("mask", introducer_mask);
        // imshow("mask", introducer_mask);
        char key = (char)waitKey(30);
        if (key == 'q' || key == 27) {
            break;
        }


    }

    return 0;
}