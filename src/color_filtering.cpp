#include <pylon/PylonIncludes.h>
#include <sys/stat.h>

#include <eigen3/Eigen/Core>
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/ximgproc.hpp>
#include <source/AStar.hpp>

#include "AdaptiveControllerVisionClass/visionClass.hpp"
using namespace cv;

int h_slider = 0;        // slider pos value
int h_slider_max = 255;  // slider max value
int s_slider = 0;        // slider pos value
int s_slider_max = 255;  // slider max value
int v_slider = 0;        // slider pos value
int v_slider_max = 255;  // slider max value

void on_low_h_thresh_trackbar(int, void*) { printf("%d\n", h_slider); }
void on_low_s_thresh_trackbar(int, void*) { printf("%d\n", s_slider); }
void on_low_v_thresh_trackbar(int, void*) { printf("%d\n", v_slider); }
const String window_capture_name = "Video Capture";
const String window_detection_name = "Object Detection";

int main(int argc, char* argv[]) {
    namedWindow(window_capture_name);
    namedWindow(window_detection_name);
    createTrackbar("Low Hue", window_detection_name, &h_slider, h_slider_max,
                   on_low_h_thresh_trackbar);
    createTrackbar("Low Sat", window_detection_name, &s_slider, s_slider_max,
                   on_low_s_thresh_trackbar);
    createTrackbar("Low V", window_detection_name, &v_slider, v_slider_max,
                   on_low_v_thresh_trackbar);

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
        .SetValue(25000.0);

    Pylon::CFloatParameter(camera.GetNodeMap(), "BslSaturation")
        .TrySetValue(1.5, Pylon::FloatValueCorrection_ClipToRange);

    Pylon::CFloatParameter(camera.GetNodeMap(), "BslHue")
        .TrySetValue(20, Pylon::FloatValueCorrection_ClipToRange);

    Pylon::CFloatParameter(camera.GetNodeMap(), "BslContrast")
        .TrySetValue(0.5, Pylon::FloatValueCorrection_ClipToRange);

    // Pylon::CEnumParameter(camera.GetNodeMap(), "BslLightSourcePreset")
    //     .TrySetValue("FactoryLED6000K");
    // Pylon::CEnumParameter(camera.GetNodeMap(),
    // "BslLightSourcePresetFeatureSelector").TrySetValue("ColorAdjustment");
    // Pylon::CBooleanParameter(camera.GetNodeMap(),
    // "BslLightSourcePresetFeatureEnable").TrySetValue(false);

    Pylon::CPixelTypeMapper pixelTypeMapper(&pixelFormat);
    Pylon::EPixelType pixelType =
        pixelTypeMapper.GetPylonPixelTypeFromNodeValue(
            pixelFormat.GetIntValue());
    camera.StartGrabbing(Pylon::GrabStrategy_LatestImageOnly);
    Pylon::CGrabResultPtr ptrGrabResult;
    VisionClass viz;
    while (camera.IsGrabbing()) {
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
        resize(frame, frame, Size(frame.cols * 3 / 8, frame.rows * 3 / 8),
               INTER_LINEAR);
        cvtColor(frame, frame_HSV, COLOR_BGR2HSV);
        inRange(frame_HSV, Scalar(h_slider, s_slider, v_slider), Scalar(255, 255, 255), mask);

        /*delete left-hand artifacts*/
        Point p1(0, 0), p2(0, frame.rows), p3(frame.cols * 0.15, 0);
        std::vector<Point> lpts = {p1, p2, p3};

        Point p4(frame.cols, 0), p5(frame.cols, frame.rows),
            p6(frame.cols * 0.95, 0);
        std::vector<Point> rpts = {p4, p5, p6};

        polylines(mask, lpts, true, Scalar(0, 0, 0), 125);
        polylines(mask, rpts, true, Scalar(0, 0, 0), 120);

        bitwise_and(frame, frame, final_result, mask);
        final_result = viz.preprocessImg(frame, rrows, rcols);

        // blur(mask, mask, Size(3, 3));
        imshow(window_detection_name, mask);
        imshow(window_capture_name, frame);
        // imshow("mask", introducer_mask);
        // imshow("mask", introducer_mask);
        char key = (char)waitKey(30);
        if (key == 'q' || key == 27) {
            break;
        }
    }

    return 0;
}