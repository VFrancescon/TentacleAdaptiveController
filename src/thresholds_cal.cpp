#include "extraCameraFuncs.hpp"

using namespace cv;
const int max_value_H = 256;
const int max_value = 256;
const String window_capture_name = "Video Capture";
const String window_detection_name = "Object Detection";

int g_slider = 60;       // slider pos value
int g_slider_max = 256;  // slider max value

void on_low_H_thresh_trackbar(int, void*) { printf("%d\n", g_slider); }

// Mat IntroducerMask(Mat src);

int main(int argc, char* argv[]) {
    // VideoCapture
    // cap("/home/vittorio/coil_manipulator/src/opencv/BothRoutes_INOUT_V1.mp4");
    namedWindow(window_capture_name);
    namedWindow(window_detection_name);
    createTrackbar("Low Thresh", window_detection_name, &g_slider, g_slider_max,
                   on_low_H_thresh_trackbar);
    Mat frame, frame_BGR, frame_threshold;
    Mat contourMAT;

    /*pylon video input here
    -----------------------------------------------------------
    */
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
    Pylon::CPixelTypeMapper pixelTypeMapper(&pixelFormat);
    Pylon::EPixelType pixelType =
        pixelTypeMapper.GetPylonPixelTypeFromNodeValue(
            pixelFormat.GetIntValue());
    camera.StartGrabbing(Pylon::GrabStrategy_LatestImageOnly);
    Pylon::CGrabResultPtr ptrGrabResult;

    /*-----------------------------------------------------------
    pylon video input here*/

    // cap >> frame;
    // int rows = frame.rows * 3 / 8;
    // int cols = frame.cols * 3 / 8;
    // make image smaller
    // resize(frame, frame, Size(cols, rows), INTER_LINEAR);

    /*
    Add streaming pylonImg to frame here
    */
    // camera.RetrieveResult(5000, ptrGrabResult,
    // Pylon::TimeoutHandling_ThrowException); const uint8_t* preImageBuffer =
    // (uint8_t*) ptrGrabResult->GetBuffer();
    // formatConverter.Convert(pylonImage, ptrGrabResult);
    // frame = cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(),
    // CV_8UC3, (uint8_t *) pylonImage.GetBuffer());

    // Mat introducer_mask = IntroducerMask(frame);

    while (camera.IsGrabbing()) {
        // // while(true){
        camera.RetrieveResult(5000, ptrGrabResult,
                              Pylon::TimeoutHandling_ThrowException);
        const uint8_t* pImageBuffer = (uint8_t*)ptrGrabResult->GetBuffer();
        formatConverter.Convert(pylonImage, ptrGrabResult);
        frame = cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(),
                        CV_8UC3, (uint8_t*)pylonImage.GetBuffer());
        // cap >> frame;

        if (frame.empty()) {
            break;
        }

        int rows = frame.rows * 3 / 8;
        int cols = frame.cols * 3 / 8;
        // make image smaller
        resize(frame, frame, Size(cols, rows), INTER_LINEAR);

        cvtColor(frame, frame_BGR, COLOR_BGR2GRAY);
        blur(frame_BGR, frame_BGR, Size(5, 5));
        threshold(frame_BGR, frame_threshold, g_slider, 255, THRESH_BINARY_INV);

        // frame_threshold.copyTo(contourMAT, introducer_mask);
        // std::vector<std::vector<Point>> contours;
        // std::vector<Vec4i> hierarchy;
        // findContours(contourMAT, contours, hierarchy, RETR_LIST,
        // CHAIN_APPROX_SIMPLE); contourMAT = Mat::zeros(frame_threshold.rows,
        // frame_threshold.cols, CV_8UC1 ); drawContours(contourMAT, contours,
        // -1, Scalar(255,255,0), CV_FILLED, LINE_8, hierarchy);
        // ximgproc::thinning(contourMAT, contourMAT, 0);
        // Show the frames
        imshow(window_capture_name, frame);
        imshow(window_detection_name, frame_threshold);
        // imshow("mask", introducer_mask);
        // imshow("mask", introducer_mask);
        char key = (char)waitKey(30);
        if (key == 'q' || key == 27) {
            break;
        }
        if (key == 'n') {
            // cap >> frame;
            int rows = frame.rows * 3 / 8;
            int cols = frame.cols * 3 / 8;

            // make image smaller
            resize(frame, frame, Size(cols, rows), INTER_LINEAR);
        }
    }
    camera.DestroyDevice();
    Pylon::PylonTerminate();
    // cap.release();
    // destroyAllWindows();
    return 0;
}
