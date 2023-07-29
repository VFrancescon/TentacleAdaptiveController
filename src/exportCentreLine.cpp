#include "ControllerPrototype.hpp"

void drawLegend(Mat &post_img);
Mat preprocessImg(Mat &post_img);

int rrows;
int rcols;


int main(int argc, char *argv[]) {

    // time_t curr_time;
    // tm *curr_tm;
    // time(&curr_time);
    // curr_tm = localtime(&curr_time);
    // char date_string[100];
    // strftime(date_string, 50, "%d_%m_%y_%H%M%S", curr_tm);
    // std::string date(date_string);
    // std::ofstream extractedPoints;
    // extractedPoints.open("../extractedPoints.csv", std::ios_base::app);
    // extractedPoints << date << "\n";
    // extractedPoints
    //     << "Px, Py" << "\n";

    // Mat pre_img, post_img, intr_mask;
    // Pylon::PylonInitialize();
    // Pylon::CImageFormatConverter formatConverter;
    // formatConverter.OutputPixelFormat = Pylon::PixelType_BGR8packed;
    // Pylon::CPylonImage pylonImage;
    // Pylon::CInstantCamera camera(
    //     Pylon::CTlFactory::GetInstance().CreateFirstDevice());
    // camera.Open();
    // Pylon::CIntegerParameter width(camera.GetNodeMap(), "Width");
    // Pylon::CIntegerParameter height(camera.GetNodeMap(), "Height");
    // Pylon::CEnumParameter pixelFormat(camera.GetNodeMap(), "PixelFormat");

    // Pylon::CFloatParameter(camera.GetNodeMap(), "ExposureTime")
    //     .SetValue(20000.0);

    // Size frameSize = Size((int)width.GetValue(), (int)height.GetValue());
    // int codec = VideoWriter::fourcc('M', 'J', 'P', 'G');
    // width.TrySetValue(PYLON_WIDTH, Pylon::IntegerValueCorrection_Nearest);
    // height.TrySetValue(PYLON_HEIGHT, Pylon::IntegerValueCorrection_Nearest);
    // Pylon::CPixelTypeMapper pixelTypeMapper(&pixelFormat);
    // Pylon::EPixelType pixelType =
    //     pixelTypeMapper.GetPylonPixelTypeFromNodeValue(
    //         pixelFormat.GetIntValue());
    // camera.StartGrabbing(Pylon::GrabStrategy_LatestImageOnly);
    // Pylon::CGrabResultPtr ptrGrabResult;
    // camera.RetrieveResult(5000, ptrGrabResult,
    //                       Pylon::TimeoutHandling_ThrowException);
    // const uint8_t *preImageBuffer = (uint8_t *)ptrGrabResult->GetBuffer();
    // formatConverter.Convert(pylonImage, ptrGrabResult);
    // pre_img = cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(),
    //                   CV_8UC3, (uint8_t *)pylonImage.GetBuffer());
    Mat pre_img = imread("../small_proc/raw_frame.png", IMREAD_COLOR);
    Mat mask = imread("../small_proc/raw_mask.jpg", IMREAD_GRAYSCALE);
    // resizing the image for faster processing
    rrows = pre_img.rows * 3 / 8;
    rcols = pre_img.cols * 3 / 8;

    // resize(pre_img, pre_img, Size(rcols, rrows), INTER_LINEAR);
    Mat pre_img_grey, pre_img_th;
    Mat pre_img_masked = Mat::zeros(Size(rcols, rrows), CV_8UC1);

    cvtColor(pre_img, pre_img_grey, COLOR_BGR2GRAY);
    blur(pre_img_grey, pre_img_grey, Size(5, 5));
    threshold(pre_img_grey, pre_img_th, threshold_low, threshold_high,
              THRESH_BINARY_INV);
    // post_img_th.copyTo(post_img_masked, intr_mask);
    pre_img_th.copyTo(pre_img_masked, mask);

    std::vector<Point> cntrLine;
    std::vector<std::vector<Point>> precontours;
    cntrLine = findCtrLine(pre_img_masked, precontours);
    Point base = cntrLine.at(0);

    // while(true){

        // camera.RetrieveResult(5000, ptrGrabResult,
        //                       Pylon::TimeoutHandling_ThrowException);
        // const uint8_t *pImageBuffer = (uint8_t *)ptrGrabResult->GetBuffer();
        // formatConverter.Convert(pylonImage, ptrGrabResult);
        // post_img =
        //     cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(),
        //             CV_8UC3, (uint8_t *)pylonImage.GetBuffer());

        // if (post_img.empty()) {
        //     break;
        // }
        // Mat post_img_masked = Mat::zeros(Size(rcols, rrows), CV_8UC1);
        // post_img_masked = preprocessImg(post_img);


        cntrLine = findCtrLine(pre_img_masked, precontours,base);
        Mat outputIMG = mask.clone();
        cvtColor(outputIMG, outputIMG, COLOR_GRAY2BGR);
        for(auto i: cntrLine){
            circle(outputIMG, i, 1, Scalar(255, 0, 0), 2);
            circle(pre_img, i, 1, Scalar(255, 0, 0), 2);
        }
        // imshow("Centreline", pre_img_masked);
        imshow("Pre_img", pre_img);
        imshow("output", outputIMG);
        
        //save pre_img
        imwrite("../small_proc/centreline.png", outputIMG);
        // imshow("mask", mask);
        char c = (char)waitKey(0);
        // if (c == 115) {
        //     // for(auto i: cntrLine){
        //     //     // extractedPoints << i.x << "," << i.y << "\n";
        //     // }
        // }
        
    return 0;
}

void drawLegend(Mat &post_img) {

    cv::Point TopLeftLegend(0, 390);
    cv::Point BottomRightLegend(230, 450);
    cv::Point InnerTopLeftLegend(4, 404);
    cv::Point InnerBottomRightLegend(196, 446);

    rectangle(post_img, TopLeftLegend, BottomRightLegend, Scalar(0, 0, 0), 4);
    rectangle(post_img, InnerTopLeftLegend, InnerBottomRightLegend,
              Scalar(255, 255, 255), 1);

    putText(post_img, "Detected", TopLeftLegend + Point(84, 26),
            FONT_HERSHEY_DUPLEX, 1, Scalar(0, 0, 0));
    putText(post_img, "Desired", TopLeftLegend + Point(84, 52),
            FONT_HERSHEY_DUPLEX, 1, Scalar(0, 0, 0));
    // blue rect. Desired
    rectangle(post_img, TopLeftLegend + Point(5, 12),
              TopLeftLegend + Point(80, 22), Scalar(255, 0, 0), FILLED);
    // red rect. Detected
    rectangle(post_img, TopLeftLegend + Point(5, 38),
              TopLeftLegend + Point(80, 48), Scalar(0, 0, 255), FILLED);


}

Mat preprocessImg(Mat &post_img){
    resize(post_img, post_img, Size(rcols, rrows), INTER_LINEAR);
    Mat post_img_grey, post_img_th;
    Mat post_img_masked = Mat::zeros(Size(rcols, rrows), CV_8UC1);

    cvtColor(post_img, post_img_grey, COLOR_BGR2GRAY);
    blur(post_img_grey, post_img_grey, Size(5, 5));
    threshold(post_img_grey, post_img_th, threshold_low, threshold_high,
                THRESH_BINARY_INV);
    // post_img_th.copyTo(post_img_masked, intr_mask);
    post_img_th.copyTo(post_img_masked);
    return post_img_masked;
}

