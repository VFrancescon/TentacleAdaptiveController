#include "ControllerPrototype.hpp"

void drawLegend(Mat &post_img);
Mat preprocessImg(Mat &post_img);

int rrows;
int rcols;

std::vector<Point> findCtrLine(Mat post_img_masked, std::vector<std::vector<Point>> &contours);

int main(int argc, char *argv[]) {

    time_t curr_time;
    tm *curr_tm;
    time(&curr_time);
    curr_tm = localtime(&curr_time);
    char date_string[100];
    strftime(date_string, 50, "%d_%m_%y_%H%M%S", curr_tm);
    std::string date(date_string);
    std::ofstream extractedPoints;
    extractedPoints.open("../extractedPoints.csv", std::ios_base::app);
    extractedPoints << date << "\n";
    extractedPoints
        << "Px, Py" << "\n";

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
    width.TrySetValue(PYLON_WIDTH, Pylon::IntegerValueCorrection_Nearest);
    height.TrySetValue(PYLON_HEIGHT, Pylon::IntegerValueCorrection_Nearest);
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

    resize(pre_img, pre_img, Size(rcols, rrows), INTER_LINEAR);
    Mat pre_img_grey, pre_img_th;
    Mat pre_img_masked = Mat::zeros(Size(rcols, rrows), CV_8UC1);

    cvtColor(pre_img, pre_img_grey, COLOR_BGR2GRAY);
    blur(pre_img_grey, pre_img_grey, Size(5, 5));
    threshold(pre_img_grey, pre_img_th, threshold_low, threshold_high,
              THRESH_BINARY_INV);
    // post_img_th.copyTo(post_img_masked, intr_mask);
    pre_img_th.copyTo(pre_img_masked);

    std::vector<Point> cntrLine;
    std::vector<std::vector<Point>> precontours;


    while(true){

        camera.RetrieveResult(5000, ptrGrabResult,
                              Pylon::TimeoutHandling_ThrowException);
        const uint8_t *pImageBuffer = (uint8_t *)ptrGrabResult->GetBuffer();
        formatConverter.Convert(pylonImage, ptrGrabResult);
        post_img =
            cv::Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(),
                    CV_8UC3, (uint8_t *)pylonImage.GetBuffer());

        if (post_img.empty()) {
            break;
        }
        Mat post_img_masked = Mat::zeros(Size(rcols, rrows), CV_8UC1);
        post_img_masked = preprocessImg(post_img);


        cntrLine = findCtrLine(post_img, precontours);

        cv::imshow("Post", post_img);
        imshow("Centreline", post_img);
        char c = (char)waitKey(1);
        if (c == 115) {
            for(auto i: cntrLine){
                extractedPoints << i.x << "," << i.y << "\n";
            }
        }
        if (c == 27) {
            break;
        }
        
    }


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

std::vector<Point> findCtrLine(Mat post_img_masked, std::vector<std::vector<Point>> &contours)
{
    // std::cout << "---------------\n\nExtafuncs version\n";
    Mat contours_bin;
    // std::vector<std::vector<Point> > contours;
    std::vector<Vec4i> hierarchy;
    // find contours, ignore hierarchy
    findContours(post_img_masked, contours, hierarchy, RETR_LIST, CHAIN_APPROX_SIMPLE);
    contours_bin = Mat::zeros(post_img_masked.size(), CV_8UC1);

    // draw contours and fill the open area
    drawContours(contours_bin, contours, -1, Scalar(255, 255, 255), cv::FILLED, LINE_8, hierarchy);
    // empty matrix. Set up to 8-bit 1 channel data. Very important to set up properly.
    Mat skeleton = Mat::zeros(post_img_masked.rows, post_img_masked.rows, CV_8U);

    // take the filled contour and thin it using Zhang Suen method. Only works with 8-bit 1 channel data.
    ximgproc::thinning(contours_bin, skeleton, 0);

    contours.clear();
    hierarchy.clear();
    findContours(skeleton, contours, hierarchy, RETR_LIST, CHAIN_APPROX_SIMPLE);

    std::vector<Point> cntLine;
    findNonZero(skeleton, cntLine);

    Point basePoint = cntLine.at(0);

    for(int i = 1; i < cntLine.size(); i++){
        cntLine.at(i) = basePoint + cntLine.at(i) ;
    }
    return cntLine;
}