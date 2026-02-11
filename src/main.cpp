#include <iostream>
#include <opencv2/opencv.hpp>
#include <sl/Camera.hpp>

using namespace std;
using namespace sl;

void on_trackbar(int, void*) {}

int main() {
    Camera zed;

    InitParameters init_params;
    init_params.sdk_verbose = true;
    init_params.depth_mode = DEPTH_MODE::ULTRA;
    init_params.coordinate_system = COORDINATE_SYSTEM::LEFT_HANDED_Z_UP;
    init_params.coordinate_units = UNIT::CENTIMETER;
    init_params.depth_minimum_distance = 0.15;
    init_params.camera_resolution = RESOLUTION::VGA;
    init_params.camera_fps = 30;

    ERROR_CODE err = zed.open(init_params);
    if (err != ERROR_CODE::SUCCESS) {
        cout << "Failed to open ZED camera. Error: " << toString(err) << endl;
        return -1;
    }

    sl::Mat image;
    sl::Mat point_cloud;

    int h_min = 0, s_min = 83, v_min = 0;
    int h_max = 27, s_max = 255, v_max = 255;

    cv::namedWindow("Trackbars", cv::WINDOW_AUTOSIZE);
    cv::createTrackbar("H Min", "Trackbars", &h_min, 179, on_trackbar);
    cv::createTrackbar("H Max", "Trackbars", &h_max, 179, on_trackbar);
    cv::createTrackbar("S Min", "Trackbars", &s_min, 255, on_trackbar);
    cv::createTrackbar("S Max", "Trackbars", &s_max, 255, on_trackbar);
    cv::createTrackbar("V Min", "Trackbars", &v_min, 255, on_trackbar);
    cv::createTrackbar("V Max", "Trackbars", &v_max, 255, on_trackbar);

    while (true) {
        if (zed.grab() == ERROR_CODE::SUCCESS) {
            zed.retrieveImage(image, VIEW::LEFT);
            zed.retrieveMeasure(point_cloud, MEASURE::XYZ); // point cloud 3D

            cv::Mat image_cv(image.getHeight(), image.getWidth(), CV_8UC4, image.getPtr<sl::uchar1>(MEM::CPU));
            image_cv = image_cv.clone(); // safe clone
            cv::Mat image_bgr;
            cv::cvtColor(image_cv, image_bgr, cv::COLOR_BGRA2BGR);

            cv::Mat hsv_image;
            cv::cvtColor(image_bgr, hsv_image, cv::COLOR_BGR2HSV);

            cv::Mat mask;
            cv::inRange(hsv_image, cv::Scalar(h_min, s_min, v_min), cv::Scalar(h_max, s_max, v_max), mask);

            std::vector<std::vector<cv::Point>> ball_contours;
            cv::findContours(mask, ball_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

            double largest = 0;
            std::vector<cv::Point> ballContour;

            for (auto &contour : ball_contours) {
                double area = cv::contourArea(contour);
                if (area > largest) {
                    largest = area;
                    ballContour = contour;
                }
            }

            if (!ballContour.empty() && largest > 4) {
                cv::Moments moments = cv::moments(ballContour);
                cv::Point center(moments.m10 / moments.m00, moments.m01 / moments.m00);

                cv::circle(image_cv, center, 7, cv::Scalar(0, 255, 255), -1); 

                sl::float4 point;
                point_cloud.getValue(center.x, center.y, &point);

                if (std::isfinite(point.z)) {
                    cout << "Dist: " << point.z << " cm" << endl;
                } else {
                    cout << "No data" << endl;
                }
            }

            cv::imshow("Original", image_cv);
            cv::imshow("Mask", mask);
        } else {
            cout << "Failed to fetch frame from ZED." << endl;
        }

        char key = cv::waitKey(1);
        if (key == 'q' || key == 27) break;
    }

    zed.close();
    return 0;
}
