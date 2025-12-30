#include "dart_detector.h"
#include <opencv2/opencv.hpp>
#include <opencv2/video/tracking.hpp>
#include <iostream>
#include <vector>
#include <cmath>

// ==================== GLOBALS ====================
static int img_width = 0;
static int img_height = 0;

static cv::Mat prev_gray, current_gray;
static bool first_frame = true;

// ==================== ÉTAT DU SYSTÈME ====================
enum DetectorState {
    IDLE,
    IMPACT_DETECTED
};

static DetectorState state = IDLE;

// ==================== MÉMOIRE ====================
static cv::Point2f last_valid_tip;

// ==================== PARAMÈTRES ====================
static constexpr int MIN_CONTOUR_AREA = 5;   // pixels mini pour conserver un contour
static constexpr float DIST_THRESHOLD = 10.f; // distance max d'un point à la droite

// =========================================================

int dart_detector_init(int width, int height)
{
    img_width = width;
    img_height = height;

    cv::namedWindow("DIFF", cv::WINDOW_AUTOSIZE);
    cv::namedWindow("DEBUG", cv::WINDOW_AUTOSIZE);

    std::cout << "[DART] Detector initialized (difference + central filtering)" << std::endl;
    return 0;
}

// =========================================================

int dart_detector_process(const unsigned char* input_frame,
                          size_t frame_size,
                          double* impact_u,
                          double* impact_v)
{
    if (!input_frame || frame_size < (size_t)img_width * img_height * 3)
        return -1;

    cv::Mat frame(img_height, img_width, CV_8UC3, (void*)input_frame);
    cv::cvtColor(frame, current_gray, cv::COLOR_BGR2GRAY);

    if (first_frame) {
        current_gray.copyTo(prev_gray);
        first_frame = false;
        return 0;
    }

    // ==================== DIFF ====================
    cv::Mat diff_full;
    cv::absdiff(current_gray, prev_gray, diff_full);
    cv::GaussianBlur(diff_full, diff_full, cv::Size(5,5), 0);
    cv::threshold(diff_full, diff_full, 40, 255, cv::THRESH_BINARY);

    // ==================== TIERS CENTRAL ====================
    int h_start = img_height / 3;
    int h_end   = 2 * img_height / 3;
    cv::Mat diff_central = diff_full(cv::Range(h_start, h_end), cv::Range::all()).clone();

    // ==================== CONTOURS ====================
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(diff_central, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if (contours.empty()) {
        prev_gray = current_gray.clone();
        return 0;
    }

    auto max_cont_it = std::max_element(contours.begin(), contours.end(),
                                       [](const std::vector<cv::Point>& a,
                                          const std::vector<cv::Point>& b){
                                           return cv::contourArea(a) < cv::contourArea(b);
                                       });
    std::vector<cv::Point> main_contour = *max_cont_it;

    for (auto &pt : main_contour) pt.y += h_start;

    // ==================== FIT LINE ====================
    cv::Vec4f line;
    cv::fitLine(main_contour, line, cv::DIST_L2, 0, 0.01, 0.01);
    cv::Point2f line_dir(line[0], line[1]);
    cv::Point2f line_pt(line[2], line[3]);
    if (line_dir.y < 0) line_dir = -line_dir;

    // ==================== FILTRAGE ====================
    cv::Mat diff_filtered = cv::Mat::zeros(diff_full.size(), CV_8UC1);
    for (int y = 0; y < diff_full.rows; ++y) {
        for (int x = 0; x < diff_full.cols; ++x) {
            if (diff_full.at<uchar>(y,x) > 0) {
                cv::Point2f pt(x,y);
                float dist = std::abs((pt.x - line_pt.x) * line_dir.y - (pt.y - line_pt.y) * line_dir.x);
                if (dist < DIST_THRESHOLD)
                    diff_filtered.at<uchar>(y,x) = 255;
            }
        }
    }

    // ==================== SUPPRESSION PETITS BRUITS ====================
    std::vector<std::vector<cv::Point>> filtered_contours;
    cv::findContours(diff_filtered, filtered_contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    cv::Mat final_mask = cv::Mat::zeros(diff_filtered.size(), CV_8UC1);
    for (auto &c : filtered_contours) {
        if (cv::contourArea(c) >= MIN_CONTOUR_AREA)
            cv::drawContours(final_mask, std::vector<std::vector<cv::Point>>{c}, -1, 255, cv::FILLED);
    }

    // ==================== IMPACT ====================
    std::vector<cv::Point> points;
    cv::findNonZero(final_mask, points);
    if (points.empty()) {
        prev_gray = current_gray.clone();
        return 0;
    }

    cv::Point impact_pt = *std::max_element(points.begin(), points.end(),
                                            [](const cv::Point& a, const cv::Point& b){
                                                return a.y < b.y;
                                            });

    last_valid_tip = impact_pt;
    if (impact_u) *impact_u = last_valid_tip.x;
    if (impact_v) *impact_v = last_valid_tip.y;

    std::cout << "[DART] IMPACT @ (" << last_valid_tip.x << ", " << last_valid_tip.y << ")" << std::endl;

    // ==================== DEBUG ====================
    cv::Mat debug = frame.clone();
    cv::circle(debug, last_valid_tip, 8, {0,255,0}, -1);
    cv::imshow("DIFF", diff_full);
    cv::imshow("DEBUG", debug);
    cv::waitKey(1);

    prev_gray = current_gray.clone();
    state = IMPACT_DETECTED;
    return 1;
}

// =========================================================

void dart_detector_close(void)
{
    cv::destroyAllWindows();
    std::cout << "[DART] Detector closed." << std::endl;
}
