#include "extrinsic_calibration.h"
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <iomanip>
#include <string>

#define IMG         900
#define CENTER      (IMG / 2)
#define SCALE       1.0
#define BOARD_R     170.0
#define AXIS_SCALE  1500.0

static cv::Mat R, T;
static bool dragging = false;

/* =====================================================================
   Load / Save extrinsics (R|t)
   ===================================================================== */
bool load_yaml(const char* file)
{
    cv::FileStorage fs(file, cv::FileStorage::READ);
    if (!fs.isOpened())
    {
        std::cerr << "[LOAD] Cannot open " << file << "\n";
        return false;
    }

    fs["R"] >> R;
    fs["t"] >> T;

    if (R.empty() || T.empty())
    {
        std::cerr << "[LOAD] R or T is empty in yaml!\n";
        return false;
    }

    std::cout << "[LOAD] R loaded:\n" << R << "\n";
    std::cout << "[LOAD] T loaded:\n" << T << "\n";
    return true;
}

void save_yaml(const char* file)
{
    cv::FileStorage fs(file, cv::FileStorage::WRITE);
    if (!fs.isOpened())
    {
        std::cerr << "[SAVE] Cannot write " << file << "\n";
        return;
    }
    fs << "R" << R;
    fs << "t" << T;
    std::cout << "[SAVE] Extrinsics saved to " << file << "\n";
}

/* =====================================================================
   World -> Image (2D display, ignore Z)
   ===================================================================== */
cv::Point W2I(double X, double Y)
{
    return cv::Point(
        static_cast<int>(CENTER + X * SCALE),
        static_cast<int>(CENTER - Y * SCALE)
    );
}

/* =====================================================================
   Main debug scene drawing
   ===================================================================== */
void draw_scene(cv::Mat& img)
{
    img.setTo(cv::Scalar(30, 30, 40));  // dark background

    // Board / world reference circle
    cv::circle(img, W2I(0, 0), static_cast<int>(BOARD_R * SCALE),
               cv::Scalar(180, 180, 180), 2);

    if (R.empty() || T.empty() || R.rows != 3 || T.rows != 3)
    {
        cv::putText(img, "R or T not initialized", cv::Point(40, 80),
                    cv::FONT_HERSHEY_SIMPLEX, 1.1, cv::Scalar(0, 80, 255), 3);
        return;
    }

    // Camera position in world frame
    cv::Mat Rt = R.t();               // R transpose
    cv::Mat C = -Rt * T;              // C = -R^T * t
    double Cx = C.at<double>(0, 0);
    double Cy = C.at<double>(1, 0);
    double Cz = C.at<double>(2, 0);

    // Camera axes expressed in world frame
    cv::Mat ex = Rt * (cv::Mat_<double>(3, 1) << 1, 0, 0);
    cv::Mat ey = Rt * (cv::Mat_<double>(3, 1) << 0, 1, 0);
    cv::Mat ez = Rt * (cv::Mat_<double>(3, 1) << 0, 0, 1);

    // Camera position marker
    cv::Point cam_pt = W2I(Cx, Cy);
    cv::circle(img, cam_pt, 8, cv::Scalar(0, 220, 220), -1);
    cv::circle(img, cam_pt, 12, cv::Scalar(255, 255, 0), 2);

    // Axes (long for visibility)
    const double len = AXIS_SCALE;

    // X red
    cv::Point px = W2I(Cx + ex.at<double>(0) * len, Cy + ex.at<double>(1) * len);
    cv::line(img, cam_pt, px, cv::Scalar(0, 0, 255), 3);
    cv::circle(img, px, 6, cv::Scalar(0, 0, 255), -1);

    // Y green
    cv::Point py = W2I(Cx + ey.at<double>(0) * len, Cy + ey.at<double>(1) * len);
    cv::line(img, cam_pt, py, cv::Scalar(0, 255, 0), 3);
    cv::circle(img, py, 6, cv::Scalar(0, 255, 0), -1);

    // Z blue
    cv::Point pz = W2I(Cx + ez.at<double>(0) * len, Cy + ez.at<double>(1) * len);
    cv::line(img, cam_pt, pz, cv::Scalar(255, 255, 0), 3);
    cv::circle(img, pz, 6, cv::Scalar(255, 255, 0), -1);

    // Text info
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(2);
    oss << "Camera @ " << Cx << " , " << Cy << " , " << Cz;
    cv::putText(img, oss.str(), cv::Point(20, 40),
                cv::FONT_HERSHEY_SIMPLEX, 0.9, cv::Scalar(220, 220, 100), 2);

    double tnorm = cv::norm(T);
    oss.str("");
    oss << "||t|| = " << tnorm << " mm";
    cv::putText(img, oss.str(), cv::Point(20, 70),
                cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(180, 180, 255), 2);

    cv::Mat rvec;
    cv::Rodrigues(R, rvec);
    double angle_deg = cv::norm(rvec) * 180.0 / CV_PI;
    oss.str("");
    oss << "Rotation : " << angle_deg << " deg";
    cv::putText(img, oss.str(), cv::Point(20, 100),
                cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(180, 255, 180), 2);
}

/* =====================================================================
   Mouse callback: manual camera move (XY plane only)
   ===================================================================== */
void mouse_cb(int event, int x, int y, int flags, void*)
{
    (void)flags;

    if (R.empty() || T.empty()) return;

    if (event == cv::EVENT_LBUTTONDOWN)
        dragging = true;

    if (event == cv::EVENT_LBUTTONUP)
        dragging = false;

    if (dragging && event == cv::EVENT_MOUSEMOVE)
    {
        double world_x = (x - CENTER) / SCALE;
        double world_y = (CENTER - y) / SCALE;

        cv::Mat Rt = R.t();
        cv::Mat C = -Rt * T;

        C.at<double>(0, 0) = world_x;
        C.at<double>(1, 0) = world_y;

        T = -R * C;

        std::cout << "Camera moved -> Cx=" << world_x
                  << "  Cy=" << world_y
                  << "  Cz=" << C.at<double>(2,0) << "\n";
    }
}

/* =====================================================================
   Main debug / manual tuning loop
   ===================================================================== */
int live_calibrate_extrinsics(int cam_id,
                              int w, int h,
                              const char* intrinsic,
                              const char* output_yaml)
{
    (void)cam_id; (void)w; (void)h; (void)intrinsic;

    std::cout << "\n=== Extrinsics debug mode ===\n\n";
    std::cout << "Keys:\n";
    std::cout << "  s     -> save\n";
    std::cout << "  a/d   -> small rotation around Y\n";
    std::cout << "  click + drag -> move camera (XY plane)\n";
    std::cout << "  ESC   -> quit\n\n";

    if (!load_yaml(output_yaml))
    {
        std::cout << "[INIT] No extrinsics found -> identity\n";
        R = cv::Mat::eye(3, 3, CV_64F);
        T = cv::Mat::zeros(3, 1, CV_64F);
    }

    cv::Mat canvas(IMG, IMG, CV_8UC3);
    cv::namedWindow("extrinsic_debug", cv::WINDOW_AUTOSIZE);
    cv::setMouseCallback("extrinsic_debug", mouse_cb);

    while (true)
    {
        draw_scene(canvas);
        cv::imshow("extrinsic_debug", canvas);

        int key = cv::waitKey(30);
        if (key == 27) break;

        if (key == 's' || key == 'S')
        {
            save_yaml(output_yaml);
        }

        if (key == 'a' || key == 'A')
        {
            cv::Mat rvec = (cv::Mat_<double>(3,1) << 0, 0.04, 0);
            cv::Mat dR; cv::Rodrigues(rvec, dR);
            R = dR * R;
            std::cout << "[ROT] +0.04 rad around Y\n";
        }
        if (key == 'd' || key == 'D')
        {
            cv::Mat rvec = (cv::Mat_<double>(3,1) << 0, -0.04, 0);
            cv::Mat dR; cv::Rodrigues(rvec, dR);
            R = dR * R;
            std::cout << "[ROT] -0.04 rad around Y\n";
        }
    }

    std::cout << "\n=== Debug mode finished ===\n";
    cv::destroyAllWindows();
    return 0;
}
