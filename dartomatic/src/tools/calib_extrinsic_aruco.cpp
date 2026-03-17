#include <opencv2/opencv.hpp>
#include <opencv2/objdetect/aruco_detector.hpp>
#include <opencv2/calib3d.hpp>

#include <iostream>
#include <string>
#include <vector>

int main(int argc, char** argv)
{
    if (argc < 4) {
        std::cerr << "Usage: " << argv[0]
                  << " <camera_id> <calib_yaml> <output_yaml> [marker_length_m] [reproj_threshold_px]"
                  << std::endl;
        std::cerr << "Example: " << argv[0]
                  << " 0 data/cam_param/camera_params_0.yaml data/cam_param/camera_extrinsics_0.yaml 0.18 0.5"
                  << std::endl;
        return -1;
    }

    int         camera_id     = std::stoi(argv[1]);
    std::string calib_yaml    = argv[2];
    std::string output_yaml   = argv[3];
    float       marker_length = (argc > 4) ? std::stof(argv[4]) : 0.18f;
    double      threshold_px  = (argc > 5) ? std::stod(argv[5]) : 0.5;

    /* -------------------------------------------------- */
    /* Charger les intrinsèques                           */
    /* -------------------------------------------------- */
    cv::FileStorage fs(calib_yaml, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "[EXTR] Error: cannot open " << calib_yaml << std::endl;
        return -1;
    }

    cv::Mat camera_matrix, dist_coeffs;
    fs["camera_matrix"] >> camera_matrix;
    fs["distortion_coefficients"] >> dist_coeffs;
    fs.release();

    if (camera_matrix.empty() || dist_coeffs.empty()) {
        std::cerr << "[EXTR] Error: camera params not found in " << calib_yaml << std::endl;
        return -1;
    }

    std::cout << "[EXTR] Intrinsics loaded from " << calib_yaml << std::endl;
    std::cout << "[EXTR] Camera ID   : " << camera_id << std::endl;
    std::cout << "[EXTR] Output      : " << output_yaml << std::endl;
    std::cout << "[EXTR] Marker len  : " << marker_length << " m" << std::endl;
    std::cout << "[EXTR] Threshold   : " << threshold_px << " px" << std::endl;

    /* -------------------------------------------------- */
    /* Setup ArUco                                        */
    /* -------------------------------------------------- */
    cv::aruco::Dictionary dictionary =
        cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_50);

    cv::aruco::DetectorParameters parameters;
    cv::aruco::ArucoDetector detector(dictionary, parameters);

    /* -------------------------------------------------- */
    /* Ouvrir caméra                                       */
    /* -------------------------------------------------- */
    cv::VideoCapture cap(camera_id, cv::CAP_V4L2);
    if (!cap.isOpened()) {
        std::cerr << "[EXTR] Error: cannot open camera " << camera_id << std::endl;
        return -1;
    }

    cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M','J','P','G'));
    cap.set(cv::CAP_PROP_FRAME_WIDTH,  1280);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
    cap.set(cv::CAP_PROP_AUTOFOCUS, 0);
    cap.set(cv::CAP_PROP_FOCUS, 10);

    /* -------------------------------------------------- */
    /* Géométrie du marqueur                               */
    /* -------------------------------------------------- */
    /*
     * Convention :
     *   l'origine du monde est au centre du marqueur
     *   et donc au centre du plateau si le marqueur y est centré
     */
    std::vector<cv::Point3f> obj_points = {
        {-marker_length / 2.0f,  marker_length / 2.0f, 0.0f},
        { marker_length / 2.0f,  marker_length / 2.0f, 0.0f},
        { marker_length / 2.0f, -marker_length / 2.0f, 0.0f},
        {-marker_length / 2.0f, -marker_length / 2.0f, 0.0f}
    };

    std::vector<cv::Vec3d> good_rvecs, good_tvecs;

    std::cout << "\n[EXTR] Press 's' to save averaged pose, 'q' to quit." << std::endl;

    while (true)
    {
        cv::Mat frame;
        cap >> frame;
        if (frame.empty()) {
            std::cerr << "[EXTR] Empty frame, stopping." << std::endl;
            break;
        }

        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        detector.detectMarkers(gray, corners, ids);

        if (!ids.empty()) {
            cv::aruco::drawDetectedMarkers(frame, corners, ids);

            for (size_t i = 0; i < ids.size(); ++i) {
                cv::Vec3d rvec, tvec;
                bool ok = cv::solvePnP(
                    obj_points,
                    corners[i],
                    camera_matrix,
                    dist_coeffs,
                    rvec,
                    tvec,
                    false,
                    cv::SOLVEPNP_IPPE_SQUARE
                );

                if (!ok) continue;

                /* Erreur de reprojection */
                std::vector<cv::Point2f> projected;
                cv::projectPoints(obj_points, rvec, tvec,
                                  camera_matrix, dist_coeffs,
                                  projected);

                double err = 0.0;
                for (int k = 0; k < 4; k++) {
                    double dx = projected[k].x - corners[i][k].x;
                    double dy = projected[k].y - corners[i][k].y;
                    err += std::sqrt(dx * dx + dy * dy);
                }
                err /= 4.0;

                cv::drawFrameAxes(frame, camera_matrix, dist_coeffs,
                                  rvec, tvec, marker_length * 0.5f);

                if (err < threshold_px) {
                    good_rvecs.push_back(rvec);
                    good_tvecs.push_back(tvec);
                }

                char txt[128];

                snprintf(txt, sizeof(txt), "reproj err: %.2f px", err);
                cv::putText(frame, txt, cv::Point(15, 35),
                            cv::FONT_HERSHEY_SIMPLEX, 0.7,
                            cv::Scalar(0, 255, 0), 2);

                snprintf(txt, sizeof(txt), "t(mm): %.1f %.1f %.1f",
                         tvec[0] * 1000.0,
                         tvec[1] * 1000.0,
                         tvec[2] * 1000.0);
                cv::putText(frame, txt, cv::Point(15, 65),
                            cv::FONT_HERSHEY_SIMPLEX, 0.6,
                            cv::Scalar(200, 200, 0), 2);

                snprintf(txt, sizeof(txt), "good samples: %zu", good_tvecs.size());
                cv::putText(frame, txt, cv::Point(15, 95),
                            cv::FONT_HERSHEY_SIMPLEX, 0.6,
                            cv::Scalar(0, 255, 0), 2);

                std::cout << "[EXTR] ID " << ids[i]
                          << " | reproj=" << err << " px"
                          << " | t(mm)="
                          << tvec[0] * 1000.0 << " "
                          << tvec[1] * 1000.0 << " "
                          << tvec[2] * 1000.0
                          << std::endl;
            }
        } else {
            cv::putText(frame, "No marker detected", cv::Point(15, 35),
                        cv::FONT_HERSHEY_SIMPLEX, 0.7,
                        cv::Scalar(0, 0, 255), 2);
        }

        cv::imshow("ArUco Extrinsics", frame);

        int key = cv::waitKey(1);
        if (key == 'q' || key == 'Q') {
            break;
        }

        if (key == 's' || key == 'S') {
            if (good_tvecs.empty()) {
                std::cout << "[EXTR] WARN: no good samples yet." << std::endl;
                continue;
            }

            cv::Vec3d mean_tvec(0, 0, 0);
            cv::Vec3d mean_rvec(0, 0, 0);

            for (size_t k = 0; k < good_tvecs.size(); k++) {
                mean_tvec += good_tvecs[k];
                mean_rvec += good_rvecs[k];
            }

            mean_tvec /= (double)good_tvecs.size();
            mean_rvec /= (double)good_rvecs.size();

            cv::Mat R;
            cv::Rodrigues(mean_rvec, R);

            cv::FileStorage out(output_yaml, cv::FileStorage::WRITE);
            out << "R" << R;
            out << "t" << cv::Mat(mean_tvec);
            out.release();

            std::cout << "\n[EXTR][SAVED] " << good_tvecs.size()
                      << " samples -> " << output_yaml << std::endl;
            std::cout << "[EXTR] mean t(mm): "
                      << mean_tvec[0] * 1000.0 << " "
                      << mean_tvec[1] * 1000.0 << " "
                      << mean_tvec[2] * 1000.0 << std::endl;

            cv::putText(frame, "SAVED!", cv::Point(15, 125),
                        cv::FONT_HERSHEY_SIMPLEX, 1.2,
                        cv::Scalar(0, 255, 0), 3);
            cv::imshow("ArUco Extrinsics", frame);
            cv::waitKey(800);
        }
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}