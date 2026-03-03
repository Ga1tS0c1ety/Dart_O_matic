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
                  << " <camera_id> <calib_yaml> <output_yaml> [marker_length_m]" << std::endl;
        std::cerr << "Example: " << argv[0]
                  << " 0 camera0_calib.yaml extrinsics_cam0.yaml 0.18" << std::endl;
        return -1;
    }

    int         cameraId     = std::stoi(argv[1]);
    std::string calibYaml    = argv[2];
    std::string outputYaml   = argv[3];
    float       markerLength = (argc > 4) ? std::stof(argv[4]) : 0.18f;
    
    double threshold = (argc > 5) ? std::stod(argv[5]) : 0.5;

    // Load intrinsics
    cv::FileStorage fs(calibYaml, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        std::cerr << "Error: cannot open " << calibYaml << std::endl;
        return -1;
    }

    cv::Mat cameraMatrix, distCoeffs;
    fs["camera_matrix"]          >> cameraMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    fs.release();

    if (cameraMatrix.empty() || distCoeffs.empty()) {
        std::cerr << "Error: camera params not found in " << calibYaml << std::endl;
        return -1;
    }

    std::cout << "Intrinsics loaded from " << calibYaml << std::endl;
    std::cout << "Camera ID  : " << cameraId     << std::endl;
    std::cout << "Output     : " << outputYaml   << std::endl;
    std::cout << "Marker len : " << markerLength << " m" << std::endl;

    // ArUco setup
    cv::aruco::Dictionary       dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_50);
    cv::aruco::DetectorParameters parameters;
    cv::aruco::ArucoDetector    detector(dictionary, parameters);

    // Open camera
    cv::VideoCapture cap(cameraId, cv::CAP_V4L2);
    if (!cap.isOpened()) {
        std::cerr << "Error: cannot open camera " << cameraId << std::endl;
        return -1;
    }

    cap.set(cv::CAP_PROP_FOURCC,      cv::VideoWriter::fourcc('M','J','P','G'));
    cap.set(cv::CAP_PROP_FRAME_WIDTH,  1280);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720);
    cap.set(cv::CAP_PROP_AUTOFOCUS,    0);
    cap.set(cv::CAP_PROP_FOCUS,        10);

    // 3D marker points (origin = center of marker = center of dartboard)
    std::vector<cv::Point3f> objPoints = {
        {-markerLength / 2,  markerLength / 2, 0},
        { markerLength / 2,  markerLength / 2, 0},
        { markerLength / 2, -markerLength / 2, 0},
        {-markerLength / 2, -markerLength / 2, 0}
    };
    
    
    cv::Vec3d best_rvec, best_tvec;
    bool      has_result = false;

    std::cout << "\nPress 's' to save when marker is stable, 'q' to quit" << std::endl;
	
	std::vector<cv::Vec3d> good_rvecs, good_tvecs;
	
    while (true)
    {
        cv::Mat frame;
        cap >> frame;
        if (frame.empty()) break;

        // Detect on grayscale only (no double detection)
        cv::Mat gray;
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);

        std::vector<int>                      ids;
        std::vector<std::vector<cv::Point2f>> corners;
        detector.detectMarkers(gray, corners, ids);

        if (!ids.empty())
        {
            cv::aruco::drawDetectedMarkers(frame, corners, ids);

            for (size_t i = 0; i < ids.size(); ++i)
            {
                cv::Vec3d rvec, tvec;
                bool ok = cv::solvePnP(
                    objPoints, corners[i],
                    cameraMatrix, distCoeffs,
                    rvec, tvec,
                    false, cv::SOLVEPNP_IPPE_SQUARE
                );

                if (ok)
				{
					best_rvec  = rvec;
					best_tvec  = tvec;
					has_result = true;

					// Reprojection error ? calcul EN PREMIER
					std::vector<cv::Point2f> projected;
					cv::projectPoints(objPoints, rvec, tvec,
									  cameraMatrix, distCoeffs, projected);
					double err = 0;
					for (int k = 0; k < 4; k++) {
						double dx = projected[k].x - corners[i][k].x;
						double dy = projected[k].y - corners[i][k].y;
						err += std::sqrt(dx*dx + dy*dy);
					}
					err /= 4.0;

					cv::drawFrameAxes(frame, cameraMatrix, distCoeffs,
									  rvec, tvec, markerLength * 0.5f);

					// Accumulation des bonnes mesures
					if (err < threshold) {
						good_rvecs.push_back(rvec);
						good_tvecs.push_back(tvec);
					}

					cv::Mat R;
					cv::Rodrigues(rvec, R);

					// Display
					char txt[64];
					snprintf(txt, sizeof(txt), "reproj err: %.2f px", err);
					cv::putText(frame, txt, cv::Point(15, 35),
								cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 255, 0), 2);

					snprintf(txt, sizeof(txt), "t: %.1f %.1f %.1f mm",
							 tvec[0]*1000, tvec[1]*1000, tvec[2]*1000);
					cv::putText(frame, txt, cv::Point(15, 65),
								cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(200, 200, 0), 2);

					snprintf(txt, sizeof(txt), "Good samples: %zu", good_tvecs.size());
					cv::putText(frame, txt, cv::Point(15, 95),
								cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);

					std::cout << "ID " << ids[i]
							  << " | reproj: " << err << " px"
							  << " | t(mm): "
							  << tvec[0]*1000 << " "
							  << tvec[1]*1000 << " "
							  << tvec[2]*1000 << std::endl;
				}
            }
        }
        else
        {
            cv::putText(frame, "No marker detected", cv::Point(15, 35),
                        cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);
        }
        

        cv::imshow("ArUco Extrinsics - Cam " + std::to_string(cameraId), frame);

        int key = cv::waitKey(1);
        if (key == 'q') break;
        if (key == 's')
		{
			if (good_tvecs.empty()) {
				std::cout << "[WARN] No good samples yet (reproj < 0.5px)" << std::endl;
			} else {
				cv::Vec3d mean_tvec(0,0,0), mean_rvec(0,0,0);
				for (size_t k = 0; k < good_tvecs.size(); k++) {
					mean_tvec += good_tvecs[k];
					mean_rvec += good_rvecs[k];
				}
				mean_tvec /= (double)good_tvecs.size();
				mean_rvec /= (double)good_rvecs.size();

				cv::Mat R;
				cv::Rodrigues(mean_rvec, R);

				cv::FileStorage out(outputYaml, cv::FileStorage::WRITE);
				out << "R" << R;
				out << "t" << cv::Mat(mean_tvec);
				out.release();

				std::cout << "\n[SAVED] " << good_tvecs.size() << " samples -> " << outputYaml << std::endl;
				std::cout << "t (mm): "
						  << mean_tvec[0]*1000 << " "
						  << mean_tvec[1]*1000 << " "
						  << mean_tvec[2]*1000 << std::endl;

				cv::putText(frame, "SAVED!", cv::Point(15, 125),
							cv::FONT_HERSHEY_SIMPLEX, 1.2, cv::Scalar(0,255,0), 3);
				cv::imshow("ArUco Extrinsics - Cam " + std::to_string(cameraId), frame);
				cv::waitKey(800);
			}
		}
    }

    cap.release();
    cv::destroyAllWindows();
    return 0;
}
