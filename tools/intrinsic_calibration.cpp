/**
 * This program open a video stream of files containing calibration patterns and
 * output a protobuf file containing the intrinsic parameters of the camera
 * estimated from the images.
 *
 */

#include <hl_monitoring/replay_image_provider.h>
#include <hl_monitoring/utils.h>


#include <opencv2/opencv.hpp>
#include <tclap/CmdLine.h>

#include <iostream>

using namespace hl_monitoring;

// This calibration method is highly inspired from:
// https://docs.opencv.org/3.2.0/dc/dbb/tutorial_py_calibration.html
int main(int argc, char ** argv) {
  TCLAP::CmdLine cmd("Extract the pose of the camera inside a video using pre-computed intrinsic"
                     "parameters", ' ', "0.9");

  TCLAP::ValueArg<std::string> video_arg("v", "video", "The path to the video", true,
                                         "video.avi", "string");
  TCLAP::ValueArg<std::string> output_arg("o", "output", "The path to the output file", true,
                                          "intrinsic_parameters.bin", "string");
  TCLAP::ValueArg<float> frequency_arg("f", "frequency",
                                       "Maximal frequency at which images are shown",
                                       false, 1.0, "float");
  TCLAP::SwitchArg show_switch("s", "show",
                                 "Show images used for calibration and after calibration",
                                 cmd, false);

  cmd.add(video_arg);
  cmd.add(output_arg);
  cmd.add(frequency_arg);

  double sleep_time = 1000 / frequency_arg.getValue();
  
  try {
    cmd.parse(argc, argv);
    
  } catch (const TCLAP::ArgException & e) {
    std::cerr << "error: " << e.error() << " for arg " << e.argId() << std::endl;
  }

  ReplayImageProvider image_provider(video_arg.getValue());

  std::vector<std::vector<cv::Point3f>> objPoints;
  std::vector<std::vector<cv::Point2f>> imgPoints;
  cv::Size img_size;

  int successCount = 0;
  int imageCount = 0;
  while (!image_provider.isStreamFinished()) {
    cv::Mat img = image_provider.getNextImg();

    img_size = img.size();
    
    cv::Mat gray;
    cv::cvtColor(img, gray, CV_BGR2GRAY);

    std::vector<cv::Point2f> corners;
    cv::Size patternSize(9,6);
    bool success = cv::findChessboardCorners(gray, patternSize, corners);

    if (!success) {
      std::cerr << "failed to find corners" << std::endl;
      if (show_switch.getValue()) {
        cv::imshow("test", img);
        cv::waitKey(sleep_time);
      }
    } else {
      successCount++;
      if (show_switch.getValue()) {
        cv::drawChessboardCorners(img, patternSize, corners, success);
        cv::imshow("test", img);
        cv::waitKey(sleep_time);
      }

      std::vector<cv::Point3f> currentObjPoints;
      double markerSize = 0.05;
      for (int row = 0; row < patternSize.height; row++) {
        for (int col = 0; col < patternSize.width; col++) {
          currentObjPoints.push_back(cv::Point3f(row * markerSize, col * markerSize, 0));
        }
      }
      objPoints.push_back(currentObjPoints);
      imgPoints.push_back(corners);
    }
    imageCount++;
  }

  std::cout << "Success: " << successCount << "/" << imageCount << std::endl;

  // Calibration

  cv::Mat camera_matrix;
  cv::Mat distortion_coeffs;
  std::vector<cv::Mat> rvecs, tvecs;

  double error = cv::calibrateCamera(objPoints, imgPoints, img_size,
                                     camera_matrix, distortion_coeffs, rvecs, tvecs);

  std::cout << "Error: " << error << std::endl;
  std::cout << "Camera Matrix: " << camera_matrix << std::endl;
  std::cout << "Distortion Coeffs: " << distortion_coeffs << std::endl;

  if (show_switch.getValue()) {
    cv::Mat optimalMatrix;
    optimalMatrix = cv::getOptimalNewCameraMatrix(camera_matrix, distortion_coeffs, img_size, 0.0);
    cv::Mat undistorded;
    image_provider.restartStream();
    while (!image_provider.isStreamFinished()) {
      cv::Mat img = image_provider.getNextImg();
      // Undistort
      cv::undistort(img, undistorded, camera_matrix, distortion_coeffs);//, optimalMatrix);
      cv::imshow("img", img);
      cv::imshow("undistorded", undistorded);
      cv::waitKey(sleep_time);
    }
  }

  IntrinsicParameters result;
  cvToIntrinsic(camera_matrix, distortion_coeffs, img_size, &result);
  std::ofstream out(output_arg.getValue());
  if (!out.good()) {
    throw std::runtime_error(HL_MONITOR_DEBUG + " failed to open file '"
                             + output_arg.getValue() + "'");
  }
  if (!result.SerializeToOstream(&out)) {
    throw std::runtime_error(HL_MONITOR_DEBUG + " failed to write in file '"
                             + output_arg.getValue() + "'");
  }
}