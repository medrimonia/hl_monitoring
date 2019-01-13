/**
 * This program open a video stream and a file describing the intrinsic
 * parameters of the camera. It uses manual input to estimate the pose of the
 * camera and then draw the field inside the image for the rest of the video.
 *
 * User interface notes:
 * - TODO
 */

#include <hl_monitoring/field.h>
#include <hl_monitoring/replay_image_provider.h>
#include <hl_monitoring/utils.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <tclap/CmdLine.h>

#include <fstream>

using namespace hl_monitoring;

class StaticCalibrationTool {
public:
  StaticCalibrationTool(const cv::Mat & img,
                        std::unique_ptr<Field> field,
                        const IntrinsicParameters & camera_parameters)
    : calib_img(img), is_good(true), point_index(0)
    {
      intrinsicToCV(camera_parameters, &camera_matrix, &distortion_coefficients, &img_size);

      for (const auto & entry : field->getPointsOfInterest()) {
        std::cout << "Adding point '" << entry.first << "' -> " << entry.second << std::endl;
        points_names.push_back(entry.first);
        points_in_world.push_back(entry.second);
      }

      cv::namedWindow("display");
      
      cv::setMouseCallback("display",
                           [](int event, int x, int y, int, void *param) -> void {
                             StaticCalibrationTool * tool = (StaticCalibrationTool *)param;
                             tool->onClick(event, x, y, param);
                           }, this);
      printTagRequest();
    }

  bool isGood() {
    return is_good;
  }

  void onClick(int event, int x, int y, void * param) {
    if (event != cv::EVENT_LBUTTONDOWN) {
      return;
    }
    if (point_index >= (int) points_in_world.size()) {
      return;
    }
    points_in_img[point_index] = cv::Point(x,y);
    if (points_in_img.size() >= 4) {
      cv::Mat tvec, rvec;
      std::vector<cv::Point3f> object_points;
      std::vector<cv::Point2f> img_points;
      for (const auto & entry : points_in_img) {
        object_points.push_back(points_in_world[entry.first]);
        img_points.push_back(entry.second);
      }
      std::cout  << "camera_matrix.size" << camera_matrix.size() << std::endl;
      cv::solvePnP(object_points, img_points, camera_matrix, distortion_coefficients, rvec, tvec);
    }
    point_index++;
    printTagRequest();
  }

  void printTagRequest() {
    if (point_index < (int) points_in_world.size()) {
      std::cout << "Click on point '" << points_names[point_index] << "', pos in world: "
                << points_in_world[point_index] << std::endl;
    } else {
      std::cout << "All points have been tagged" << std::endl;
    }
  }

  // Main loop
  void update() {
    cv::Mat display_img = calib_img.clone();
    for (const auto & entry : points_in_img) {
      cv::circle(display_img, entry.second, 5, cv::Scalar(0,0,0), -1);
    }
    cv::imshow("display", display_img);
    char key = cv::waitKey(30);
    switch (key) {
      case 'q':
        is_good = false;
        break;
      case 'i':
        point_index++;
        printTagRequest();
    }
    //TODO
    // - Draw manual input points
    // - If pose is available: draw field
    // - Wait for key (30 ms)
    // - Handle key
    //   - q/esc: set is_good to false
    //   - backspace: remove last point in the list
  }

  void onClick() {
    // - Add point
    // - Update index
    // - If enough points, update pose
  }

  void saveMetaInformation(const std::string & path) {
    // TODO
  }

private:
  std::unique_ptr<Field> field;

  cv::Mat calib_img;
  
  cv::Mat camera_matrix;
  cv::Mat distortion_coefficients;
  cv::Size img_size;

  cv::Mat rvec;
  cv::Mat tvec;

  std::vector<std::string> points_names;

  /**
   * The points of interst
   */
  std::vector<cv::Point3f> points_in_world;

  /**
   * The points manually tagged in the image associated with the index of the point in
   * 'points_in_world' vector
   */
  std::map<int,cv::Point> points_in_img;

  /**
   * Index of the next point to click on
   */
  int point_index;

  bool is_good;
};

int main(int argc, char ** argv) {
  TCLAP::CmdLine cmd("Extract the pose of the camera inside a video using pre-computed intrinsic"
                     "parameters", ' ', "0.9");

  TCLAP::ValueArg<std::string> video_arg("v", "video", "The path to the video", true,
                                         "video.avi", "string");
  TCLAP::ValueArg<std::string> output_arg("o","output",
                                          "The output path for meta-information of the video",
                                          true, "output.bin", "string");
  TCLAP::ValueArg<std::string> intrinsic_arg("i","intrinsic",
                                             "Path to the file containing the intrinsic parameters",
                                             true, "intrinsic.bin", "string");
  TCLAP::ValueArg<std::string> field_arg("f","field",
                                         "The path to the field file containing the dimensions",
                                         true, "field.json", "string");
  cmd.add(video_arg);
  cmd.add(output_arg);
  cmd.add(intrinsic_arg);
  cmd.add(field_arg);

  try {
    cmd.parse(argc, argv);
    
  } catch (const TCLAP::ArgException & e) {
    std::cerr << "error: " << e.error() << " for arg " << e.argId() << std::endl;
  }

  ReplayImageProvider provider(video_arg.getValue());
  
  IntrinsicParameters intrinsic;
  std::ifstream in(intrinsic_arg.getValue());
  if (!in.good()) {
    throw std::runtime_error(HL_MONITOR_DEBUG + " failed to open file '"
                             + intrinsic_arg.getValue() + "'");
  }
  intrinsic.ParseFromIstream(&in);

  std::unique_ptr<Field> field(new Field());
  field->loadFile(field_arg.getValue());

  StaticCalibrationTool calib_tool(provider.getNextImg(), std::move(field), intrinsic);

  while(calib_tool.isGood()) {
    calib_tool.update();
  }
  calib_tool.saveMetaInformation(output_arg.getValue());
}
