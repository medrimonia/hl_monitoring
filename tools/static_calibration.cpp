/**
 * This program open a video stream and a file describing the intrinsic
 * parameters of the camera. It uses manual input to estimate the pose of the
 * camera and then draw the field inside the image for the rest of the video.
 *
 * User interface notes:
 * - TODO
 */

#include <hl_monitoring/replay_image_provider.h>

#include <tclap/CmdLine.h>

using namespace hl_monitoring;

class StaticCalibrationTool {
public:
  StaticCalibrationTool(std::unique_ptr<ImageProvider> provider)
    : image_provider(std::move(provider)), is_good(true)
    {
      //TODO:
      // - Set cursor on first frame
      // - import points from 'Field' object
      // - Set callback for click
    }

  bool isGood() {
    return is_good;
  }

  // Main loop
  void update() {
    //TODO
    // - Get current image
    // - Draw manual input points
    // - If pose is available: draw field
    // - Wait for key (30 ms)
    // - Handle key
    //   - q/esc: set status to ko
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
  std::unique_ptr<ImageProvider> image_provider;

  std::vector<cv::Point3f> points_in_world;

  /**
   * The points manually tagged in the image associated with the index of the point in
   * 'points_in_world' vector
   */
  std::map<int,cv::Point2f> point_in_img;

  /**
   * Index of the next point to click on
   */
  int point_index;

  Pose3D estimated_pose;

  bool is_good;
};

int main(int argc, char ** argv) {
  TCLAP::CmdLine cmd("Extract the pose of the camera inside a video using pre-computed intrinsic"
                     "parameters", ' ', "0.9");

  TCLAP::ValueArg<std::string> video_arg("v", "video", "The path to the video", true,
                                         "video.avi", "string");
  TCLAP::ValueArg<std::string> meta_arg("m","meta-information",
                                        "The path to meta information of the video",
                                        true, "video_meta_information.bin", "string");
  TCLAP::ValueArg<std::string> output_arg("o","output",
                                          "The output path for meta-information of the video",
                                          true, "output.bin", "string");
  cmd.add(video_arg);
  cmd.add(meta_arg);
  cmd.add(output_arg);
  
  try {
    cmd.parse(argc, argv);
    
  } catch (const TCLAP::ArgException & e) {
    std::cerr << "error: " << e.error() << " for arg " << e.argId() << std::endl;
  }

  std::unique_ptr<ImageProvider> provider(new ReplayImageProvider(video_arg.getValue(), meta_arg.getValue()));

  StaticCalibrationTool calib_tool(std::move(provider));

  while(calib_tool.isGood()) {
    calib_tool.update();
  }
  calib_tool.saveMetaInformation(output_arg.getValue());
}
