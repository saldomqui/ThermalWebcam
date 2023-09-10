
/** @file

    @brief recitification of fisheye camera images
*/

/************************************************************************
 * by Salvador Dominguez (Saga Robotics Ltc)
 *
 ************************************************************************/

#include <sys/stat.h>

#include <cv_bridge/cv_bridge.h>

#include <image_transport/camera_subscriber.h>
#include <image_transport/image_transport.h>

#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_srvs/Trigger.h>

// ROS topics synchronization
#include <message_filters/subscriber.h>

#include <image_geometry/pinhole_camera_model.h>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// Eigen
#include <eigen3/Eigen/Dense>

#include "utils_pinhole.h"
#include "utils_kannala.h"
#include "utils_mei.h"

//#include <SDL2/SDL.h>
//#include <SDL2/SDL_image.h>

#include "yaml-cpp/yaml.h"

// #include "spline.h"
#include "map_interpolator.h"

using namespace std;
// using namespace cv;
using namespace message_filters;
using namespace sensor_msgs;

namespace webcam_image_rectifier
{
  class WebcamImageRectifier
  {
  public:
    WebcamImageRectifier(ros::NodeHandle global_nh, ros::NodeHandle local_nh);
    ~WebcamImageRectifier();

  private:
    // Subscribers
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    ros::Subscriber req_cam_name_sub_;

    // Publishers
    ros::Publisher cam_name_pub_;
    image_transport::Publisher img_rectified_pub_;

    // Global variables
    string calibration_folder_; // In calibration mode, where to save the camera calibration file

    string camera_name_;
    string camera_selected_;

    // for undistort images
    cv::Mat map_rect1_, map_rect2_;

    cv::Mat image_rectified_;

    void publishRectifiedImage(const cv::Mat &img, ros::Time tstamp, string frame_id);
    void imgCallback(const sensor_msgs::ImageConstPtr &image_msg);
    void reqCameraNameCallback(const std_msgs::Empty msg);
    void selectCamCallback(const std_msgs::StringConstPtr &msg_sel);
  protected:
  };
};
