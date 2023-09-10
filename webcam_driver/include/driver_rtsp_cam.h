
/** @file

    @brief ROS driver interface for rtsp cameras
*/

/************************************************************************
 * by Salvador Dominguez (Saga Robotics Ltc)
 *
 ************************************************************************/

#include <stdio.h>
#include <signal.h>

#include <ros/ros.h>

// #include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>

#include <std_srvs/Trigger.h>

// OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

using namespace std;

namespace rtsp_cam_driver
{
  class DriverRtspCam
  {
  public:
    DriverRtspCam(ros::NodeHandle nh, ros::NodeHandle priv_nh, int cap_fps, string frame_id);
    ~DriverRtspCam();

    bool grab_image(void);
    bool retrieve_image(cv::Mat &img);
    bool capture_image(cv::Mat &img);
    void publishImage(const ros::Time &tstamp, const cv::Mat &img);
    void release(void);
    // bool init(void);
  private:
    ros::NodeHandle nh_;      // node handle
    ros::NodeHandle priv_nh_; // private node handle

    // boost::shared_ptr<image_transport::ImageTransport> it_;
    ros::Publisher image_pub_;

    string camera_frame_;

    cv::VideoCapture *cam_; // RGB Camera object

    string rtsp_resource_;
    string image_raw_topic_;

    ros::Time time_last_image_captured_;

  protected:
  };
};
