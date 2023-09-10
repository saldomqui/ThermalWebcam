#ifndef saga_fisheyecam_calibration_H
#define saga_fisheyecam_calibration_H

// roscpp
#include "ros/ros.h"

#include <std_srvs/Trigger.h>

#include <image_transport/camera_subscriber.h>
#include <image_transport/image_transport.h>

#include <sensor_msgs/Image.h>

#include <cv_bridge/cv_bridge.h>

#include "map_interpolator.h"

using namespace std;

/**
 * The WebcamCalibration class
 */

class WebcamCalibration
{
public:
  /** \fn WebcamCalibration()
   * \brief WebcamCalibration class constructor
   */
  WebcamCalibration(void);

  /** \fn ~WebcamCalibration()
   * \brief WebcamCalibration class destructor
   */
  ~WebcamCalibration();

  int threshold_val_;

private:
  ros::NodeHandle local_nh_; // local handle used to read local parameters
  ros::NodeHandle global_nh_;

  // Publishers
  image_transport::Publisher debug_img_pub_;
  image_transport::Publisher binary_img_pub_;
  image_transport::Publisher grey_img_pub_;

  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;

  // ros::Publisher snaphost_ready_pub;

  // Services
  ros::ServiceServer calibrate_camera_service_;

  // Subscribers
  double horiz_dist_incr_mm_;
  double vert_dist_incr_mm_;
  int process_each_;

  MAP_INP::parameters cal_params_;
  cv::Mat map1_, map2_;
  bool calibrated_;

  string calibration_folder_;

  cv::Point center_pt_;

  void imgCallback(const sensor_msgs::ImageConstPtr &img_msg);
  bool calibrateCameraCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);

  void publishImage(const image_transport::Publisher &pub, const cv::Mat &img, ros::Time tstamp, string frame_id, string encoding);
  void computeCentroids(vector<cv::Point> pts, vector<cv::Point> &centr, vector<vector<cv::Point>> &clust);
  void drawCentroids(cv::Mat &img, const vector<cv::Point> &centroids, const cv::Scalar &color, int marker_size);
  void drawCheckPoints(cv::Mat &img, const cv::Scalar &color, int marker_size);
  // void generateOutputPattern(const int img_width, const int img_height, vector<cv::Point> &pattern);
  void rectifyImage(const cv::Mat &img_distorted, cv::Mat &img_rectified);
  void findCorrespondences(const vector<cv::Point> &distorted_points, std::vector<std::pair<cv::Point, cv::Point>> &corresp);
  int findCenterPoint(const vector<cv::Point> &points);
  void findCenterPoint(const vector<vector<cv::Point2f>> &points);
  void drawCorrespondences(const cv::Mat &img);
  int findLeft(const vector<cv::Point> &points, const cv::Point &pt, const std::vector<bool> &marked);
  int findRight(const vector<cv::Point> &points, const cv::Point &pt, const std::vector<bool> &marked);
  int findUp(const vector<cv::Point> &points, const cv::Point &pt, const std::vector<bool> &marked);
  int findDown(const vector<cv::Point> &points, const cv::Point &pt, const std::vector<bool> &marked);
  void orderCorrespondences(const std::vector<std::pair<cv::Point, cv::Point>> &corresp);
  bool findRow(int y, int &closest_row_idx);
  int findClosestPtInRow(vector<cv::Point2f> &list, int x);
  void findControlPoints(const cv::Mat &img_in, cv::Mat &img_out);
};

#endif // saga_fisheyecam_calibration_H
