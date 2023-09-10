#include <fstream>
#include <sys/stat.h>

#include <sstream> // std::stringstream

// For file map metadata reading
// #include "yaml-cpp/yaml.h"

#include <opencv2/opencv.hpp>
// #include <opencv2/calib3d/calib3d.hpp>

#include "webcam_calibration.h"

#define MIN_DIST_X_SEARCH_HORIZ 10
#define MAX_DIST_Y_SEARCH_HORIZ 30

#define MIN_DIST_Y_SEARCH_VERT 10
#define MAX_DIST_X_SEARCH_VERT 30

#define DEG_2_RAD (M_PI / 180.0)

#define MIN_CLUSTER_NUM_PTS 5

#define CROSS_CHECK_DIST 15

// WebcamCalibration class constructor
WebcamCalibration::WebcamCalibration(void) : local_nh_("~"),
                                             global_nh_(""),
                                             threshold_val_(127),
                                             it_(global_nh_),
                                             // cal_params_.horizCalDist(180.0),
                                             // cal_params_.vertCalDist(140.0),
                                             horiz_dist_incr_mm_(10.0),
                                             vert_dist_incr_mm_(10.0),
                                             calibrated_(false),
                                             process_each_(1)
{
  bool calibrate = true;

  local_nh_.getParam("process_each", process_each_); // Minimum snapshot period between consecutive snapshots
  cout << "saga_fisheyecam_calibration: process_each:" << process_each_ << endl;

  local_nh_.getParam("threshold_val", threshold_val_); // Minimum snapshot period between consecutive snapshots
  cout << "saga_fisheyecam_calibration: threshold_val:" << threshold_val_ << endl;

  local_nh_.getParam("horiz_calib_dist_mm", cal_params_.horizCalDist);
  cout << "saga_fisheyecam_calibration: horiz_calib_dist_mm:" << cal_params_.horizCalDist << endl;

  local_nh_.getParam("vert_calib_dist_mm", cal_params_.vertCalDist);
  cout << "saga_fisheyecam_calibration: vert_calib_dist_mm:" << cal_params_.vertCalDist << endl;

  local_nh_.getParam("horiz_ang_incr", horiz_dist_incr_mm_);
  cout << "saga_fisheyecam_calibration: horiz_ang_incr:" << horiz_dist_incr_mm_ << endl;

  local_nh_.getParam("vert_dist_incr_mm", vert_dist_incr_mm_);
  cout << "saga_fisheyecam_calibration: vert_dist_incr_mm:" << vert_dist_incr_mm_ << endl;

  local_nh_.getParam("calibration_folder", calibration_folder_);
  cout << "saga_fisheyecam_calibration: calibration_folder:" << calibration_folder_ << endl;

  local_nh_.getParam("calibrate", calibrate);
  cout << "saga_fisheyecam_calibration: calibrate:" << calibrate << endl;

  calibrated_ = !calibrate;
  if (calibrated_)
  {
    if (MAP_INP::loadCalibrationMapFromYAML(calibration_folder_, cal_params_))
    {
      MAP_INP::initUndistortRectifyMap(cal_params_, map1_, map2_);
      findCenterPoint(cal_params_.imgpoints);
    }
  }

  // Publishers
  debug_img_pub_ = it_.advertise("debug_img", 1);
  binary_img_pub_ = it_.advertise("binary_img", 1);
  grey_img_pub_ = it_.advertise("grey_img", 1);

  // snaphost_ready_pub = global_nh_.advertise<saga_web_snapshots_diagnostics::SnapshotParams>("/pano_view_snapshot_ready", 1);

  // Subscribers
  image_sub_ = it_.subscribe("image_in", 1, &WebcamCalibration::imgCallback, this);

  // req_cam_name_sub = global_nh_.subscribe<std_msgs::Empty>("/request_camera_name", 1, &WebcamCalibration::reqCameraNameCallback, this);
  //  temp_range_sub_ = global_nh_.subscribe<flir_thermal_client::TempMeasRange>("temp_meas_range", 1, &WebcamCalibration::tempRangeCallback, this);

  // Service servers
  // update_snapshot_service = global_nh_.advertiseService("thermal_tracking/update_snapshot", &WebcamCalibration::updateSnapshotCallback, this);
  calibrate_camera_service_ = global_nh_.advertiseService("calibrate_camera", &WebcamCalibration::calibrateCameraCallback, this);
}

// Destructor
WebcamCalibration::~WebcamCalibration()
{
  ros::waitForShutdown();
}

bool WebcamCalibration::calibrateCameraCallback(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
  cout << "saga_fisheyecam_calibration: Requested camera calibration" << endl;

  if (cal_params_.objpoints.size() && (cal_params_.objpoints.size() == cal_params_.imgpoints.size()))
  {
    for (int j = 0; j < cal_params_.objpoints.size(); j++)
    {
      cout << "row: i -->" << j << " objpts:" << cal_params_.objpoints[j].size() << " imgpts:" << cal_params_.imgpoints[j].size() << endl;
      for (int i = 0; i < cal_params_.objpoints[j].size(); i++)
      {
        cout << "img-(" << cal_params_.imgpoints[j][i].x << "," << cal_params_.imgpoints[j][i].y << "), obj-(" << cal_params_.objpoints[j][i].x << "," << cal_params_.objpoints[j][i].y << ")" << endl;
      }
    }

    // computeRemap(cal_params_.imgSize, cal_params_.objpoints, cal_params_.imgpoints, map1_, map2_);
    MAP_INP::initUndistortRectifyMap(cal_params_, map1_, map2_);
    MAP_INP::saveCalibrationMapToYAMLFile(calibration_folder_, cal_params_);
    MAP_INP::loadCalibrationMapFromYAML(calibration_folder_, cal_params_);
    // loadCalibrationMapFromYAML();
    calibrated_ = true;
  }
  else
    calibrated_ = false;

  return true;
}

void WebcamCalibration::imgCallback(const sensor_msgs::ImageConstPtr &img_msg)
{
  cv::Mat binary_img1;
  cv::Mat binary_img2;
  cv_bridge::CvImageConstPtr cv_ptr;
  cv::Mat grey_img;
  cv::Mat debug_img;
  vector<cv::Point> white_pts;
  static int count = 0;
  vector<cv::Point> pattern;
  vector<cv::Point> centroids;

  // cout << "webcam_calibration: img comming" << endl;
  if (!(count % process_each_))
  {
    // Retrieve image data and store it on a cv::Mat struct
    cv_ptr = cv_bridge::toCvShare(img_msg, sensor_msgs::image_encodings::RGB8);

    cal_params_.imgSize = cvSize(cv_ptr->image.cols, cv_ptr->image.rows);

    cv::cvtColor(cv_ptr->image, grey_img, cv::COLOR_BGR2GRAY);
    publishImage(grey_img_pub_, grey_img, img_msg->header.stamp, img_msg->header.frame_id, "mono8");

    // Binarize thermal data by applying some thresholds
    cv::threshold(grey_img, binary_img1, static_cast<unsigned char>(threshold_val_), 255, cv::THRESH_BINARY);
    // publishImage(binary_img_pub_, binary_img1, img_msg->header.stamp, img_msg->header.frame_id, "mono8");

    findControlPoints(binary_img1, binary_img2);
    publishImage(binary_img_pub_, binary_img2, img_msg->header.stamp, img_msg->header.frame_id, "mono8");

    cv::findNonZero(binary_img2, white_pts);

    debug_img = cv::Mat::zeros(cv_ptr->image.rows, cv_ptr->image.cols * 2, CV_8UC3);
    cv_ptr->image.copyTo(debug_img(cv::Rect(0, 0, cv_ptr->image.cols, cv_ptr->image.rows)));

    cv::line(debug_img, cv::Point(1, 0), cv::Point(1, debug_img.rows - 1), cv::Scalar(100, 100, 100), 2, cv::LINE_8);
    cv::line(debug_img, cv::Point(binary_img2.cols, 0), cv::Point(binary_img2.cols, debug_img.rows - 1), cv::Scalar(100, 100, 100), 2, cv::LINE_8);
    cv::line(debug_img, cv::Point(debug_img.cols - 3, 0), cv::Point(debug_img.cols - 3, debug_img.rows - 1), cv::Scalar(100, 100, 100), 2, cv::LINE_8);
    if (!calibrated_)
    {
      vector<vector<cv::Point>> clusters;

      computeCentroids(white_pts, centroids, clusters);
      drawCentroids(debug_img, centroids, cv::Scalar(0, 255, 0), 20);

      std::vector<std::pair<cv::Point, cv::Point>> corresp_pts;

      findCorrespondences(centroids, corresp_pts);
      orderCorrespondences(corresp_pts);

      //      cv_ptr->image.copyTo(debug_img(cv::Rect(cv_ptr->image.cols, 0, cv_ptr->image.cols, cv_ptr->image.rows)));
    }
    else
    {
      cv::Mat img_rectified;

      rectifyImage(cv_ptr->image, img_rectified);
      img_rectified.copyTo(debug_img(cv::Rect(cv_ptr->image.cols, 0, cv_ptr->image.cols, cv_ptr->image.rows)));
    }

    drawCheckPoints(debug_img, cv::Scalar(255, 0, 0), 20);
    drawCorrespondences(debug_img);
    publishImage(debug_img_pub_, debug_img, img_msg->header.stamp, img_msg->header.frame_id, "rgb8");
  }
  count++;
}

void WebcamCalibration::drawCorrespondences(const cv::Mat &img)
{
  for (int j = 0; j < cal_params_.objpoints.size(); j++)
  {
    for (int i = 0; i < cal_params_.objpoints[j].size(); i++)
    {
      cv::Scalar color(
          (double)std::rand() / RAND_MAX * 255,
          (double)std::rand() / RAND_MAX * 255,
          (double)std::rand() / RAND_MAX * 255);
      cv::line(img, cal_params_.imgpoints[j][i], cv::Point(cal_params_.objpoints[j][i].x + cal_params_.imgSize.width, cal_params_.objpoints[j][i].y), color, 1, cv::LINE_AA);
    }
  }
}

void WebcamCalibration::findCorrespondences(const vector<cv::Point> &distorted_points, std::vector<std::pair<cv::Point, cv::Point>> &corresp)
{
  int x_grid_idx = 0, y_grid_idx = 0;
  std::pair<cv::Point, cv::Point> corresp_pair;
  std::vector<bool> marked;

  marked.resize(distorted_points.size());
  for (int m = 0; m < marked.size(); m++)
    marked[m] = false;

  int center_idx = findCenterPoint(distorted_points);
  int middle_idx = center_idx;

  while (middle_idx >= 0)
  {
    x_grid_idx = 0;

    corresp_pair.first = distorted_points[middle_idx];
    corresp_pair.second = cv::Point((cal_params_.imgSize.width / 2) + (x_grid_idx * horiz_dist_incr_mm_ * cal_params_.imgSize.width) / cal_params_.horizCalDist, (cal_params_.imgSize.height / 2) - (y_grid_idx * vert_dist_incr_mm_ * cal_params_.imgSize.height) / cal_params_.vertCalDist);
    corresp.push_back(corresp_pair);
    marked[middle_idx] = true;

    int idx = findLeft(distorted_points, distorted_points[middle_idx], marked);
    while (idx >= 0)
    {
      x_grid_idx--;
      corresp_pair.first = distorted_points[idx];
      corresp_pair.second = cv::Point((cal_params_.imgSize.width / 2) + (x_grid_idx * horiz_dist_incr_mm_ * cal_params_.imgSize.width) / cal_params_.horizCalDist, (cal_params_.imgSize.height / 2) - (y_grid_idx * vert_dist_incr_mm_ * cal_params_.imgSize.height) / cal_params_.vertCalDist);
      corresp.push_back(corresp_pair);
      marked[idx] = true;
      idx = findLeft(distorted_points, distorted_points[idx], marked);
    }

    x_grid_idx = 0;
    idx = findRight(distorted_points, distorted_points[middle_idx], marked);
    while (idx >= 0)
    {
      x_grid_idx++;
      corresp_pair.first = distorted_points[idx];
      corresp_pair.second = cv::Point((cal_params_.imgSize.width / 2) + (x_grid_idx * horiz_dist_incr_mm_ * cal_params_.imgSize.width) / cal_params_.horizCalDist, (cal_params_.imgSize.height / 2) - (y_grid_idx * vert_dist_incr_mm_ * cal_params_.imgSize.height) / cal_params_.vertCalDist);
      corresp.push_back(corresp_pair);
      marked[idx] = true;
      idx = findRight(distorted_points, distorted_points[idx], marked);
    }

    y_grid_idx++;

    middle_idx = findUp(distorted_points, distorted_points[middle_idx], marked);
  }

  x_grid_idx = 0;
  y_grid_idx = -1;
  middle_idx = findDown(distorted_points, distorted_points[center_idx], marked);

  while (middle_idx >= 0)
  {
    x_grid_idx = 0;

    corresp_pair.first = distorted_points[middle_idx];
    corresp_pair.second = cv::Point((cal_params_.imgSize.width / 2) + (x_grid_idx * horiz_dist_incr_mm_ * cal_params_.imgSize.width) / cal_params_.horizCalDist, (cal_params_.imgSize.height / 2) - (y_grid_idx * vert_dist_incr_mm_ * cal_params_.imgSize.height) / cal_params_.vertCalDist);
    corresp.push_back(corresp_pair);
    marked[middle_idx] = true;

    int idx = findLeft(distorted_points, distorted_points[middle_idx], marked);
    while (idx >= 0)
    {
      x_grid_idx--;
      corresp_pair.first = distorted_points[idx];
      corresp_pair.second = cv::Point((cal_params_.imgSize.width / 2) + (x_grid_idx * horiz_dist_incr_mm_ * cal_params_.imgSize.width) / cal_params_.horizCalDist, (cal_params_.imgSize.height / 2) - (y_grid_idx * vert_dist_incr_mm_ * cal_params_.imgSize.height) / cal_params_.vertCalDist);
      corresp.push_back(corresp_pair);
      marked[idx] = true;
      idx = findLeft(distorted_points, distorted_points[idx], marked);
    }

    x_grid_idx = 0;
    idx = findRight(distorted_points, distorted_points[middle_idx], marked);
    while (idx >= 0)
    {
      x_grid_idx++;
      corresp_pair.first = distorted_points[idx];
      corresp_pair.second = cv::Point((cal_params_.imgSize.width / 2) + (x_grid_idx * horiz_dist_incr_mm_ * cal_params_.imgSize.width) / cal_params_.horizCalDist, (cal_params_.imgSize.height / 2) - (y_grid_idx * vert_dist_incr_mm_ * cal_params_.imgSize.height) / cal_params_.vertCalDist);
      corresp.push_back(corresp_pair);
      marked[idx] = true;
      idx = findRight(distorted_points, distorted_points[idx], marked);
    }

    y_grid_idx--;
    middle_idx = findDown(distorted_points, distorted_points[middle_idx], marked);
  }
}

bool WebcamCalibration::findRow(int y, int &closest_row_idx)
{
  int min_y = 1E6;

  closest_row_idx = -1;

  for (int j = 0; j < cal_params_.objpoints.size(); j++)
  {
    if (int(cal_params_.objpoints[j][0].y) == y)
    {
      closest_row_idx = j;
      return true;
    }
    else if ((int(cal_params_.objpoints[j][0].y) > y) && (int(cal_params_.objpoints[j][0].y) < min_y))
    {
      min_y = int(cal_params_.objpoints[j][0].y);
      closest_row_idx = j;
    }
  }

  return false;
}

int WebcamCalibration::findClosestPtInRow(vector<cv::Point2f> &list, int x)
{
  int min_x = 1E6;
  int closest_pt_in_row_idx = -1;

  for (int i = 0; i < list.size(); i++)
  {
    if ((int(list[i].x) > x) && (int(list[i].x) < min_x))
    {
      min_x = int(list[i].x);
      closest_pt_in_row_idx = i;
    }
  }

  return closest_pt_in_row_idx;
}

void WebcamCalibration::orderCorrespondences(const std::vector<std::pair<cv::Point, cv::Point>> &corresp)
{

  cal_params_.imgpoints.clear();
  cal_params_.objpoints.clear();

  for (int i = 0; i < corresp.size(); i++)
  {
    int closest_row_idx;
    vector<cv::Point2f> pt2d_list;
    vector<cv::Point2f> pt3d_list;

    if (findRow(corresp[i].second.y, closest_row_idx)) // If the row for this y coordinate already exists in cal_params_.objpoints list
    {
      int closest_pt_in_row_idx = findClosestPtInRow(cal_params_.objpoints[closest_row_idx], corresp[i].second.x); // Find the point in the row that is larger but closer than the x value
      if (closest_pt_in_row_idx >= 0)                                                                              // Found a point that is larger. Insert point before it
      {
        cal_params_.objpoints[closest_row_idx].insert(cal_params_.objpoints[closest_row_idx].begin() + closest_pt_in_row_idx, cv::Point2f(float(corresp[i].second.x), float(corresp[i].second.y)));
        cal_params_.imgpoints[closest_row_idx].insert(cal_params_.imgpoints[closest_row_idx].begin() + closest_pt_in_row_idx, cv::Point2f(float(corresp[i].first.x), float(corresp[i].first.y)));
      }
      else // Not found any point int the row that is larger, so append point at the end
      {
        cal_params_.objpoints[closest_row_idx].push_back(cv::Point2f(float(corresp[i].second.x), float(corresp[i].second.y)));
        cal_params_.imgpoints[closest_row_idx].push_back(cv::Point2f(float(corresp[i].first.x), float(corresp[i].first.y)));
      }
    }
    else if (closest_row_idx >= 0) // Insert row before the closest found row
    {
      pt3d_list.push_back(cv::Point2f(float(corresp[i].second.x), float(corresp[i].second.y)));
      pt2d_list.push_back(cv::Point2f(float(corresp[i].first.x), float(corresp[i].first.y)));
      cal_params_.objpoints.insert(cal_params_.objpoints.begin() + closest_row_idx, pt3d_list);
      cal_params_.imgpoints.insert(cal_params_.imgpoints.begin() + closest_row_idx, pt2d_list);
    }
    else
    { // append row at the end
      pt3d_list.push_back(cv::Point2f(float(corresp[i].second.x), float(corresp[i].second.y)));
      pt2d_list.push_back(cv::Point2f(float(corresp[i].first.x), float(corresp[i].first.y)));
      cal_params_.objpoints.push_back(pt3d_list);
      cal_params_.imgpoints.push_back(pt2d_list);
    }
  }
}

int WebcamCalibration::findCenterPoint(const vector<cv::Point> &points)
{
  int idx = -1;
  double min_dist = 1E6;
  int left_most_idx = -1;
  int right_most_idx = -1;
  int x_max = 0;
  int x_min = cal_params_.imgSize.width - 1;
  int up_most_idx = -1;
  int down_most_idx = -1;
  int y_max = 0;
  int y_min = cal_params_.imgSize.height - 1;

  for (int i = 0; i < points.size(); i++)
  {
    if (points[i].x < x_min)
    {
      x_min = points[i].x;
      left_most_idx = i;
    }
    if (points[i].x > x_max)
    {
      x_max = points[i].x;
      right_most_idx = i;
    }
    if (points[i].y < y_min)
    {
      y_min = points[i].y;
      up_most_idx = i;
    }
    if (points[i].y > y_max)
    {
      y_max = points[i].y;
      down_most_idx = i;
    }
  }

  center_pt_ = cv::Point((x_max + x_min) / 2, (y_max + y_min) / 2);

  //cout << "theoretical mid point x:" << center_pt_.x << " y:" << center_pt_.y << endl;
  for (int i = 0; i < points.size(); i++)
  {
    double dist = sqrt(pow(points[i].x - center_pt_.x, 2.0) + pow(points[i].y - center_pt_.y, 2.0));

    if (dist < min_dist)
    {
      idx = i;
      min_dist = dist;
    }
  }

  center_pt_ = points[idx];
  //cout << "computed mid point x:" << center_pt_.x << " y:" << center_pt_.y << endl;
  return idx;
}

void WebcamCalibration::findCenterPoint(const vector<vector<cv::Point2f>> &points)
{
  int idx = -1;
  double min_dist = 1E6;
  cv::Point left_most_pt;
  cv::Point right_most_pt;
  cv::Point up_most_pt;
  cv::Point down_most_pt;
  int x_max = 0;
  int x_min = cal_params_.imgSize.width - 1;
  int y_max = 0;
  int y_min = cal_params_.imgSize.height - 1;

  for (int j = 0; j < points.size(); j++)
  {
    for (int i = 0; i < points[j].size(); i++)
    {
      if (points[j][i].x < x_min)
      {
        x_min = points[j][i].x;
        left_most_pt = points[j][i];
      }
      if (points[j][i].x > x_max)
      {
        x_max = points[j][i].x;
        right_most_pt = points[j][i];
      }
      if (points[j][i].y < y_min)
      {
        y_min = points[j][i].y;
        up_most_pt = points[j][i];
      }
      if (points[j][i].y > y_max)
      {
        y_max = points[j][i].y;
        down_most_pt = points[j][i];
      }
    }
  }

  cv::Point aux_center_pt_ = cv::Point(int((x_min + x_max) / 2), int((y_min + y_max) / 2));
  center_pt_ = aux_center_pt_;

  for (int j = 0; j < points.size(); j++)
  {
    for (int i = 0; i < points[j].size(); i++)
    {
      double dist = sqrt(pow(points[j][i].x - aux_center_pt_.x, 2.0) + pow(points[j][i].y - aux_center_pt_.y, 2.0));

      if (dist < min_dist)
      {
        center_pt_ = cv::Point(int(points[j][i].x), int(points[j][i].y));
        min_dist = dist;
      }
    }
  }
  return;
}

int WebcamCalibration::findLeft(const vector<cv::Point> &points, const cv::Point &pt, const std::vector<bool> &marked)
{
  int idx = -1;
  double min_dist = 1E6;

  for (int i = 0; i < points.size(); i++)
  {
    if (((pt.x - points[i].x) > MIN_DIST_X_SEARCH_HORIZ) && ((fabs(pt.y - points[i].y) < MAX_DIST_Y_SEARCH_HORIZ)) && (!marked[i]))
    {
      double dist = sqrt(pow(points[i].x - pt.x, 2.0) + pow(points[i].y - pt.y, 2.0));

      if (dist < min_dist)
      {
        idx = i;
        min_dist = dist;
      }
    }
  }
  return idx;
}

int WebcamCalibration::findRight(const vector<cv::Point> &points, const cv::Point &pt, const std::vector<bool> &marked)
{
  int idx = -1;
  double min_dist = 1E6;

  for (int i = 0; i < points.size(); i++)
  {
    if (((points[i].x - pt.x) > MIN_DIST_X_SEARCH_HORIZ) && ((fabs(pt.y - points[i].y) < MAX_DIST_Y_SEARCH_HORIZ)) && (!marked[i]))
    {
      double dist = sqrt(pow(points[i].x - pt.x, 2.0) + pow(points[i].y - pt.y, 2.0));

      if (dist < min_dist)
      {
        idx = i;
        min_dist = dist;
      }
    }
  }
  return idx;
}

int WebcamCalibration::findUp(const vector<cv::Point> &points, const cv::Point &pt, const std::vector<bool> &marked)
{
  int idx = -1;
  double min_dist = 1E6;

  for (int i = 0; i < points.size(); i++)
  {
    if ((fabs(pt.x - points[i].x) < MAX_DIST_X_SEARCH_VERT) && ((pt.y - points[i].y) > MIN_DIST_Y_SEARCH_VERT) && (!marked[i]))
    {
      double dist = sqrt(pow(points[i].x - pt.x, 2.0) + pow(points[i].y - pt.y, 2.0));

      if (dist < min_dist)
      {
        idx = i;
        min_dist = dist;
      }
    }
  }
  return idx;
}

int WebcamCalibration::findDown(const vector<cv::Point> &points, const cv::Point &pt, const std::vector<bool> &marked)
{
  int idx = -1;
  double min_dist = 1E6;

  for (int i = 0; i < points.size(); i++)
  {
    if ((fabs(pt.x - points[i].x) < MAX_DIST_X_SEARCH_VERT) && ((points[i].y - pt.y) > MIN_DIST_Y_SEARCH_VERT) && (!marked[i]))
    {
      double dist = sqrt(pow(points[i].x - pt.x, 2.0) + pow(points[i].y - pt.y, 2.0));

      if (dist < min_dist)
      {
        idx = i;
        min_dist = dist;
      }
    }
  }
  return idx;
}

void WebcamCalibration::rectifyImage(const cv::Mat &img_distorted, cv::Mat &img_rectified)
{
  cv::remap(img_distorted, img_rectified, map1_, map2_, cv::INTER_LINEAR);
}

void WebcamCalibration::computeCentroids(vector<cv::Point> pts, vector<cv::Point> &centr, vector<vector<cv::Point>> &clust)
{
  vector<int> labels;

  // Define the distance between clusters
  int euclidean_distance = 20;

  // With lambda function
  int th2 = euclidean_distance * euclidean_distance;
  int n_labels = cv::partition(pts, labels, [th2](const cv::Point &lhs, const cv::Point &rhs)
                               { return ((lhs.x - rhs.x) * (lhs.x - rhs.x) + (lhs.y - rhs.y) * (lhs.y - rhs.y)) < th2; });

  // cout << "labels:" << n_labels << " pts.size()" << pts.size() << endl;
  //  Store all points in same cluster, and compute centroids
  vector<cv::Point> centroids(n_labels, cv::Point(0, 0));
  vector<vector<cv::Point>> clusters(n_labels);

  for (int i = 0; i < pts.size(); ++i)
  {
    clusters[labels[i]].push_back(pts[i]);
    centroids[labels[i]] += pts[i];
  }

  for (int i = n_labels - 1; i >= 0; --i)
  {
    // cout << "cluster:" << i << " num_pts:" << clusters[i].size() << endl;
    if (clusters[i].size() < MIN_CLUSTER_NUM_PTS)
    {
      cout << "cluster:" << i << " doesn't have enough pts:" << clusters[i].size() << endl;
      centroids.erase(centroids.begin() + i);
      clusters.erase(clusters.begin() + i);
    }
    else
    {
      centroids[i].x /= clusters[i].size();
      centroids[i].y /= clusters[i].size();
    }
  }

  centr = centroids;
  clust = clusters;
}

void WebcamCalibration::drawCentroids(cv::Mat &img, const vector<cv::Point> &centroids, const cv::Scalar &color, int marker_size)
{
  for (int i = 0; i < centroids.size(); i++)
  {
    cv::Point pt1 = cv::Point(centroids[i].x - marker_size, centroids[i].y);
    cv::Point pt2 = cv::Point(centroids[i].x + marker_size, centroids[i].y);

    cv::line(img, pt1, pt2, color, 1, cv::LINE_AA);

    pt1 = cv::Point(centroids[i].x, centroids[i].y - marker_size);
    pt2 = cv::Point(centroids[i].x, centroids[i].y + marker_size);

    cv::line(img, pt1, pt2, color, 1, cv::LINE_AA);

    cv::circle(img, centroids[i], marker_size, color, 1, cv::LINE_AA);
  }
}

void WebcamCalibration::drawCheckPoints(cv::Mat &img, const cv::Scalar &color, int marker_size)
{
  for (int j = 0; j < cal_params_.objpoints.size(); j++)
  {
    for (int i = 0; i < cal_params_.objpoints[j].size(); i++)
    {
      cv::Point pt1 = cv::Point(cal_params_.objpoints[j][i].x - marker_size + cal_params_.imgSize.width, cal_params_.objpoints[j][i].y);
      cv::Point pt2 = cv::Point(cal_params_.objpoints[j][i].x + marker_size + cal_params_.imgSize.width, cal_params_.objpoints[j][i].y);

      cv::line(img, pt1, pt2, color, 1, cv::LINE_AA);

      pt1 = cv::Point(cal_params_.objpoints[j][i].x + cal_params_.imgSize.width, cal_params_.objpoints[j][i].y - marker_size);
      pt2 = cv::Point(cal_params_.objpoints[j][i].x + cal_params_.imgSize.width, cal_params_.objpoints[j][i].y + marker_size);

      cv::line(img, pt1, pt2, color, 1, cv::LINE_AA);

      // cv::circle(img, cal_params_.objpoints[j][i], 3, color, CV_FILLED);
    }
  }
  for (int j = 0; j < cal_params_.imgpoints.size(); j++)
  {
    for (int i = 0; i < cal_params_.imgpoints[j].size(); i++)
    {
      cv::Point pt1 = cv::Point(cal_params_.imgpoints[j][i].x - marker_size, cal_params_.imgpoints[j][i].y);
      cv::Point pt2 = cv::Point(cal_params_.imgpoints[j][i].x + marker_size, cal_params_.imgpoints[j][i].y);

      cv::line(img, pt1, pt2, color, 1, cv::LINE_AA);

      pt1 = cv::Point(cal_params_.imgpoints[j][i].x, cal_params_.imgpoints[j][i].y - marker_size);
      pt2 = cv::Point(cal_params_.imgpoints[j][i].x, cal_params_.imgpoints[j][i].y + marker_size);

      cv::line(img, pt1, pt2, color, 1, cv::LINE_AA);

      // cv::circle(img, cal_params_.imgpoints[j][i], 3, color, CV_FILLED);
    }
  }
  cv::circle(img, center_pt_, marker_size, cv::Scalar(255, 0, 0), 1);
}

void WebcamCalibration::publishImage(const image_transport::Publisher &pub, const cv::Mat &img, ros::Time tstamp, string frame_id, string encoding)
{
  cv_bridge::CvImage out_8b;
  out_8b.header.frame_id = frame_id;
  out_8b.header.stamp = tstamp;
  out_8b.encoding = encoding;
  out_8b.image = img;
  pub.publish(out_8b.toImageMsg());
}

void WebcamCalibration::findControlPoints(const cv::Mat &img_in, cv::Mat &img_out)
{
  img_out = cv::Mat::zeros(img_in.rows, img_in.cols, CV_8UC1);

  for (int j = 0; j < img_in.rows; j++)
  {
    for (int i = 0; i < img_in.cols; i++)
    {
      uint8_t px_val;
      int count_horiz = 0;
      int count_vert = 0;

      for (int k = -CROSS_CHECK_DIST; k < CROSS_CHECK_DIST; k++)
      {
        int j_k = j + k;

        if ((j_k >= 0) && (j_k < img_in.rows))
          if (img_in.at<uint8_t>(j_k, i) == 255)
            count_vert++;

        int i_k = i + k;

        if ((i_k >= 0) && (i_k < img_in.cols))
          if (img_in.at<uint8_t>(j, i_k) == 255)
            count_horiz++;
      }

      if ((count_vert >= CROSS_CHECK_DIST) && (count_horiz >= CROSS_CHECK_DIST))
        img_out.at<uint8_t>(j, i) = 255;
    }
  }
}