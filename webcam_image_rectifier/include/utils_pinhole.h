#ifndef UTILS_H
#define UTILs_H

#include <cmath>
#include <cstdio>
#include <iomanip>

//ROS
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

//Eigen
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>

//OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>

namespace PINHOLE
{

   struct parameters
   {
      std::string camera_name;
      int imageWidth;
      int imageHeight;
      double k1;
      double k2;
      double p1;
      double p2;
      double fx;
      double fy;
      double cx;
      double cy;
   };

   //this function computes the undistortion and rectification transformation map.
   cv::Mat initUndistortRectifyMap(cv::Mat &map1, cv::Mat &map2, cv::Mat rmat, parameters Param);
   //%output K, map1, map2

   bool readFromYamlFile(const std::string &filename, parameters &Param);

   bool readParamFromCameraInfo(sensor_msgs::CameraInfo info_cam, parameters &param, double resolution_multiplier);

   bool loadIntrinsicFromYAML(const std::string filename, sensor_msgs::CameraInfo &info_cam, parameters &param, double res_multiplier);

   // Projects 3D points to the image plane (Pi function)
   void spaceToPlane(const Eigen::Vector3d &P, Eigen::Vector2d &p, parameters Param);
   //%output p

   //function to undistort a vector of points
   void rectifyImagePoints(const std::vector<cv::Point2f> &src, std::vector<cv::Point2f> &dst, parameters Param);

   //function to undistort a point
   void rectifyImagePoint(const cv::Point2f &src, cv::Point2f &dst, parameters Param);

   // Lift points from the image plane to the projective space
   void liftProjective(const Eigen::Vector2d &p, Eigen::Vector3d &P, parameters Param);
   //%output P

   //opencv way
   /*
bool loadIntrinsicFromYAML(const std::string filename, sensor_msgs::CameraInfo &info_cam, cv::Mat& M, cv::Mat& D, cv::Mat& R, cv::Mat& P);

bool loadIntrinsicFromXML(const std::string filename, sensor_msgs::CameraInfo &info_cam, cv::Mat& M, cv::Mat& D);

bool loadExtrinsicFromYAML(const std::string filename, sensor_msgs::CameraInfo &info_caml, sensor_msgs::CameraInfo &info_camr, cv::Mat& R1_cv, cv::Mat& R2_cv);

bool loadExtrinsicFromXML(const std::string filename, sensor_msgs::CameraInfo &info_caml, sensor_msgs::CameraInfo &info_camr, cv::Mat& R1, cv::Mat& P1, cv::Mat& R2, cv::Mat& P2);
*/
}

#endif
