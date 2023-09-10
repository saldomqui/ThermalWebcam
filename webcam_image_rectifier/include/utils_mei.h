#ifndef UTILS_MEI_H
#define UTILS_MEI_H

#include <cmath>
#include <cstdio>

//Eigen
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>

//OpenCV
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

//ROS
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

/**
 * C. Mei, and P. Rives, Single View Point Omnidirectional Camera Calibration
 * from Planar Grids, ICRA 2007
 */

namespace MEI
{

   struct parameters
   {
      std::string camera_name;
      int imageWidth;
      int imageHeight;
      double xi;
      double k1;
      double k2;
      double p1;
      double p2;
      double gamma1;
      double gamma2;
      double u0;
      double v0;

      double fx;
      double fy;
      double cx;
      double cy;
   };

   //this function computes the undistortion and rectification transformation map.
   cv::Mat initUndistortRectifyMap(cv::Mat &map1, cv::Mat &map2, cv::Mat rmat, parameters Param);
   //%output K, map1, map2

   bool readFromYamlFile(const std::string &filename, parameters &Param, double resolution_multiplier);

   bool readParamFromCameraInfo(sensor_msgs::CameraInfo info_cam, parameters &param, double resolution_multiplier);

   bool loadIntrinsicFromYAML(const std::string filename, sensor_msgs::CameraInfo &info_cam, MEI::parameters &param, double resolution_multiplier);

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

}
#endif
