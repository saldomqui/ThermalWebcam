#ifndef UTILS_KANNALA_H
#define UTILS_KANNALA_H

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
 * J. Kannala, and S. Brandt, A Generic Camera Model and Calibration Method
 * for Conventional, Wide-Angle, and Fish-Eye Lenses, PAMI 2006
 */

namespace KANNALA
{

   struct parameters
   {
      std::string camera_name;
      int imageWidth;
      int imageHeight;
      double k2;
      double k3;
      double k4;
      double k5;
      double mu;
      double mv;
      double u0;
      double v0;

      double fx;
      double fy;
      double cx;
      double cy;
   };

   template <typename T>
   static T r(T k2, T k3, T k4, T k5, T theta);

   //this function computes the undistortion and rectification transformation map.
   cv::Mat initUndistortRectifyMap(cv::Mat &map1, cv::Mat &map2, cv::Mat rmat, parameters param);
   //%output K, map1, map2

   bool readFromYamlFile(const std::string &filename, parameters &param, double resolution_multiplier);

   bool readParamFromCameraInfo(sensor_msgs::CameraInfo info_cam, parameters &param, double resolution_multiplier);

   bool loadIntrinsicFromYAML(const std::string filename, sensor_msgs::CameraInfo &info_cam, parameters &param, double resolution_multiplier);

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

   void backprojectSymmetric(const Eigen::Vector2d &p_u, double &theta, double &phi, parameters Param);

}

#endif
