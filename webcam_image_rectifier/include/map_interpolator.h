#ifndef MAP_INP_H
#define MAP_INP_H

#include <sstream> // std::stringstream

// OpenCV

#include <opencv2/opencv.hpp>

#include "spline.h"

using namespace cv;
using namespace std;

namespace MAP_INP
{
   typedef struct remapIp_
   {
      spline sp_obj_y;  
      spline sp_img_x;  
      spline sp_img_y;
   } remapIp;

   struct parameters
   {
      std::string camera_name;
      cv::Size imgSize;
      double horiz_cal_dist_mm;
      double vert_cal_dist_mm;
      vector<vector<cv::Point2f>> objpoints; // Points must be organized in rows of vectors of increasing y coordinates, each vector in increasing x coordinate
      vector<vector<cv::Point2f>> imgpoints;
   };

   // this function computes the undistortion and rectification transformation map.
   void initUndistortRectifyMap(parameters &params, cv::Mat &map1, cv::Mat &map2t);

   // function to undistort a vector of points
   void rectifyImagePoints(const std::vector<cv::Point2f> &src, std::vector<cv::Point2f> &dst, const cv::Mat &mapx, const cv::Mat &mapy);

   // function to undistort a point
   void rectifyImagePoint(const cv::Point2f &src, cv::Point2f &dst, const cv::Mat &mapx, const cv::Mat &mapy);

   void rectifyImage(const cv::Mat &img_distorted, cv::Mat &img_rectified, const cv::Mat &mapx, const cv::Mat &mapy);

   void buildInterpolatedModel(const vector<vector<cv::Point2f>> &objpts, const vector<vector<cv::Point2f>> &imgpts, vector<MAP_INP::remapIp> &inp_model);
   void computeYinterpolator(int x, const vector<MAP_INP::remapIp> &inp_model, MAP_INP::remapIp &vert_rmap);
   bool saveCalibrationMapsBinary(const string &path, const cv::Mat &mapx, const cv::Mat &mapy);
   bool doesFolderExists(const string &folder);
   bool loadCalibrationMapFromYAML(const string &path, parameters &params);
   bool loadCalibrationBinaryMap(const string &path, cv::Mat &mapx, cv::Mat &mapy);
}
#endif
