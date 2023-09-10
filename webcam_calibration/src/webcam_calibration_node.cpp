/************************************************************************
 * by Salvador Dominguez (Saga Robotics Ltc)
 *
 ************************************************************************/
/**
\file    webcam_calibration.cpp
\brief   Tool to track objects by its temperature in the incoming pointcloud using thermal_scanner
\author  Salvador Dominguez
\date    21/09/2022
*/

//#include <vector>

// roscpp
#include "ros/ros.h"

#include "webcam_calibration.h"

using namespace std;

WebcamCalibration *wc;//A pointer to the WebcamCalibration class

int main(int argc, char** argv)
{
  ros::init(argc, argv, "webcam_calibration_node");

  wc=new WebcamCalibration();

  ros::Time::init();

  ros::spin();

  delete(wc);

  return(0);
}
