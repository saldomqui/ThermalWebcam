/************************************************************************
 * by Salvador Dominguez (thorvald Robotics Ltc)
 *
 ************************************************************************/
#include "webcam_image_rectifier.h"


/** \brief the main function of the program
 *
 */
int main(int argc, char **argv)
{

  ros::init(argc, argv, "webcam_image_rectifier");

  ros::NodeHandle local_nh("~");
  ros::NodeHandle global_nh("");

  webcam_image_rectifier::WebcamImageRectifier *img_rec = new webcam_image_rectifier::WebcamImageRectifier(global_nh, local_nh);

  // Listen for topics
  ros::spin();

  delete img_rec;

  return 0;
}
