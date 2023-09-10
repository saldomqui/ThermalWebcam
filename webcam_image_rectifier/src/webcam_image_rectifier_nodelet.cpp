/************************************************************************
 * by Salvador Dominguez (thorvald Robotics Ltc)
 *
 ************************************************************************/
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include "webcam_image_rectifier.h"

namespace webcam_image_rectifier
{
  class WebcamImageRectifierNodelet : public nodelet::Nodelet
  {
  public:
    webcam_image_rectifier::WebcamImageRectifier *img_rec;

    WebcamImageRectifierNodelet()
    {
    }
    ~WebcamImageRectifierNodelet()
    {
      delete img_rec;
    }

  private:
    ros::NodeHandle node;
    ros::NodeHandle priv_nh;

    virtual void onInit()
    {
      node = getNodeHandle();
      priv_nh = getPrivateNodeHandle();

      img_rec = new webcam_image_rectifier::WebcamImageRectifier(node, priv_nh);
    }
  };
}

// Register this plugin with pluginlib. Names must match nodelet.xml
//
// parameters : class type, base clase type
PLUGINLIB_EXPORT_CLASS(webcam_image_rectifier::WebcamImageRectifierNodelet, nodelet::Nodelet)
