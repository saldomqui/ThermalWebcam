/************************************************************************
 * by Salvador Dominguez (Saga Robotics Ltc)
 *
 ************************************************************************/
#include "driver_rtsp_cam.h"

// For configuration file
#include "yaml-cpp/yaml.h"

typedef struct config_
{
  int capture_fps;
  float capture_delay;
} config;

rtsp_cam_driver::DriverRtspCam *dvr_rtsp_cam;

void sigsegv_handler(int sig)
{
  cout << "rtsp_cam_driver SIGTERM received" << endl;
  ros::shutdown(); // stop the main loop
}

bool loadConfigFile(const string &file, config &cfg)
{
  cout << "rtsp_cam_driver: Loading config file:" << file << endl;
  YAML::Node doc = YAML::LoadFile(file.c_str());

  try
  {
    cfg.capture_delay = doc["capture_delay"].as<float>();
    cout << "capture_delay:" << cfg.capture_delay << endl;
  }
  catch (YAML::InvalidScalar)
  {
    cout << "Calibration file does not contain a tag for capture_delay value or it is invalid." << endl;
    return false;
  }

  return true;
}

int main(int argc, char **argv)
{
  string config_file;
  config cfg;
  string frame_id;

  cout << "Initialising ROS rtsp_cam_driver_node:" << endl;

  signal(SIGTERM, &sigsegv_handler);

  ros::init(argc, argv, "rtsp_cam_driver_node");
  ros::NodeHandle node("");
  ros::NodeHandle priv_nh("~");

  priv_nh.getParam("config_file", config_file);
  cout << "config_file:" << config_file << endl;
  if (!loadConfigFile(config_file, cfg))
  {
    cout << "Can't load configuration parameters" << endl;
    exit(-1);
  }

  priv_nh.getParam("frame_id", frame_id);
  cout << "frame_id:" << frame_id << endl;

  dvr_rtsp_cam = new rtsp_cam_driver::DriverRtspCam(node, priv_nh, cfg.capture_fps, frame_id);

  ros::Rate rate(50);
  cv::Mat img;
  while (ros::ok())
  {
    ros::spinOnce();
    if (!dvr_rtsp_cam->grab_image())
    {
      delete dvr_rtsp_cam;

      cout << "rtsp_cam_driver: reopening capture device.." << endl;
      dvr_rtsp_cam = new rtsp_cam_driver::DriverRtspCam(node, priv_nh, cfg.capture_fps, frame_id);
      sleep(5);
    }
    else
    {
      if (dvr_rtsp_cam->retrieve_image(img))
        dvr_rtsp_cam->publishImage(ros::Time::now() - ros::Duration(cfg.capture_delay), img);
    }

    rate.sleep();
  }
  // ros::spin();
  ros::waitForShutdown();

  delete dvr_rtsp_cam;

  cout << "rtsp_cam_driver exiting..." << endl;
  return 0;
}
