/************************************************************************
 * by Salvador Dominguez (Saga Robotics Ltc)
 *
 ************************************************************************/
#include <boost/thread.hpp>

#include <csignal>

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include "driver_rtsp_cam.h"

// For configuration file
#include "yaml-cpp/yaml.h"

typedef struct config_
{
    int capture_fps;
    float capture_delay;
} config;

namespace rtsp_cam_driver
{
    class DriverRtspCamNodelet : public nodelet::Nodelet
    {
    public:
        rtsp_cam_driver::DriverRtspCam *dvr_rtsp_cam;

        DriverRtspCamNodelet()
        {
            // this_object = this;
        }
        ~DriverRtspCamNodelet()
        {
            NODELET_INFO("DriverRtspCamNodelet destructor. Releasing cam");
            dvr_rtsp_cam->release();
            sleep(2);
            deviceThread_->join();
            delete dvr_rtsp_cam;
        }

    private:
        volatile bool running_;
        boost::shared_ptr<boost::thread> deviceThread_;
        ros::NodeHandle node;
        ros::NodeHandle priv_nh;
        config cfg;
        string frame_id;

        // static DriverRtspCamNodelet *this_object;
        virtual void onInit()
        {
            string config_file;

            node = getNodeHandle();
            priv_nh = getPrivateNodeHandle();

            NODELET_INFO("Initialising ROS rtsp_cam_driver_node:");

            priv_nh.getParam("config_file", config_file);
            NODELET_INFO("config_file: %s", config_file.c_str());
            if (!loadConfigFile(config_file, cfg))
            {
                NODELET_INFO("Can't load configuration parameters");
                exit(-1);
            }

            priv_nh.getParam("frame_id", frame_id);
            cout << "frame_id:" << frame_id << endl;

            dvr_rtsp_cam = new rtsp_cam_driver::DriverRtspCam(node, priv_nh, cfg.capture_fps, frame_id);

            deviceThread_ = boost::shared_ptr<boost::thread>(new boost::thread(boost::bind(&DriverRtspCamNodelet::rtspCaptureThread, this)));
        }

        static void sigintHandler(int signum)
        {
            printf("rtsp_cam_driver catched Ctrl+C signal");
            // this_object->dvr_rtsp_cam->release();
        }

        void rtspCaptureThread()
        {
            ros::Rate rate(50);

            running_ = true;

            signal(SIGTERM, DriverRtspCamNodelet::sigintHandler);

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
            NODELET_INFO("Exiting rtsp cam driver thread");
            dvr_rtsp_cam->release();
            delete dvr_rtsp_cam;
            running_ = false;
        }

        bool loadConfigFile(const string &file, config &cfg)
        {
            NODELET_INFO("rtsp_cam_driver: Loading config file:");
            YAML::Node doc = YAML::LoadFile(file.c_str());

            try
            {
                cfg.capture_fps = doc["capture_fps"].as<int>();
                NODELET_INFO("capture_fps:%d", cfg.capture_fps);
            }
            catch (YAML::InvalidScalar)
            {
                cout << "Calibration file does not contain a tag for capture_fps value or it is invalid." << endl;
                return false;
            }
            try
            {
                cfg.capture_delay = doc["capture_delay"].as<float>();
                NODELET_INFO("capture_delay:%f", cfg.capture_delay);
            }
            catch (YAML::InvalidScalar)
            {
                cout << "Calibration file does not contain a tag for capture_delay value or it is invalid." << endl;
                return false;
            }

            return true;
        }
    };

} // namespace

// Register this plugin with pluginlib. Names must match nodelet.xml
//
// parameters : class type, base clase type
PLUGINLIB_EXPORT_CLASS(rtsp_cam_driver::DriverRtspCamNodelet, nodelet::Nodelet)
