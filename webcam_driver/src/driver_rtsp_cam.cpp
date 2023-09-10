/************************************************************************
 * by Salvador Dominguez (Saga Robotics Ltc)
 *
 ************************************************************************/

#include "driver_rtsp_cam.h"

#include <chrono>

#define CAPTURE_TIMEOUT 4.0

using std::chrono::duration;
using std::chrono::duration_cast;
using std::chrono::high_resolution_clock;
using std::chrono::milliseconds;

#define errExit(msg)        \
    do                      \
    {                       \
        perror(msg);        \
        exit(EXIT_FAILURE); \
    } while (0)

namespace rtsp_cam_driver
{

    DriverRtspCam::DriverRtspCam(ros::NodeHandle nh,
                                 ros::NodeHandle priv_nh,
                                 int cap_fps,
                                 string frame_id) : nh_(nh),
                                                  priv_nh_(priv_nh),
                                                  camera_frame_(frame_id),
                                                  rtsp_resource_(""),
                                                  time_last_image_captured_(ros::Time::now())

    {
        priv_nh_.getParam("rtsp_resource", rtsp_resource_);
        cout << "webcam_driver: rtsp_resource:" << rtsp_resource_ << endl;

        priv_nh_.getParam("image_raw_topic", image_raw_topic_);
        cout << "webcam_driver: image_raw_topic:" << image_raw_topic_ << endl;

        cam_ = new cv::VideoCapture(rtsp_resource_);
        cam_->set(cv::CAP_PROP_FPS, double(cap_fps));

        image_pub_ = nh_.advertise<sensor_msgs::Image>(image_raw_topic_, 1);
    }

    DriverRtspCam::~DriverRtspCam()
    {
        cout << "webcam_driver: releasing rtsp camera" << endl;
        cam_->release();
        delete cam_;
    }

    void DriverRtspCam::release(void)
    {
        cam_->release();
        // delete cam_;
    }
    /*
        bool DriverRtspCam::init(void)
        {
            cout << "setting up rtsp_cam cam video stream" << endl;

            if (!cam_->isOpened())
            {
                cout << "Failed to open camera." << endl;
                return false;
            }

            return true;
        }
    */
    bool DriverRtspCam::capture_image(cv::Mat &img)
    {
        // auto t1 = high_resolution_clock::now();

        if (!cam_->isOpened())
            cam_->open(rtsp_resource_);

        if (cam_->isOpened())
        {
            if (!cam_->read(img))
            {
                cout << "webcam_driver: Camera read image error" << endl;
                return false;
            }
        }

        // auto t2 = high_resolution_clock::now();

        /* Getting number of milliseconds as a double. */
        // duration<double, std::milli> ms_double = t2 - t1;

        // std::cout << "read image: " << ms_double.count() << "ms\n";
        return true;
    }

    bool DriverRtspCam::retrieve_image(cv::Mat &img)
    {
        // auto t1 = high_resolution_clock::now();

        if (!cam_->isOpened())
            cam_->open(rtsp_resource_);
        if (!cam_->retrieve(img))
        {
            cout << "webcam_driver: Camera retrieve image error" << endl;
            return false;
        }

        // auto t2 = high_resolution_clock::now();

        /* Getting number of milliseconds as a double. */
        // duration<double, std::milli> ms_double = t2 - t1;

        // std::cout << "retrieve image: " << ms_double.count() << "ms\n";

        return true;
    }

    bool DriverRtspCam::grab_image(void)
    {
        // auto t1 = high_resolution_clock::now();

        if (!cam_->isOpened())
            cam_->open(rtsp_resource_);

        if (!cam_->grab())
        {
            cout << "webcam_driver: Camera grab image error" << endl;
            return false;
        }

        // auto t2 = high_resolution_clock::now();

        /* Getting number of milliseconds as a double. */
        // duration<double, std::milli> ms_double = t2 - t1;

        // std::cout << "grab image: " << ms_double.count() << "ms\n";

        time_last_image_captured_ = ros::Time::now();

        return true;
    }

    void DriverRtspCam::publishImage(const ros::Time &tstamp, const cv::Mat &img)
    {
        cv_bridge::CvImage out_img;

        out_img.image = img;
        out_img.header.stamp = tstamp;
        out_img.header.frame_id = camera_frame_;
        out_img.encoding = "bgr8";
        image_pub_.publish(out_img.toImageMsg());
    }

} // namespace
