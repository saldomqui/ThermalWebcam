/************************************************************************
 * by Salvador Dominguez (Saga Robotics Ltc)
 *
 ************************************************************************/

#include "webcam_image_rectifier.h"

#define RAD_2_DEG (180.0 / M_PI)
#define DEG_2_RAD (M_PI / 180.0)

namespace webcam_image_rectifier
{
    WebcamImageRectifier::WebcamImageRectifier(ros::NodeHandle global_nh, ros::NodeHandle local_nh) : it_(global_nh),
                                                                                                      camera_name_("none"),
                                                                                                      camera_selected_("none")
    {
        MAP_INP::parameters params;

        local_nh.getParam("calibration_folder", calibration_folder_); // The path of the calibration file. If empty, calibration will be taken from topic
        cout << "webcam_image_rectifier: calibration_folder:" << calibration_folder_ << endl;

        local_nh.getParam("camera_name", camera_name_);
        cout << "webcam_image_rectifier: camera_name:" << camera_name_ << endl;

        if (MAP_INP::loadCalibrationMapFromYAML(calibration_folder_, params))
        {
            MAP_INP::initUndistortRectifyMap(params, map_rect1_, map_rect2_);
        }
        else
        {
            cout << "Can't load calibration interpolation calibration mapping " << endl;
            exit(0);
        }

        // Publishing
        img_rectified_pub_ = it_.advertise("image_out", 1);
        cam_name_pub_ = global_nh.advertise<std_msgs::String>("/camera_name", 1);

        image_sub_ = it_.subscribe("image_in", 1, &WebcamImageRectifier::imgCallback, this);
        req_cam_name_sub_ = global_nh.subscribe<std_msgs::Empty>("/request_camera_name", 1, &WebcamImageRectifier::reqCameraNameCallback, this);
    }

    WebcamImageRectifier::~WebcamImageRectifier()
    {
    }

    // Image processing callback
    /** \brief callback to catch the input images in ROS and process them
     *
     */
    void WebcamImageRectifier::imgCallback(const sensor_msgs::ImageConstPtr &image_msg)
    {
        // cout << "webcam_image_rectifier: image comming..."  << endl;
        ros::Time tstamp;

        // cout << "webcam_image_rectifier: processing image of camera:" << camera_name_ << endl;
        //  Get timestamp
        tstamp = image_msg->header.stamp;

        // get distorted img in OpenCV format
        cv_bridge::CvImageConstPtr cv_ptr;

        // cout << "webcam_image_rectifier: incomming img encoding --> " << image_msg->encoding << endl;
        if (!image_msg->encoding.compare("mono8"))
        {
            // cout << "webcam_image_rectifier: Incomming image in B/W" << endl;
            cv_ptr = cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::MONO8);
        }
        else
        {
            // cout << "webcam_image_rectifier: Incomming image in color" << endl;
            // cout << "webcam_image_rectifier: Converging color image to mono" << endl;
            cv_ptr = cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::BGR8);
        }

        remap(cv_ptr->image, image_rectified_, map_rect1_, map_rect2_, cv::INTER_LINEAR);
        //  cout << "img width:" << image_rectified_.cols << " height:" << image_rectified_.rows << endl;

        publishRectifiedImage(image_rectified_, image_msg->header.stamp, image_msg->header.frame_id);
    }

    void WebcamImageRectifier::publishRectifiedImage(const cv::Mat &img, ros::Time tstamp, string frame_id)
    {
        // Publish the image with features
        cv_bridge::CvImage out_msg;

        if (img.channels() == 1)
            out_msg.encoding = sensor_msgs::image_encodings::MONO8;
        else
            out_msg.encoding = sensor_msgs::image_encodings::BGR8;
        out_msg.image = img;
        out_msg.header.seq = 1;
        out_msg.header.frame_id = frame_id;
        out_msg.header.stamp = tstamp;
        img_rectified_pub_.publish(out_msg.toImageMsg());
    }

    void WebcamImageRectifier::reqCameraNameCallback(const std_msgs::Empty msg)
    {
        std_msgs::String msg_cam_name;

        cout << "webcam_image_rectifier: Requested camera name remotely. Camera name:" << camera_name_ << endl;

        msg_cam_name.data = camera_name_;

        cam_name_pub_.publish(msg_cam_name);
    }

    void WebcamImageRectifier::selectCamCallback(const std_msgs::StringConstPtr &msg_sel)
    {
        cout << "webcam_image_rectifier: selected camera:" << msg_sel->data << endl;
        camera_selected_ = msg_sel->data;
    }

} // namespace
