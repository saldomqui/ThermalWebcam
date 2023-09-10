#include "utils_pinhole.h"

//For configuration file
#include "yaml-cpp/yaml.h"

using namespace cv;
using namespace std;

namespace PINHOLE
{

    bool readFromYamlFile(const std::string &filename,
                          parameters &param, double resolution_multiplier)
    {
        std::cout << "Calibration file:" << filename << std::endl;

        YAML::Node doc = YAML::LoadFile(filename.c_str());

        param.camera_name = doc["camera_name"].as<string>();

        param.imageWidth = doc["image_width"].as<int>();
        param.imageHeight = doc["image_height"].as<int>();

        cout << "image_width:" << param.imageWidth << " image_height:" << param.imageHeight << endl;

        vector<double> D = doc["distortion_coefficients"]["data"].as<vector<double>>();
        param.k1 = static_cast<double>(D[0]) / pow(resolution_multiplier, 2.0);
        param.k2 = static_cast<double>(D[1]) / pow(resolution_multiplier, 2.0);
        param.p1 = static_cast<double>(D[2]);
        param.p2 = static_cast<double>(D[3]);
        cout << "k1:" << param.k1 << " k2:" << param.k2 << " p1:" << param.p1 << " p2:" << param.p2 << endl;

        vector<double> K = doc["camera_matrix"]["data"].as<vector<double>>();
        param.fx = resolution_multiplier * static_cast<double>(K[0]);
        param.fy = resolution_multiplier * static_cast<double>(K[4]);
        param.cx = resolution_multiplier * static_cast<double>(K[2]);
        param.cy = resolution_multiplier * static_cast<double>(K[5]);
        cout << "fx:" << param.fx << " fy:" << param.fy << " cx:" << param.cx << " cy:" << param.cy << endl;

        return true;
    }

    bool readParamFromCameraInfo(sensor_msgs::CameraInfo info_cam, parameters &param, double resolution_multiplier)
    {
        param.camera_name = info_cam.header.frame_id;

        param.imageWidth = info_cam.width;
        param.imageHeight = info_cam.height;
        cout << "image_width:" << param.imageWidth << " image_height:" << param.imageHeight << endl;

        param.fx = resolution_multiplier * info_cam.K[0];
        param.fy = resolution_multiplier * info_cam.K[4];
        param.cx = resolution_multiplier * info_cam.K[2];
        param.cy = resolution_multiplier * info_cam.K[5];
        cout << "fx:" << param.fx << " fy:" << param.fy << " cx:" << param.cx << " cy:" << param.cy << endl;

        param.k1 = info_cam.D[0] / pow(resolution_multiplier, 2.0);
        param.k2 = info_cam.D[1] / pow(resolution_multiplier, 4.0);
        param.p1 = info_cam.D[2];
        param.p2 = info_cam.D[3];
        cout << "k1:" << param.k1 << " k2:" << param.k2 << " p1:" << param.p1 << " p2:" << param.p2 << endl;

        return true;
    }

    bool loadIntrinsicFromYAML(const std::string filename, sensor_msgs::CameraInfo &info_cam, parameters &param, double res_multiplier)
    {
        if (!readFromYamlFile(filename, param, res_multiplier))
        {
            printf("Failed to open file %s\n", filename.c_str());
            return false;
        }

        //Distortion parameters
        info_cam.D.push_back(param.k1); //k1
        info_cam.D.push_back(param.k2); //k2
        info_cam.D.push_back(param.p1); //p1
        info_cam.D.push_back(param.p2); //p2
        info_cam.D.push_back(0);        //k3

        //Intrinsic camera matrix
        info_cam.K[0] = param.fx; //fx
        info_cam.K[1] = 0;
        info_cam.K[2] = param.cx; //cx
        info_cam.K[3] = 0;
        info_cam.K[4] = param.fy; //fy
        info_cam.K[5] = param.cy; //cy
        info_cam.K[6] = 0;
        info_cam.K[7] = 0;
        info_cam.K[8] = 1;

        info_cam.R[0] = 1.0;
        info_cam.R[1] = 0.0;
        info_cam.R[2] = 0.0;
        info_cam.R[3] = 0.0;
        info_cam.R[4] = 1.0;
        info_cam.R[5] = 0.0;
        info_cam.R[6] = 0.0;
        info_cam.R[7] = 0.0;
        info_cam.R[8] = 1.0;

        info_cam.distortion_model = "plumb_bob";

        info_cam.binning_x = 0;
        info_cam.binning_y = 0;
        info_cam.roi.x_offset = 0;
        info_cam.roi.y_offset = 0;
        info_cam.roi.do_rectify = false;

        info_cam.width = param.imageWidth;
        info_cam.height = param.imageHeight;

        info_cam.roi.width = info_cam.width;
        info_cam.roi.height = info_cam.height;

        //Projection camera matrix
        info_cam.P[0] = param.fx; //fx
        info_cam.P[1] = 0;
        info_cam.P[2] = param.cx; //cx
        info_cam.P[3] = 0;        //Tx
        info_cam.P[4] = 0;
        info_cam.P[5] = param.fy; //fy
        info_cam.P[6] = param.cy; //cy
        info_cam.P[7] = 0;        //Ty
        info_cam.P[8] = 0;
        info_cam.P[9] = 0;
        info_cam.P[10] = 1; //1.0
        info_cam.P[11] = 0;

        return true;
    }

    cv::Mat initUndistortRectifyMap(cv::Mat &map1, cv::Mat &map2, cv::Mat rmat,
                                    parameters param)
    {
        cv::Size imageSize(param.imageWidth, param.imageHeight);

        cv::Mat mapX = cv::Mat::zeros(imageSize.height, imageSize.width, CV_32F);
        cv::Mat mapY = cv::Mat::zeros(imageSize.height, imageSize.width, CV_32F);

        Eigen::Matrix3f K_rect;

        if (param.cx == -1.0f || param.cy == -1.0f)
        {
            K_rect << param.fx, 0, imageSize.width / 2,
                0, param.fy, imageSize.height / 2,
                0, 0, 1;
        }
        else
        {
            K_rect << param.fx, 0, param.cx,
                0, param.fy, param.cy,
                0, 0, 1;
        }

        if (param.fx == -1.0f || param.fy == -1.0f)
        {
            K_rect(0, 0) = param.fx;
            K_rect(1, 1) = param.fy;
        }

        Eigen::Matrix3f K_rect_inv = K_rect.inverse();

        Eigen::Matrix3f R, R_inv;
        cv::cv2eigen(rmat, R);
        R_inv = R.inverse();

        for (int v = 0; v < imageSize.height; ++v)
        {
            for (int u = 0; u < imageSize.width; ++u)
            {
                Eigen::Vector3f xo;
                xo << u, v, 1;

                Eigen::Vector3f uo = R_inv * K_rect_inv * xo;

                Eigen::Vector2d p;
                spaceToPlane(uo.cast<double>(), p, param);

                mapX.at<float>(v, u) = p(0);
                mapY.at<float>(v, u) = p(1);
            }
        }

        cv::convertMaps(mapX, mapY, map1, map2, CV_32FC1, false);

        cv::Mat K_rect_cv;
        cv::eigen2cv(K_rect, K_rect_cv);
        return K_rect_cv;
    }

    /**
 * \brief Apply distortion to input point (from the normalised plane)
 *
 * \param p_u undistorted coordinates of point on the normalised plane
 * \return to obtain the distorted point: p_d = p_u + d_u
 */
    void distortion(const Eigen::Vector2d &p_u, Eigen::Vector2d &d_u, parameters param)
    {
        double k1 = param.k1;
        double k2 = param.k2;
        double p1 = param.p1;
        double p2 = param.p2;

        double mx2_u, my2_u, mxy_u, rho2_u, rad_dist_u;

        mx2_u = p_u(0) * p_u(0);
        my2_u = p_u(1) * p_u(1);
        mxy_u = p_u(0) * p_u(1);
        rho2_u = mx2_u + my2_u;
        rad_dist_u = k1 * rho2_u + k2 * rho2_u * rho2_u;
        d_u << p_u(0) * rad_dist_u + 2.0 * p1 * mxy_u + p2 * (rho2_u + 2.0 * mx2_u),
            p_u(1) * rad_dist_u + 2.0 * p2 * mxy_u + p1 * (rho2_u + 2.0 * my2_u);
    }

    /**
 * \brief Project a 3D point (\a x,\a y,\a z) to the image plane in (\a u,\a v)
 *
 * \param P 3D point coordinates
 * \param p return value, contains the image point coordinates
 */
    void spaceToPlane(const Eigen::Vector3d &P, Eigen::Vector2d &p,
                      parameters param)
    {
        Eigen::Vector2d p_u, p_d;

        // Project points to the normalised plane
        p_u << P(0) / P(2), P(1) / P(2);

        {
            // Apply distortion
            Eigen::Vector2d d_u;
            distortion(p_u, d_u, param);
            p_d = p_u + d_u;
        }

        // Apply generalised projection matrix
        p << param.fx * p_d(0) + param.cx,
            param.fy * p_d(1) + param.cy;
    }

    /**
 * \brief Lifts a point from the image plane to its projective ray
 *
 * \param p image coordinates
 * \param P coordinates of the projective ray
 */
    void liftProjective(const Eigen::Vector2d &p, Eigen::Vector3d &P,
                        parameters param)
    {
        double m_inv_K11 = 1.0 / param.fx;
        double m_inv_K13 = -param.cx / param.fx;
        double m_inv_K22 = 1.0 / param.fy;
        double m_inv_K23 = -param.cy / param.fy;

        double mx_d, my_d, mx2_d, mxy_d, my2_d, mx_u, my_u;
        double rho2_d, rho4_d, radDist_d, Dx_d, Dy_d, inv_denom_d;
        double lambda;

        // Lift points to normalised plane
        mx_d = m_inv_K11 * p(0) + m_inv_K13;
        my_d = m_inv_K22 * p(1) + m_inv_K23;

        if (0)
        {
            double k1 = param.k1;
            double k2 = param.k2;
            double p1 = param.p1;
            double p2 = param.p2;

            // Apply inverse distortion model
            // proposed by Heikkila
            mx2_d = mx_d * mx_d;
            my2_d = my_d * my_d;
            mxy_d = mx_d * my_d;
            rho2_d = mx2_d + my2_d;
            rho4_d = rho2_d * rho2_d;
            radDist_d = k1 * rho2_d + k2 * rho4_d;
            Dx_d = mx_d * radDist_d + p2 * (rho2_d + 2 * mx2_d) + 2 * p1 * mxy_d;
            Dy_d = my_d * radDist_d + p1 * (rho2_d + 2 * my2_d) + 2 * p2 * mxy_d;
            inv_denom_d = 1 / (1 + 4 * k1 * rho2_d + 6 * k2 * rho4_d + 8 * p1 * my_d + 8 * p2 * mx_d);

            mx_u = mx_d - inv_denom_d * Dx_d;
            my_u = my_d - inv_denom_d * Dy_d;
        }
        else
        {
            // Recursive distortion model
            int n = 8;
            Eigen::Vector2d d_u;
            distortion(Eigen::Vector2d(mx_d, my_d), d_u, param);
            // Approximate value
            mx_u = mx_d - d_u(0);
            my_u = my_d - d_u(1);

            for (int i = 1; i < n; ++i)
            {
                distortion(Eigen::Vector2d(mx_u, my_u), d_u, param);
                mx_u = mx_d - d_u(0);
                my_u = my_d - d_u(1);
            }
        }

        // Obtain a projective ray
        P << mx_u, my_u, 1.0;
    }

    // undistortion function for a vector of points
    void rectifyImagePoints(const std::vector<cv::Point2f> &src, std::vector<cv::Point2f> &dst, parameters param)
    {
        dst.resize(src.size());

        for (size_t i = 0; i < src.size(); ++i)
        {
            const cv::Point2f &p = src.at(i);

            Eigen::Vector3d P;
            liftProjective(Eigen::Vector2d(p.x, p.y), P, param);

            P /= P(2);

            dst.at(i) = cv::Point2f(P(0) * param.fx + param.cx, P(1) * param.fy + param.cy);
        }
    }

    // undistortion function for a single point
    void rectifyImagePoint(const cv::Point2f &src, cv::Point2f &dst, parameters param)
    {
        Eigen::Vector3d P;

        liftProjective(Eigen::Vector2d(src.x, src.y), P, param);

        P /= P(2);

        dst.x = P(0) * param.fx + param.cx;
        dst.y = P(1) * param.fy + param.cy;
    }
}

/*
bool loadIntrinsicFromYAML(const std::string filename, sensor_msgs::CameraInfo &info_cam, cv::Mat& M, cv::Mat& D, cv::Mat& R, cv::Mat& P)
{
   cv::FileStorage fs(filename, cv::FileStorage::READ);
   double width,height;
   fs["K"] >> M;
   fs["D"] >> D;
   fs["R"] >> R;
   fs["P"] >> P;
   fs["width"] >> width;
   fs["height"] >> height;

   //Distortion parameters
   info_cam.D.push_back(D.at<float>(0));//k1
   info_cam.D.push_back(D.at<float>(1));//k2
   info_cam.D.push_back(D.at<float>(2));//t1
   info_cam.D.push_back(D.at<float>(3));//t2
   info_cam.D.push_back(D.at<float>(4));//k3

   //Intrinsic camera matrix
   info_cam.K[0]=M.at<float>(0,0);//fx
   info_cam.K[1]=M.at<float>(0,1);
   info_cam.K[2]=M.at<float>(0,2);//cx
   info_cam.K[3]=M.at<float>(1,0);
   info_cam.K[4]=M.at<float>(1,1);//fy
   info_cam.K[5]=M.at<float>(1,2);//cy
   info_cam.K[6]=M.at<float>(2,0);
   info_cam.K[7]=M.at<float>(2,1);
   info_cam.K[8]=M.at<float>(2,2);

   info_cam.distortion_model="plumb_bob";

   info_cam.binning_x=0;
   info_cam.binning_y=0;
   info_cam.roi.x_offset=0;
   info_cam.roi.y_offset=0;
   info_cam.roi.do_rectify=false;

   info_cam.width = width;
   info_cam.height = height;

   info_cam.roi.width=info_cam.width;
   info_cam.roi.height=info_cam.height;

   //Rectification matrix
   info_cam.R[0]=R.at<double>(0,0);
   info_cam.R[1]=R.at<double>(0,1);
   info_cam.R[2]=R.at<double>(0,2);
   info_cam.R[3]=R.at<double>(1,0);
   info_cam.R[4]=R.at<double>(1,1);
   info_cam.R[5]=R.at<double>(1,2);
   info_cam.R[6]=R.at<double>(2,0);
   info_cam.R[7]=R.at<double>(2,1);
   info_cam.R[8]=R.at<double>(2,2);

   info_cam.P[0]=P.at<double>(0,0);//fx
   info_cam.P[1]=P.at<double>(0,1);
   info_cam.P[2]=P.at<double>(0,2);//cx
   info_cam.P[3]=P.at<double>(0,3);//Tx
   info_cam.P[4]=P.at<double>(1,0);
   info_cam.P[5]=P.at<double>(1,1);//fy
   info_cam.P[6]=P.at<double>(1,2);//cy
   info_cam.P[7]=P.at<double>(1,3);//Ty
   info_cam.P[8]=P.at<double>(2,0);
   info_cam.P[9]=P.at<double>(2,1);
   info_cam.P[10]=P.at<double>(2,2);
   info_cam.P[11]=P.at<double>(2,3);//1.0

   fs.release();
   return true;
}

bool loadExtrinsicFromXML(const std::string filename, sensor_msgs::CameraInfo &info_caml, sensor_msgs::CameraInfo &info_camr, Mat& R1, Mat& P1, Mat& R2, Mat& P2)
{
   FileStorage fs;
   Mat R, T;
   fs.open(filename, CV_STORAGE_READ);
   if(!fs.isOpened())
   {
      printf("Failed to open file %s\n", filename.c_str());
      return false;
   }
   printf("open file %s\n", filename.c_str());
   fs["R_left"] >> R1;
   fs["P_left"] >> P1;
   fs["R_right"] >> R2;
   fs["P_right"] >> P2;
   fs["R"] >> R;
   fs["T"] >> T;

//    if(P2.at<double>(0,3)==0)
   P2.at<double>(0,3) = P2.at<double>(0,0)*T.at<double>(0,0);

   //Rectification matrix
   info_caml.R[0]=R1.at<double>(0,0);
   info_caml.R[1]=R1.at<double>(0,1);
   info_caml.R[2]=R1.at<double>(0,2);
   info_caml.R[3]=R1.at<double>(1,0);
   info_caml.R[4]=R1.at<double>(1,1);
   info_caml.R[5]=R1.at<double>(1,2);
   info_caml.R[6]=R1.at<double>(2,0);
   info_caml.R[7]=R1.at<double>(2,1);
   info_caml.R[8]=R1.at<double>(2,2);

   info_camr.R[0]=R2.at<double>(0,0);
   info_camr.R[1]=R2.at<double>(0,1);
   info_camr.R[2]=R2.at<double>(0,2);
   info_camr.R[3]=R2.at<double>(1,0);
   info_camr.R[4]=R2.at<double>(1,1);
   info_camr.R[5]=R2.at<double>(1,2);
   info_camr.R[6]=R2.at<double>(2,0);
   info_camr.R[7]=R2.at<double>(2,1);
   info_camr.R[8]=R2.at<double>(2,2);

   //Projection camera matrix
   info_caml.P[0]=P1.at<double>(0,0);//fx
   info_caml.P[1]=P1.at<double>(0,1);
   info_caml.P[2]=P1.at<double>(0,2);//cx
   info_caml.P[3]=P1.at<double>(0,3);//Tx
   info_caml.P[4]=P1.at<double>(1,0);
   info_caml.P[5]=P1.at<double>(1,1);//fy
   info_caml.P[6]=P1.at<double>(1,2);//cy
   info_caml.P[7]=P1.at<double>(1,3);//Ty
   info_caml.P[8]=P1.at<double>(2,0);
   info_caml.P[9]=P1.at<double>(2,1);
   info_caml.P[10]=P1.at<double>(2,2);
   info_caml.P[11]=P1.at<double>(2,3);//1.0

   info_camr.P[0]=P2.at<double>(0,0);//fx
   info_camr.P[1]=P2.at<double>(0,1);
   info_camr.P[2]=P2.at<double>(0,2);//cx
   info_camr.P[3]=P2.at<double>(0,3);//Tx
   info_camr.P[4]=P2.at<double>(1,0);
   info_camr.P[5]=P2.at<double>(1,1);//fy
   info_camr.P[6]=P2.at<double>(1,2);//cy
   info_camr.P[7]=P2.at<double>(1,3);//Ty
   info_camr.P[8]=P2.at<double>(2,0);
   info_camr.P[9]=P2.at<double>(2,1);
   info_camr.P[10]=P2.at<double>(2,2);
   info_camr.P[11]=P2.at<double>(2,3);//1.0

   fs.release();
   return true;
}

bool loadIntrinsicFromXML(const std::string filename, sensor_msgs::CameraInfo &info_cam, Mat& M1, Mat& D1)
{
   double width,height;
   FileStorage fs;
   fs.open(filename, CV_STORAGE_READ);
   if(!fs.isOpened())
   {
      printf("Failed to open file %s\n", filename.c_str());
      return false;
   }
   printf("open file %s\n", filename.c_str());

   fs["M"] >> M1;
   fs["D"] >> D1;
   fs["width"] >> width;
   fs["height"] >> height;

   //Distortion parameters
   info_cam.D.push_back(D1.at<float>(0));//k1
   info_cam.D.push_back(D1.at<float>(1));//k2
   info_cam.D.push_back(D1.at<float>(2));//t1
   info_cam.D.push_back(D1.at<float>(3));//t2
   info_cam.D.push_back(D1.at<float>(4));//k3

   //Intrinsic camera matrix
   info_cam.K[0]=M1.at<float>(0,0);//fx
   info_cam.K[1]=M1.at<float>(0,1);
   info_cam.K[2]=M1.at<float>(0,2);//cx
   info_cam.K[3]=M1.at<float>(1,0);
   info_cam.K[4]=M1.at<float>(1,1);//fy
   info_cam.K[5]=M1.at<float>(1,2);//cy
   info_cam.K[6]=M1.at<float>(2,0);
   info_cam.K[7]=M1.at<float>(2,1);
   info_cam.K[8]=M1.at<float>(2,2);

   info_cam.R[0]=1.0;
   info_cam.R[1]=0.0;
   info_cam.R[2]=0.0;
   info_cam.R[3]=0.0;
   info_cam.R[4]=1.0;
   info_cam.R[5]=0.0;
   info_cam.R[6]=0.0;
   info_cam.R[7]=0.0;
   info_cam.R[8]=1.0;

   info_cam.distortion_model="plumb_bob";

   info_cam.binning_x=0;
   info_cam.binning_y=0;
   info_cam.roi.x_offset=0;
   info_cam.roi.y_offset=0;
   info_cam.roi.do_rectify=false;

   info_cam.width = width;
   info_cam.height = height;

   info_cam.roi.width=info_cam.width;
   info_cam.roi.height=info_cam.height;

   //Projection camera matrix
   info_cam.P[0]=M1.at<float>(0,0); //fx
   info_cam.P[1]=0;
   info_cam.P[2]=M1.at<float>(0,2); //cx
   info_cam.P[3]=0;        //Tx
   info_cam.P[4]=0;
   info_cam.P[5]=M1.at<float>(1,1); //fy
   info_cam.P[6]=M1.at<float>(1,2); //cy
   info_cam.P[7]=0;        //Ty
   info_cam.P[8]=0;
   info_cam.P[9]=0;
   info_cam.P[10]=1;       //1.0
   info_cam.P[11]=0;

   fs.release();
   return true;
}

bool loadExtrinsicFromYAML(const std::string filename, sensor_msgs::CameraInfo &info_caml, sensor_msgs::CameraInfo &info_camr, Mat& R1, Mat& P1, Mat& R2, Mat& P2, Mat& M1, Mat& M2)
{
   Mat R,T,Quick;   
   Eigen::Matrix3d rot;
   cv::FileStorage fs(filename, cv::FileStorage::READ);

   if (!fs.isOpened())
   {
      std::cout<<"error"<<std::endl;
      return false;
   }
   cv::FileNode n = fs["transform"];
   double qx = static_cast<double>(n["q_x"]);
   double qy = static_cast<double>(n["q_y"]);
   double qz = static_cast<double>(n["q_z"]);
   double qw = static_cast<double>(n["q_w"]);
   double tx = static_cast<double>(n["t_x"]);
   double ty = static_cast<double>(n["t_y"]);
   double tz = static_cast<double>(n["t_z"]);

   fromQuaternionToRotationMatrix(qx,qy,qz,qw,rot);
   cv::eigen2cv(rot,R);
   T.push_back(tx);
   T.push_back(ty);
   T.push_back(tz);

   stereoRectify(M1, cv::Mat::zeros(1, 5, CV_32F), M2, cv::Mat::zeros(1, 5, CV_32F), Size(1296,966), R, T, R1, R2, P1, P2, Quick, CALIB_ZERO_DISPARITY, -1);

//    if(P2.at<double>(0,3)==0)
   P2.at<double>(0,3) = P2.at<double>(0,0)*T.at<double>(0,0);

   //Rectification matrix
   info_caml.R[0]=R1.at<double>(0,0);
   info_caml.R[1]=R1.at<double>(0,1);
   info_caml.R[2]=R1.at<double>(0,2);
   info_caml.R[3]=R1.at<double>(1,0);
   info_caml.R[4]=R1.at<double>(1,1);
   info_caml.R[5]=R1.at<double>(1,2);
   info_caml.R[6]=R1.at<double>(2,0);
   info_caml.R[7]=R1.at<double>(2,1);
   info_caml.R[8]=R1.at<double>(2,2);

   info_camr.R[0]=R2.at<double>(0,0);
   info_camr.R[1]=R2.at<double>(0,1);
   info_camr.R[2]=R2.at<double>(0,2);
   info_camr.R[3]=R2.at<double>(1,0);
   info_camr.R[4]=R2.at<double>(1,1);
   info_camr.R[5]=R2.at<double>(1,2);
   info_camr.R[6]=R2.at<double>(2,0);
   info_camr.R[7]=R2.at<double>(2,1);
   info_camr.R[8]=R2.at<double>(2,2);

   //Projection camera matrix
   info_caml.P[0]=P1.at<double>(0,0);//fx
   info_caml.P[1]=P1.at<double>(0,1);
   info_caml.P[2]=P1.at<double>(0,2);//cx
   info_caml.P[3]=P1.at<double>(0,3);//Tx
   info_caml.P[4]=P1.at<double>(1,0);
   info_caml.P[5]=P1.at<double>(1,1);//fy
   info_caml.P[6]=P1.at<double>(1,2);//cy
   info_caml.P[7]=P1.at<double>(1,3);//Ty
   info_caml.P[8]=P1.at<double>(2,0);
   info_caml.P[9]=P1.at<double>(2,1);
   info_caml.P[10]=P1.at<double>(2,2);
   info_caml.P[11]=P1.at<double>(2,3);//1.0

   info_camr.P[0]=P2.at<double>(0,0);//fx
   info_camr.P[1]=P2.at<double>(0,1);
   info_camr.P[2]=P2.at<double>(0,2);//cx
   info_camr.P[3]=P2.at<double>(0,3);//Tx
   info_camr.P[4]=P2.at<double>(1,0);
   info_camr.P[5]=P2.at<double>(1,1);//fy
   info_camr.P[6]=P2.at<double>(1,2);//cy
   info_camr.P[7]=P2.at<double>(1,3);//Ty
   info_camr.P[8]=P2.at<double>(2,0);
   info_camr.P[9]=P2.at<double>(2,1);
   info_camr.P[10]=P2.at<double>(2,2);
   info_camr.P[11]=P2.at<double>(2,3);//1.0

   fs.release();
   return true;
}
*/