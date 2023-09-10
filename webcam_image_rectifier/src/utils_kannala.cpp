#include "utils_kannala.h"

//For configuration file
#include "yaml-cpp/yaml.h"

using namespace cv;
using namespace std;

namespace KANNALA
{

    template <typename T>
    T r(T k2, T k3, T k4, T k5, T theta)
    {
        // k1 = 1
        return theta +
               k2 * theta * theta * theta +
               k3 * theta * theta * theta * theta * theta +
               k4 * theta * theta * theta * theta * theta * theta * theta +
               k5 * theta * theta * theta * theta * theta * theta * theta * theta * theta;
    }

    bool readFromYamlFile(const std::string &filename,
                          parameters &param, double resolution_multiplier)
    {
        std::cout << "Calibration file:" << filename << std::endl;

        YAML::Node doc = YAML::LoadFile(filename.c_str());

        param.camera_name = doc["camera_name"].as<string>();

        param.imageWidth = resolution_multiplier * doc["image_width"].as<int>();
        param.imageHeight = resolution_multiplier * doc["image_height"].as<int>();

        cout << "image_width:" << param.imageWidth << " image_height:" << param.imageHeight << endl;

        vector<double> K = doc["camera_matrix"]["data"].as<vector<double>>();
        param.mu = static_cast<double>(K[0]);
        param.mv = static_cast<double>(K[4]);
        param.u0 = static_cast<double>(K[2]);
        param.v0 = static_cast<double>(K[5]);
        cout << "mu:" << param.mu << " mv:" << param.mv << " u0:" << param.u0 << " v0:" << param.v0 << endl;

        vector<double> D = doc["distortion_coefficients"]["data"].as<vector<double>>();
        param.k2 = static_cast<double>(D[1]);
        param.k3 = static_cast<double>(D[2]);
        param.k4 = static_cast<double>(D[3]);
        param.k5 = static_cast<double>(D[4]);
        cout << "k2:" << param.k2 << " k3:" << param.k3 << " k4:" << param.k4 << " k5:" << param.k5 << endl;

        vector<double> P = doc["projection_matrix"]["data"].as<vector<double>>();
        param.fx = resolution_multiplier * static_cast<double>(P[0]);
        param.fy = resolution_multiplier * static_cast<double>(P[5]);
        param.cx = resolution_multiplier * static_cast<double>(P[2]);
        param.cy = resolution_multiplier * static_cast<double>(P[6]);
        cout << "fx:" << param.fx << " fy:" << param.fy << " cx:" << param.cx << " cy:" << param.cy << endl;

        return true;
    }

    bool readParamFromCameraInfo(sensor_msgs::CameraInfo info_cam, parameters &param, double resolution_multiplier)
    {
        param.camera_name = info_cam.header.frame_id;

        param.imageWidth = int(resolution_multiplier * info_cam.width);
        param.imageHeight = int(resolution_multiplier * info_cam.height);

        cout << "image_width:" << param.imageWidth << " image_height:" << param.imageHeight << endl;

        param.mu = info_cam.K[0];
        param.mv = info_cam.K[4];
        param.u0 = info_cam.K[2];
        param.v0 = info_cam.K[5];
        cout << "mu:" << param.mu << " mv:" << param.mv << " u0:" << param.u0 << " v0:" << param.v0 << endl;

        param.k2 = info_cam.D[1];
        param.k3 = info_cam.D[2];
        param.k4 = info_cam.D[3];
        param.k5 = info_cam.D[4];
        cout << "k2:" << param.k2 << " k3:" << param.k3 << " k4:" << param.k4 << " k5:" << param.k5 << endl;

        param.fx = resolution_multiplier * info_cam.P[0];
        param.fy = resolution_multiplier * info_cam.P[5];
        param.cx = resolution_multiplier * info_cam.P[2];
        param.cy = resolution_multiplier * info_cam.P[6];
        cout << "fx:" << param.fx << " fy:" << param.fy << " cx:" << param.cx << " cy:" << param.cy << endl;

        return true;
    }

    bool loadIntrinsicFromYAML(const std::string filename, sensor_msgs::CameraInfo &info_cam, parameters &param, double resolution_multiplier)
    {
        if (!readFromYamlFile(filename, param, resolution_multiplier))
        {
            printf("Failed to open file %s\n", filename.c_str());
            return false;
        }

        //Distortion parameters
        info_cam.D.push_back(1);        //k1
        info_cam.D.push_back(param.k2); //k2
        info_cam.D.push_back(param.k3); //k3
        info_cam.D.push_back(param.k4); //k4
        info_cam.D.push_back(param.k5); //k5

        //Intrinsic camera matrix
        info_cam.K[0] = param.mu; //fx
        info_cam.K[1] = 0;
        info_cam.K[2] = param.u0; //cx
        info_cam.K[3] = 0;
        info_cam.K[4] = param.mv; //fy
        info_cam.K[5] = param.v0; //cy
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

        info_cam.distortion_model = "kannala";

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

        if (param.cx == -1.0f && param.cy == -1.0f)
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

    void spaceToPlane(const Eigen::Vector3d &P, Eigen::Vector2d &p,
                      parameters param)
    {
        double theta = acos(P(2) / P.norm());
        double phi = atan2(P(1), P(0));

        Eigen::Vector2d p_u = r(param.k2, param.k3, param.k4, param.k5, theta) * Eigen::Vector2d(cos(phi), sin(phi));

        // Apply generalised projection matrix
        p << param.mu * p_u(0) + param.u0,
            param.mv * p_u(1) + param.v0;
    }

    void backprojectSymmetric(const Eigen::Vector2d &p_u, double &theta, double &phi,
                              parameters param)
    {
        double tol = 1e-10;
        double p_u_norm = p_u.norm();

        if (p_u_norm < 1e-10)
        {
            phi = 0.0;
        }
        else
        {
            phi = atan2(p_u(1), p_u(0));
        }

        int npow = 9;
        if (param.k5 == 0.0)
        {
            npow -= 2;
        }
        if (param.k4 == 0.0)
        {
            npow -= 2;
        }
        if (param.k3 == 0.0)
        {
            npow -= 2;
        }
        if (param.k2 == 0.0)
        {
            npow -= 2;
        }

        Eigen::MatrixXd coeffs(npow + 1, 1);
        coeffs.setZero();
        coeffs(0) = -p_u_norm;
        coeffs(1) = 1.0;

        if (npow >= 3)
        {
            coeffs(3) = param.k2;
        }
        if (npow >= 5)
        {
            coeffs(5) = param.k3;
        }
        if (npow >= 7)
        {
            coeffs(7) = param.k4;
        }
        if (npow >= 9)
        {
            coeffs(9) = param.k5;
        }

        if (npow == 1)
        {
            theta = p_u_norm;
        }
        else
        {
            // Get eigenvalues of companion matrix corresponding to polynomial.
            // Eigenvalues correspond to roots of polynomial.
            Eigen::MatrixXd A(npow, npow);
            A.setZero();
            A.block(1, 0, npow - 1, npow - 1).setIdentity();
            A.col(npow - 1) = -coeffs.block(0, 0, npow, 1) / coeffs(npow);

            Eigen::EigenSolver<Eigen::MatrixXd> es(A);
            Eigen::MatrixXcd eigval = es.eigenvalues();

            std::vector<double> thetas;
            for (int i = 0; i < eigval.rows(); ++i)
            {
                if (fabs(eigval(i).imag()) > tol)
                {
                    continue;
                }

                double t = eigval(i).real();

                if (t < -tol)
                {
                    continue;
                }
                else if (t < 0.0)
                {
                    t = 0.0;
                }

                thetas.push_back(t);
            }

            if (thetas.empty())
            {
                theta = p_u_norm;
            }
            else
            {
                theta = *std::min_element(thetas.begin(), thetas.end());
            }
        }
    }

    void liftProjective(const Eigen::Vector2d &p, Eigen::Vector3d &P,
                        parameters param)
    {
        double inv_K11 = 1.0 / param.mu;
        double inv_K13 = -param.u0 / param.mu;
        double inv_K22 = 1.0 / param.mv;
        double inv_K23 = -param.v0 / param.mv;

        // Lift points to normalised plane
        Eigen::Vector2d p_u;
        p_u << inv_K11 * p(0) + inv_K13,
            inv_K22 * p(1) + inv_K23;

        // Obtain a projective ray
        double theta, phi;
        backprojectSymmetric(p_u, theta, phi, param);

        P(0) = sin(theta) * cos(phi);
        P(1) = sin(theta) * sin(phi);
        P(2) = cos(theta);
    }

    void rectifyImagePoints(const std::vector<cv::Point2f> &src, std::vector<cv::Point2f> &dst,
                            parameters param)
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

    void rectifyImagePoint(const cv::Point2f &src, cv::Point2f &dst,
                           parameters param)
    {
        Eigen::Vector3d P;

        liftProjective(Eigen::Vector2d(src.x, src.y), P, param);

        P /= P(2);

        dst.x = P(0) * param.fx + param.cx;
        dst.y = P(1) * param.fy + param.cy;
    }
}
