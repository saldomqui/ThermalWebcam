#include "map_interpolator.h"

// For file map metadata reading
#include "yaml-cpp/yaml.h"

#include <fstream>
#include <sys/stat.h>

namespace MAP_INP
{
    void initUndistortRectifyMap(parameters &params, cv::Mat &map1, cv::Mat &map2)
    {
        vector<MAP_INP::remapIp> inp_model;

        map1 = cv::Mat(params.imgSize, CV_32FC1);
        map2 = cv::Mat(params.imgSize, CV_32FC1);

        buildInterpolatedModel(params.objpoints, params.imgpoints, inp_model);

        for (int i = 0; i < params.imgSize.width; i++)
        {
            MAP_INP::remapIp y_inp;

            computeYinterpolator(i, inp_model, y_inp);

            for (int j = 0; j < params.imgSize.height; j++)
            {
                map1.at<float>(j, i) = float(y_inp.sp_img_x(double(j)));
                map2.at<float>(j, i) = float(y_inp.sp_img_y(double(j)));
                // cout << "(" << i << "," << j << ") -> (" << map1.at<float>(j, i) << "," << map2.at<float>(j, i) << ")" << endl;
            }
        }
    }

    // undistortion function for a vector of points
    void rectifyImagePoints(const std::vector<cv::Point2f> &src, std::vector<cv::Point2f> &dst, const cv::Mat &mapx, const cv::Mat &mapy)
    {
        dst.resize(src.size());

        for (size_t i = 0; i < src.size(); ++i)
        {
            const cv::Point2f &p = src.at(i);

            rectifyImagePoint(p, dst[i], mapx, mapy);
        }
    }

    // undistortion function for a single point
    void rectifyImagePoint(const cv::Point2f &src, cv::Point2f &dst, const cv::Mat &mapx, const cv::Mat &mapy)
    {
        dst.x = mapx.at<float>(int(src.y), int(src.x));
        dst.y = mapy.at<float>(int(src.y), int(src.x));
    }

    void rectifyImage(const cv::Mat &img_distorted, cv::Mat &img_rectified, const cv::Mat &mapx, const cv::Mat &mapy)
    {
        cv::remap(img_distorted, img_rectified, mapx, mapy, cv::INTER_LINEAR);
    }

    void buildInterpolatedModel(const vector<vector<cv::Point2f>> &objpts, const vector<vector<cv::Point2f>> &imgpts, vector<MAP_INP::remapIp> &inp_model)
    {
        vector<double> obj_x_list;
        vector<double> obj_y_list;
        vector<double> img_x_list;
        vector<double> img_y_list;

        for (int j = 0; j < objpts.size(); j++) // for all rows
        {
            if (objpts[j].size() >= 3) // Must have at least 3 points
            {
                MAP_INP::remapIp rmap;

                obj_x_list.clear();
                obj_y_list.clear();
                img_x_list.clear();
                img_y_list.clear();

                for (int i = 0; i < objpts[j].size(); i++) // for all points in the row
                {
                    obj_x_list.push_back(double(objpts[j][i].x));
                    obj_y_list.push_back(double(objpts[j][i].y));
                    img_x_list.push_back(double(imgpts[j][i].x));
                    img_y_list.push_back(double(imgpts[j][i].y));
                }

                rmap.sp_obj_y.set_points(obj_x_list, obj_y_list);
                rmap.sp_img_x.set_points(obj_x_list, img_x_list);
                rmap.sp_img_y.set_points(obj_x_list, img_y_list);

                inp_model.push_back(rmap);
            }
        }

        return;
    }

    void computeYinterpolator(int x, const vector<MAP_INP::remapIp> &inp_model, MAP_INP::remapIp &y_rmap)
    {
        vector<double> obj_y_list;
        vector<double> img_x_list;
        vector<double> img_y_list;

        for (int i = 0; i < inp_model.size(); i++)
        {
            obj_y_list.push_back(inp_model[i].sp_obj_y(double(x)));
            img_x_list.push_back(inp_model[i].sp_img_x(double(x)));
            img_y_list.push_back(inp_model[i].sp_img_y(double(x)));
        }
        y_rmap.sp_img_x.set_points(obj_y_list, img_x_list);
        y_rmap.sp_img_y.set_points(obj_y_list, img_y_list);
    }

    bool doesFolderExists(const string &folder)
    {
        struct stat sb;

        if (stat(folder.c_str(), &sb) == 0 && S_ISDIR(sb.st_mode))
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    bool saveCalibrationMapsBinary(const string &path, const cv::Mat &mapx, const cv::Mat &mapy)
    {
        cout << "saving calibration maps on folder:" << path << endl;
        if (!mapx.isContinuous())
        {
            std::cout << "Not implemented yet" << std::endl;
            return false;
        }

        if (!mapy.isContinuous())
        {
            std::cout << "Not implemented yet" << std::endl;
            return false;
        }

        if (!doesFolderExists(path))
        {
            cout << "cam_calibration:" << path << " DOESN'T EXISTS. Creating it" << endl;
            mkdir(path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
        }

        int elemSizeInBytes = (int)mapx.elemSize();
        int elemType = (int)mapx.type();
        int dataSize = (int)(mapx.cols * mapx.rows * mapx.elemSize());

        FILE *FP = fopen((path + "/calib_mapx.bin").c_str(), "wb");
        int sizeImg[4] = {mapx.cols, mapx.rows, elemSizeInBytes, elemType};
        fwrite(sizeImg, 4, sizeof(int), FP);
        fwrite(mapx.data, mapx.cols * mapx.rows, elemSizeInBytes, FP);
        fclose(FP);

        FP = fopen((path + "/calib_mapy.bin").c_str(), "wb");
        fwrite(sizeImg, 4, sizeof(int), FP);
        fwrite(mapy.data, mapy.cols * mapy.rows, elemSizeInBytes, FP);
        fclose(FP);
        return true;
    }

    bool loadCalibrationMapFromYAML(const string &path, parameters &params)
    {
        cout << "cam_calibration: opening camera calibration map YAML file:" << path + "/intrinsic_calibration.yaml" << endl;
        YAML::Node doc = YAML::LoadFile((path + "/intrinsic_calibration.yaml").c_str());

        vector<int> img_size = doc["img_size"].as<vector<int>>();
        cout << "img_size: width:" << img_size[0] << " height:" << img_size[1] << endl;
        params.imgSize.width = img_size[0];
        params.imgSize.height = img_size[1];

        const YAML::Node &cal_pts_node = doc["cal_pts"];

        params.objpoints.clear();
        params.imgpoints.clear();

        for (std::size_t i = 0; i < cal_pts_node.size(); i++)
        {
            const YAML::Node &row_pts_node = cal_pts_node[i]["row"];
            vector<cv::Point2f> row_obj_pts;
            vector<cv::Point2f> row_img_pts;

            // cout << "row:" << i << endl;
            for (std::size_t j = 0; j < row_pts_node.size(); j++)
            {
                try
                {
                    vector<int> obj = row_pts_node[j]["obj"].as<vector<int>>();
                    vector<int> img = row_pts_node[j]["img"].as<vector<int>>();
                    // cout << "pt:" << j << " obj[" << obj[0] << "," << obj[1] << "], img[" << img[0] << "," << img[1] << "]" << endl;
                    row_obj_pts.push_back(cv::Point2f(float(obj[0]), float(obj[1])));

                    row_img_pts.push_back(cv::Point2f(float(img[0]), float(img[1])));
                }
                catch (YAML::InvalidNode)
                {
                    cout << "saga_fisheyecam_calibration: Can't read calibration point:" << i << endl;
                    return false;
                }
                catch (YAML::InvalidScalar &ex)
                {
                    cout << "saga_fisheyecam_calibration: Can't read scalar on node:" << i << endl;
                    return false;
                }
            }

            params.objpoints.push_back(row_obj_pts);
            params.imgpoints.push_back(row_img_pts);
        }

        return true;
    }

    bool loadCalibrationBinaryMap(const string &path, cv::Mat &mapx, cv::Mat &mapy)
    {
        FILE *fp = fopen((path + "/calib_mapx.bin").c_str(), "rb");
        int header[4];
        size_t bytes_read = fread(header, sizeof(int), 4, fp);
        int cols = header[0];
        int rows = header[1];
        int elemSizeInBytes = header[2];
        int elemType = header[3];

        std::cout << "rows=" << rows << " cols=" << cols << " elemSizeInBytes=" << elemSizeInBytes << std::endl;

        mapx = cv::Mat::ones(rows, cols, elemType);

        bytes_read = fread(mapx.data, elemSizeInBytes, (size_t)(cols * rows), fp);

        if (bytes_read != (size_t)(cols * rows))
        {
            fputs("Reading error", stderr);
            return false;
        }

        fclose(fp);

        fp = fopen((path + "/calib_mapy.bin").c_str(), "rb");
        bytes_read = fread(header, sizeof(int), 4, fp);
        cols = header[0];
        rows = header[1];
        elemSizeInBytes = header[2];
        elemType = header[3];

        std::cout << "rows=" << rows << " cols=" << cols << " elemSizeInBytes=" << elemSizeInBytes << std::endl;

        mapy = cv::Mat::ones(rows, cols, elemType);

        bytes_read = fread(mapy.data, elemSizeInBytes, (size_t)(cols * rows), fp);

        if (bytes_read != (size_t)(cols * rows))
        {
            fputs("Reading error", stderr);
            return false;
        }

        fclose(fp);
        return true;
    }
}
