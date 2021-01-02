#pragma once
#include <iostream>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>
#include <opencv2/opencv.hpp>
#include <sys/time.h>
#include <yaml-cpp/yaml.h>
class SSC
{
private:
    std::vector<int> order_vec = {0, 0, 0, 0, 0, 0, 0, 0, 0, 10, 11, 12, 13, 15, 16, 14, 17, 9, 18, 19};
    typedef std::tuple<u_char, u_char, u_char> Color;
    std::map<uint32_t, Color> _color_map, _argmax_to_rgb;
    YAML::Node learning_map;
    std::vector<int> label_map;
    bool use_sk=true;

    pcl::PointCloud<pcl::PointXYZI>::Ptr getCloud(std::string file);
    void getLabel(std::string file, std::vector<uint32_t> &sem_labels, std::vector<uint32_t> &ins_labels);
    pcl::PointCloud<pcl::PointXYZL>::Ptr getLCloud(std::string file_cloud, std::string file_label);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getColorCloud(pcl::PointCloud<pcl::PointXYZL>::Ptr &cloud_in);
    void calculate_ssc_range(const pcl::PointCloud<pcl::PointXYZL>::Ptr filtered_pointcloud,cv::Mat3f& isc_dis);
    cv::Mat1b calculate_ssc(const pcl::PointCloud<pcl::PointXYZL>::Ptr filtered_pointcloud);
    cv::Mat3b getColorImage(cv::Mat1b &desc);
    void calculate_trans(cv::Mat3f& isc_dis1,cv::Mat3f& isc_dis2,double &angle,float& diff_x,float& diff_y);
    double calculate_dis(cv::Mat1b &desc1, cv::Mat1b &desc2);
    double getScore(pcl::PointCloud<pcl::PointXYZL>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZL>::Ptr cloud2, double &angle,float& diff_x,float& diff_y);
public:
    SSC(std::string conf_file);
    ~SSC();
    double getScore(std::string cloud_file1,std::string cloud_file2,std::string label_file1,std::string label_file2,double &angle,float& diff_x,float& diff_y);
};
