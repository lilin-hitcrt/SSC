#pragma once
#include <iostream>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <opencv2/opencv.hpp>
#include <sys/time.h>
#include <yaml-cpp/yaml.h>
#include<random>
#define SHOW 0
class SSC
{
private:
    std::vector<int> order_vec = {0, 0, 0, 0, 0, 0, 0, 0, 0, 10, 11, 12, 13, 15, 16, 14, 17, 9, 18, 19};
    typedef std::tuple<u_char, u_char, u_char> Color;
    std::map<uint32_t, Color> _color_map, _argmax_to_rgb;
    YAML::Node learning_map;
    std::vector<int> label_map;
    double max_dis=50;
    double min_dis=5;
    int rings=24;
    int sectors=360;
    int sectors_range=360;
    bool rotate=false;
    bool occlusion=false;
    bool remap=true;
    std::shared_ptr<std::default_random_engine> random_generator;
    std::shared_ptr<std::uniform_int_distribution<int> > random_distribution;
    struct timeval time_t;
    bool show=false;
    std::shared_ptr<pcl::visualization::CloudViewer> viewer;
    int fastAtan2(float y,float x);
public:
    SSC(std::string conf_file);
    ~SSC();
    double getScore(pcl::PointCloud<pcl::PointXYZL>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZL>::Ptr cloud2, Eigen::Matrix4f& trans);
    double getScore(std::string cloud_file1, std::string cloud_file2, std::string label_file1, std::string label_file2, Eigen::Matrix4f& transform);
    double getScore(std::string cloud_file1, std::string cloud_file2, Eigen::Matrix4f& transform);
    double getScore(pcl::PointCloud<pcl::PointXYZL>::Ptr cloud1, pcl::PointCloud<pcl::PointXYZL>::Ptr cloud2, double &angle,float& diff_x,float& diff_y);
    double getScore(std::string cloud_file1,std::string cloud_file2,std::string label_file1,std::string label_file2,double &angle,float& diff_x,float& diff_y);
    double getScore(std::string cloud_file1,std::string cloud_file2,double &angle,float& diff_x,float& diff_y);
    pcl::PointCloud<pcl::PointXYZL>::Ptr getLCloud(std::string file_cloud, std::string file_label);
    pcl::PointCloud<pcl::PointXYZL>::Ptr getLCloud(std::string file_cloud);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr getColorCloud(pcl::PointCloud<pcl::PointXYZL>::Ptr &cloud_in);
    cv::Mat calculateSSC( pcl::PointCloud<pcl::PointXYZL>::Ptr filtered_pointcloud);
    cv::Mat project(pcl::PointCloud<pcl::PointXYZL>::Ptr filtered_pointcloud);
    cv::Mat getColorImage(cv::Mat &desc);
    void globalICP(cv::Mat& isc_dis1,cv::Mat& isc_dis2,double &angle,float& diff_x,float& diff_y);
    Eigen::Matrix4f globalICP(cv::Mat &ssc_dis1, cv::Mat &ssc_dis2);
    double calculateSim(cv::Mat &desc1, cv::Mat &desc2);
};
