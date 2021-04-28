#include <iostream>
#include <string>
#include <yaml-cpp/yaml.h>
#include <pcl/io/pcd_io.h>
#include "ssc.h"
int main(){
    std::string conf_file="../config/config_kitti.yaml";
    auto data_cfg = YAML::LoadFile(conf_file);
    auto cloud_file1=data_cfg["eval_pair"]["cloud_file1"].as<std::string>();
    auto cloud_file2=data_cfg["eval_pair"]["cloud_file2"].as<std::string>();
    auto label_file1=data_cfg["eval_pair"]["label_file1"].as<std::string>();
    auto label_file2=data_cfg["eval_pair"]["label_file2"].as<std::string>();
    SSC ssc(conf_file);
    double angle = 0;
    float diff_x=0, diff_y=0;
    auto score=ssc.getScore(cloud_file1,cloud_file2,label_file1,label_file2,angle,diff_x,diff_y);
    // auto score=ssc.getScore(cloud_file1,cloud_file2,angle,diff_x,diff_y);
    std::cout<<"score:"<<score<<std::endl;
    std::cout<<"(x,y,yaw): ("<<diff_x<<", "<<diff_y<<", "<<angle*180./M_PI<<")"<<std::endl;
    return 0;
}