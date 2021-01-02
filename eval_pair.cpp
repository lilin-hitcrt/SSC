#include <iostream>
#include <string>
#include <yaml-cpp/yaml.h>
#include <sys/time.h>
#include "ssc.h"
int main(){
    struct timeval time_t;
    double time1,time2;
    std::string conf_file="../config/config.yaml";
    auto data_cfg = YAML::LoadFile(conf_file);
    auto cloud_file1=data_cfg["eval_pair"]["cloud_file1"].as<std::string>();
    auto cloud_file2=data_cfg["eval_pair"]["cloud_file2"].as<std::string>();
    auto label_file1=data_cfg["eval_pair"]["label_file1"].as<std::string>();
    auto label_file2=data_cfg["eval_pair"]["label_file2"].as<std::string>();
    SSC ssc(conf_file);
    double angle = 0;
    float diff_x=0, diff_y=0;
    gettimeofday(&time_t,NULL);
    time1=time_t.tv_sec*1e3+time_t.tv_usec*1e-3;
    auto score=ssc.getScore(cloud_file1,cloud_file2,label_file1,label_file2,angle,diff_x,diff_y);
    gettimeofday(&time_t,NULL);
    time2=time_t.tv_sec*1e3+time_t.tv_usec*1e-3;
    std::cout<<"use time:"<<time2-time1<<"ms\n";
    std::cout<<"score:"<<score<<" "<<angle<<" "<<diff_x<<" "<<diff_y<<std::endl;
    return 0;
}