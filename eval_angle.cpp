#include <iostream>
#include <string>
#include <yaml-cpp/yaml.h>
#include "ssc.h"
int main(){
    struct timeval time_t;
    double time1,time2;
    std::string conf_file="../config/config.yaml";
    auto data_cfg = YAML::LoadFile(conf_file);
    auto cloud_path=data_cfg["eval_seq"]["cloud_path"].as<std::string>();
    auto label_path=data_cfg["eval_seq"]["label_path"].as<std::string>();
    auto pairs_file=data_cfg["eval_seq"]["pairs_file"].as<std::string>();
    auto calib_file=data_cfg["eval_seq"]["calib_file"].as<std::string>();
    auto pose_file=data_cfg["eval_seq"]["pose_file"].as<std::string>();
    auto out_file=data_cfg["eval_seq"]["out_file"].as<std::string>();
    SSC ssc(conf_file);
    std::ifstream f_pairs(pairs_file);
    std::ifstream f_calib(calib_file);
    std::ifstream f_pose(pose_file);
    std::ofstream f_out(out_file);
    Eigen::Isometry3f T=Eigen::Isometry3f::Identity();
    std::cout<<"load calib......"<<std::endl;
    for(int j=0;j<5;++j){
        std::string c;
        f_calib>>c;
        for(int i=0;i<12;++i){
            float temp;
            f_calib>>temp;
            if(j==4){
                T(i/4,i%4)=temp;
            }
        }
    }
    f_calib.close();
    std::vector<Eigen::Isometry3f,Eigen::aligned_allocator<Eigen::Isometry3f> > poses;
    std::vector<float> temp_pose;
    std::cout<<"load pose......"<<std::endl;
    while(1){
        float v=10000000;
        f_pose>>v;
        if(v==10000000){
            break;
        }
        temp_pose.emplace_back(v);
    }
    f_pose.close();
    if(temp_pose.size()%12!=0){
        std::cerr<<"pose size error:"<<temp_pose.size()<<" "<<temp_pose.size()/12<<" "<<temp_pose.size()%12;
    }
    std::cout<<"convert pose......"<<std::endl;
    for(int i=0;i<temp_pose.size();++i){
        int num_id=i%12;
        if(num_id==0){
            // std::cout<<"convert "<<poses.size()<<std::endl;
            poses.emplace_back(Eigen::Isometry3f::Identity());
        }
        poses.back()(num_id/4,num_id%4)=temp_pose[i];
    }
    int num = 1;
    float total_yaw=0,total_x=0,total_y=0;
    while (1)
    {
        std::string sequ1, sequ2;
        int label;
        f_pairs >> sequ1;
        f_pairs >> sequ2;
        f_pairs >> label;
        if(label==0){
            break;
        }
        if (sequ1.empty() || sequ2.empty())
        {
            break;
        }
        while (sequ1.size() < 6)
        {
            sequ1 = "0" + sequ1;
        }
        while (sequ2.size() < 6)
        {
            sequ2 = "0" + sequ2;
        }
        std::string cloud_file1, cloud_file2, sem_file1, sem_file2;
        cloud_file1 = cloud_path + sequ1;
        cloud_file1 = cloud_file1 + ".bin";
        cloud_file2 = cloud_path + sequ2;
        cloud_file2 = cloud_file2 + ".bin";
        sem_file1 = label_path + sequ1;
        sem_file1 = sem_file1 + ".label";
        sem_file2 = label_path + sequ2;
        sem_file2 = sem_file2 + ".label";
        gettimeofday(&time_t, NULL);
        time1 = time_t.tv_sec * 1e3 + time_t.tv_usec * 1e-3;
        double angle = 0;
        float diff_x=0, diff_y=0;
        auto score = ssc.getScore(cloud_file1, cloud_file2,sem_file1,sem_file2, angle,diff_x,diff_y);
        gettimeofday(&time_t, NULL);
        time2 = time_t.tv_sec * 1e3 + time_t.tv_usec * 1e-3;
        // std::cout <<num<<" "<<angle*180./M_PI<<" "<<diff_x<<" "<<diff_y << std::endl;
        auto pose1=T.inverse()*poses[atoi(sequ1.c_str())]*T;
        auto pose2=T.inverse()*poses[atoi(sequ2.c_str())]*T;
        auto d_pose=pose1.inverse()*pose2;
        float yaw=atan2(d_pose(1,0),d_pose(0,0));
        if(yaw<0){
            yaw+=2*M_PI;
        }
        f_out << yaw << std::endl;
        float error_yaw=fabs((yaw-angle)*180./M_PI);
        if(error_yaw>180){
            error_yaw=360-error_yaw;
        }
        total_yaw+=error_yaw;
        total_x+=fabs(d_pose(0,3)-diff_x);
        total_y+=fabs(d_pose(1,3)-diff_y);
        std::cout<<"pose error:"<<error_yaw<<" "<<fabs(d_pose(0,3)-diff_x)<<" "<<fabs(d_pose(1,3)-diff_y)<<std::endl;
        num++;
    }
    std::cout<<"average:"<<total_yaw/(num-1.0)<<" "<<total_x/(num-1.0)<<" "<<total_y/(num-1.0)<<std::endl;
    return 0;
}