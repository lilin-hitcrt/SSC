#include <iostream>
#include <string>
#include <yaml-cpp/yaml.h>
#include "ssc.h"
void zfill(std::string& in_str,int len){
        while (in_str.size() < len)
        {
            in_str = "0" + in_str;
        }
}
int main(int argc,char** argv){
    std::string conf_file="../config/config_kitti.yaml";
    if(argc>1){
        conf_file=argv[1];
    }
    auto data_cfg = YAML::LoadFile(conf_file);
    auto cloud_path=data_cfg["eval_seq"]["cloud_path"].as<std::string>();
    auto label_path=data_cfg["eval_seq"]["label_path"].as<std::string>();
    auto pairs_file=data_cfg["eval_seq"]["pairs_file"].as<std::string>();
    auto out_file=data_cfg["eval_seq"]["out_file"].as<std::string>();
    auto file_name_length=data_cfg["file_name_length"].as<int>();
    SSC ssc(conf_file);
    std::ifstream f_pairs(pairs_file);
    std::ofstream f_out(out_file);
    int num = 1;
    while (1)
    {
        std::string sequ1, sequ2;
        int label;
        f_pairs >> sequ1;
        f_pairs >> sequ2;
        f_pairs >> label;
        if (sequ1.empty() || sequ2.empty())
        {
            break;
        }
        zfill(sequ1,file_name_length);
        zfill(sequ2,file_name_length);
        std::string cloud_file1, cloud_file2, sem_file1, sem_file2;
        cloud_file1 = cloud_path + sequ1;
        cloud_file1 = cloud_file1 + ".bin";
        cloud_file2 = cloud_path + sequ2;
        cloud_file2 = cloud_file2 + ".bin";
        sem_file1 = label_path + sequ1;
        sem_file1 = sem_file1 + ".label";
        sem_file2 = label_path + sequ2;
        sem_file2 = sem_file2 + ".label";
        // double angle = 0;
        // float diff_x=0, diff_y=0;
        // auto score = ssc.getScore(cloud_file1, cloud_file2,sem_file1,sem_file2, angle,diff_x,diff_y);
        // f_out << score << " " << label << std::endl;
        // std::cout <<num<< " " << angle*180./M_PI<<" "<<diff_x<<" "<<diff_y << " " << score << " " << label << std::endl;
        Eigen::Matrix4f transform;
        auto score=ssc.getScore(cloud_file1,cloud_file2,sem_file1,sem_file2,transform);//refine angle
        f_out << score << " " << label << std::endl;
        std::cout<<num<<" "<<score<<" "<<label<<std::endl;
        num++;
    }
    return 0;
}