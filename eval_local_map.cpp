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
    auto out_file=data_cfg["eval_seq"]["out_file"].as<std::string>();
    SSC ssc(conf_file);
    std::ifstream f_pairs(pairs_file);
    std::ofstream f_out(out_file);
    int num = 1;
    while (1)
    {
        std::string sequ1;
        int sequ2=-1;
        int label;
        f_pairs >> sequ1;
        f_pairs >> sequ2;
        f_pairs >> label;
        if (sequ1.empty() || sequ2==-1)
        {
            break;
        }
        while (sequ1.size() < 6)
        {
            sequ1 = "0" + sequ1;
        }

        std::string cloud_file1, cloud_file2, sem_file1;
        cloud_file1 = cloud_path + sequ1;
        cloud_file1 = cloud_file1 + ".bin";
        cloud_file2 = "local_map"+std::to_string(sequ2)+".pcd";
        sem_file1 = label_path + sequ1;
        sem_file1 = sem_file1 + ".label";
        gettimeofday(&time_t, NULL);
        time1 = time_t.tv_sec * 1e3 + time_t.tv_usec * 1e-3;
        double angle = 0;
        float diff_x=0, diff_y=0;
        auto cloud1=ssc.getLCloud(cloud_file1,sem_file1);
        auto cloud2=ssc.getLCloud(cloud_file2);
        auto score = ssc.getScore(cloud1, cloud2, angle,diff_x,diff_y);
        gettimeofday(&time_t, NULL);
        time2 = time_t.tv_sec * 1e3 + time_t.tv_usec * 1e-3;
        f_out << score << " " << label << std::endl;
        std::cout <<num<<" "<<time2-time1<< " " << angle<<" "<<diff_x<<" "<<diff_y << " " << score << " " << label << std::endl;
        num++;
    }
    return 0;
}